#include <Wire.h>
#include "esp_task_wdt.h"

//user defines
#define USE_MPH true

//dev defines
#define I2C_DEV_ADDR 0x45 //ESP32 acts as I2C peripheral/slave. VESC controller/master polls for updates using lispBM script.
#define CORE_0 0
#define CORE_1 1
#define UART_STACK_SIZE 147456
#define BUTTON_STACK_SIZE 16384
#define I2C_STACK_SIZE 4096
#define BUTTON_PIN A1 //Max G2 button is super noisy... need to use a big ol capacitor and ADC input to reliably detect button down
#define HORN_PIN D7

using namespace std;

static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

uint8_t rxArrayI2C[8];
uint8_t rxArrayUART[16];
uint8_t txArrayI2C[8];
uint8_t txArrayUART[17];
uint8_t packetHeader[3];
uint8_t junkArray[17];
volatile bool needToTransmitCruiseToVESC = false;
volatile uint8_t currentSpeed;
volatile uint8_t batteryStateOfCharge;
volatile int tempMotor;
volatile int tempFET;
volatile uint8_t throttle = 0;
volatile uint8_t brake = 0;
volatile uint8_t aux = 0x40;
volatile bool isLightOn = false;
volatile uint8_t speedMode = 0x04;
volatile bool isCharging = 0;
volatile bool isCruiseOn = 0;
volatile uint8_t beep = 0;
volatile bool needToTransmitBeepToDash = false;
volatile bool isOff = true;
volatile bool isLocked = true;
volatile bool isOverheated = false;
volatile bool hornState; //0 is off/LOW, 1 is off/HIGH

TaskHandle_t handleUART;
TaskHandle_t handleButton;
TaskHandle_t handleI2C;

void buttonLoop(void*);
int buttonPressed(TickType_t, int);
void calculateChecksum(uint8_t*, int);
void communicateWithDashboard();
void initializeTxArray(uint8_t*);
uint16_t ninebotChecksumToUint16(uint8_t, uint8_t);
void IRAM_ATTR onReceive(int);
void IRAM_ATTR onRequest();
void setupI2C(void*);
void uartLoop(void*);
uint8_t updateSpeedMode();
bool verifyChecksum(uint16_t, uint8_t*, int);

void setup() {
  //initialize byte array with protocol header, etc. for response to dashboard
  initializeTxArray(txArrayUART);
  
  //UART to computer (Serial) and BLE (Serial1)
  Serial.begin(115200);
  Serial.setTimeout(1);
  Serial1.begin(115200, SERIAL_8N1, D4, D3);

  //horn pin goes to gate of 2N7000 FET, which then sends 12v to the horn
  pinMode(HORN_PIN, OUTPUT);
  digitalWrite(HORN_PIN, LOW);
  hornState = 0;

  xTaskCreatePinnedToCore(setupI2C, "Begin I2C to VESC", I2C_STACK_SIZE, NULL, 31, &handleI2C, CORE_1);
  xTaskCreatePinnedToCore(buttonLoop, "Button handling", BUTTON_STACK_SIZE, NULL, 1, &handleButton, CORE_0);
  xTaskCreatePinnedToCore(uartLoop, "UART comms", UART_STACK_SIZE, NULL, 16, &handleUART, CORE_0);
}

void loop() {
}

void setupI2C(void* pvParam) {
  //I2C to VESC
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);

  vTaskDelay(1000 / portTICK_PERIOD_MS);

  vTaskDelete(handleI2C);
}

void IRAM_ATTR onReceive(int len) {
  uint8_t bytesReceived = 0;
  uint16_t checksum;

  while (Wire.available()) {
    rxArrayI2C[bytesReceived] = Wire.read();
    ++bytesReceived;
  }
  if (bytesReceived == 8) {
    checksum = ninebotChecksumToUint16(rxArrayI2C[6], rxArrayI2C[7]);
    if (verifyChecksum(checksum, rxArrayI2C, 6)) {
      taskENTER_CRITICAL_ISR(&spinlock);
      currentSpeed = ((float)rxArrayI2C[0] / 10); //speed transmitted in units of (m/s * 10) to preserve precision within 8 bits
      if (USE_MPH) {
        currentSpeed *= 2.236936; //conversion factor for mph
      } else {
        currentSpeed *= 3.6; //else convert to km/h
      }
      currentSpeed = (uint8_t)round(currentSpeed);
      batteryStateOfCharge = rxArrayI2C[1]; //0-100 range from VESC BMS or voltage estimation, based on 'has-vesc-bms in lispBM script
      tempMotor = (int)rxArrayI2C[2] - 50; //temps sent offset by 50 degC to allow range between -50 and 205 degC within 8 bits
      tempFET = (int)rxArrayI2C[3] - 50;
      if (isOverheated && ((tempMotor <= 165) || (tempFET <= 80))) {
        isOverheated = !isOverheated;
      } else if (!isOverheated && ((tempMotor > 165) || (tempFET > 80))) {
        isOverheated = !isOverheated;
      }
      isCharging = (bool)(rxArrayI2C[4] & 1);
      isCruiseOn = (bool)((rxArrayI2C[4] & 2) >> 1);
      if (isCruiseOn) {
        needToTransmitCruiseToVESC = false;
      }
      taskEXIT_CRITICAL_ISR(&spinlock);
    }
  }
}

void IRAM_ATTR onRequest() {
  //access globals to populate outgoing array for VESC
  taskENTER_CRITICAL_ISR(&spinlock);
  if ((aux == 0x50) && (needToTransmitCruiseToVESC == false)) {
    aux = 0x40;
  }
  txArrayI2C[0] = throttle;
  txArrayI2C[1] = brake;
  txArrayI2C[2] = aux;
  txArrayI2C[3] = speedMode;
  txArrayI2C[4] = (isOff || isLocked);
  txArrayI2C[5] = 0;
  taskEXIT_CRITICAL_ISR(&spinlock);
  //checksum and send it
  calculateChecksum(txArrayI2C, 6);
  Wire.write(txArrayI2C, 8);
}

void buttonLoop(void* pvParam) {
  const int MS_DELAY = 45;
  const TickType_t TASK_DELAY = MS_DELAY / portTICK_PERIOD_MS;
  int outcome; //press type: 0-rejected, 1-single, 2-double, 3-long
  //esp_timer_get_time();

  pinMode(BUTTON_PIN, INPUT);

  vTaskDelay(1000 / portTICK_PERIOD_MS);

  for (;;) {
    esp_task_wdt_reset();
    vTaskDelay(TASK_DELAY);
    outcome = buttonPressed(TASK_DELAY, MS_DELAY);
    if (outcome == 0) {
      continue;
    }
    taskENTER_CRITICAL(&spinlock);
    if (outcome == 1) { //single press
      if (!isLocked) {
        isLightOn = !isLightOn;
      }
      if (isOff) {
        isOff = false;
      }
    } else if (outcome == 2 && !(isLocked || isOff)) { //double press
      if (speedMode == 4) {
        speedMode = (speedMode >> 1);
      } else if (speedMode == 2) {
        speedMode = (speedMode >> 1);
      } else if (speedMode == 1) {
        speedMode = (speedMode << 2);
      }
      //short beep for speed mode change
      if (!needToTransmitBeepToDash) {
        beep = 1;
        needToTransmitBeepToDash = 1;
      }
    } else if (outcome == 3) { //long press
      if (!isOff) {
        if (!isLocked) {
          //on and unlocked, change to off and locked
          isOff = true;
          isLocked = true;
        }
        //on and locked, change to on and unlocked
        isLocked = false;
      } else {
        //off and locked, change to on and locked
        isOff = false;
      }
    }
    taskEXIT_CRITICAL(&spinlock);
  }

  vTaskDelete(handleButton);
}
/*
Returns:
  0-Rejected input
  1-Single press
  2-Double press
  3-Long press
*/
int buttonPressed(TickType_t taskDelay, int msDelay) {
  const int POLLING_RATE = 15;
  const int DEBOUNCE_MS = 27;
  const int LOCKOUT_TIME = 35;
  const int DBL_PRESS_MS = 200;
  const int LONG_PRESS_MS = 2000;
  const int ACTIVE_POLLING_FACTOR = msDelay / POLLING_RATE; //polling after button press is detected will be faster than main loop by this factor

  const int ACTIVE_POLLING_MS_DELAY = msDelay / ACTIVE_POLLING_FACTOR;
  const TickType_t ACTIVE_POLLING_TASK_DELAY = taskDelay / ACTIVE_POLLING_FACTOR;
  const int DEBOUNCE_TIMEOUT = DEBOUNCE_MS / ACTIVE_POLLING_MS_DELAY;
  const int DBL_PRESS_TIMEOUT = DBL_PRESS_MS / ACTIVE_POLLING_MS_DELAY;
  const int LONG_PRESS_TIMEOUT = LONG_PRESS_MS / ACTIVE_POLLING_MS_DELAY;
  /*
  State:
    0-Waiting for input
    1-Waiting for debounce
    2-Input debounced
  */  
  int state = 0;
  int iterations = 0;
  int presses = 0;
  int firstPress = 0;
  bool buttonIsPressed;
  bool longPressBeepSent = 0;
  
  for (;;) {
    buttonIsPressed = (analogRead(BUTTON_PIN) < 1250);
    //input scan
    if (state == 0) {
      if (buttonIsPressed && (((iterations - firstPress) > (LOCKOUT_TIME / POLLING_RATE)) || !(firstPress > 0))) {
        state = 1;
        vTaskDelay(ACTIVE_POLLING_TASK_DELAY);
        ++iterations;
        //Serial.println("Input detected - moving to debounce");
        continue;
      } else {
        if (presses == 0) {
          return 0; //no input, keep scanning
        }
        if ((iterations - firstPress) > DBL_PRESS_TIMEOUT) {
          if (presses < 2) {
            return 1; //one debounced input
          } else {
            return 2; //two debounced inputs
          }
        }
      }
    //debounce
    } else if (state == 1) {
      if (buttonIsPressed) {
        if (iterations > DEBOUNCE_TIMEOUT) {
          state = 2;
          vTaskDelay(ACTIVE_POLLING_TASK_DELAY);
          ++iterations;
          continue;
        }
      } else {
        state = 0;
        continue;
      }
    //lift vs. long press
    } else if (state == 2) {
      if (buttonIsPressed) {  //button lifted after debounce
        taskENTER_CRITICAL(&spinlock);
        //double beep on unlock
        if (!longPressBeepSent && !needToTransmitBeepToDash && iterations > LONG_PRESS_TIMEOUT) {
          if (isLocked && !isOff) {
            beep = 3;  
          } else if (!isLocked && !isOff) {
            beep = 2;
          }
          needToTransmitBeepToDash = true;
          longPressBeepSent = true;
        }
        taskEXIT_CRITICAL(&spinlock);
      } else { 
        if (iterations > LONG_PRESS_TIMEOUT) {
          return 3;
        } else {
          ++presses;
          state = 0;
          if (presses == 1) {
            firstPress = iterations;
          }
          vTaskDelay(ACTIVE_POLLING_TASK_DELAY);
          ++iterations;
        }
      }
    }
    //otherwise keep iterating
    vTaskDelay(ACTIVE_POLLING_TASK_DELAY);
    ++iterations;
    esp_task_wdt_reset();
  }
}

void uartLoop(void* pvParam) {
  const TickType_t uartTaskDelay = 10 / portTICK_PERIOD_MS;
  
  for (;;) {
    if (Serial1.available()) {
      communicateWithDashboard();
    }
    esp_task_wdt_reset();
    vTaskDelay(uartTaskDelay);
  }

  vTaskDelete(handleUART);
}

void communicateWithDashboard() {
  uint16_t checksum;
  uint8_t bitmap;
  uint8_t lampStatus;
  
  //protocol header will be 0x5aa5 and then next byte is payload length
  Serial1.readBytes(packetHeader, 2);
  
  //check for protocol header
  if ((packetHeader[0] == 0x5a) && (packetHeader[1] == 0xa5)) {
    uint8_t dataLength = Serial1.peek();
    Serial1.readBytes(rxArrayUART, dataLength + 7);
    
    //check checksum
    checksum = ninebotChecksumToUint16(rxArrayUART[dataLength + 5], rxArrayUART[dataLength + 6]);
    if (verifyChecksum(checksum, rxArrayUART, dataLength + 5)) {
      //check if this is a packet from BLE to ESC
      if ((rxArrayUART[1] == 0x21) && (rxArrayUART[2] == 0x20)) {
        //if we have a successfully validated packet, then retrieve and store input data
        taskENTER_CRITICAL(&spinlock);
        throttle = rxArrayUART[6];
        brake = rxArrayUART[7];
        if (!needToTransmitCruiseToVESC) {
          aux = rxArrayUART[8];
        }
        if (!isLocked && !isOff) {
          if (aux == 0x50) {
            needToTransmitCruiseToVESC = true;
            if (!needToTransmitBeepToDash) {
              beep = 3;
              needToTransmitBeepToDash = 1;
            }
          }
          if (aux == 0x60 && hornState == 0) {
            digitalWrite(HORN_PIN, HIGH);
            hornState = 1;
          } else if (aux != 0x60 && hornState == 1) {
            digitalWrite(HORN_PIN, LOW);
            hornState = 0;
          }
        }
        taskEXIT_CRITICAL(&spinlock);
      }
    }
  }

  //If BLE sends 0x64, then send an update to the dash after parsing out input data
  if (rxArrayUART[3] == 0x64) {
    taskENTER_CRITICAL(&spinlock);
    
    //update with current values from VESC
    bitmap = updateSpeedMode();
    txArrayUART[7] = bitmap;
    txArrayUART[8] = batteryStateOfCharge;
    if (isLightOn) {
      lampStatus = lampStatus | 0x01; //flip lamp bit 1 to 1
    } else {
      lampStatus = lampStatus & 0xFE; //flip lamp bit 1 to 0
    }
    if (isCruiseOn) {
      lampStatus = lampStatus | 0x04; //flip lamp bit 3 to 1
    } else {
      lampStatus = lampStatus & 0xFB; //flip lamp bit 3 to 0
    }
    txArrayUART[9] = lampStatus;
    txArrayUART[10] = beep;
    txArrayUART[11] = currentSpeed;

    taskEXIT_CRITICAL(&spinlock);

    //calculate and store checksum
    calculateChecksum((uint8_t*)txArrayUART + 2, 13);

    //send it
    Serial1.write(txArrayUART, 17);
    
    //clear out data just sent from buffer because fake "half-duplex" shenanigans
    while (!Serial1.available()) {}
    Serial1.readBytes(junkArray, 17);

    taskENTER_CRITICAL(&spinlock);
    if (beep > 0) {
      beep = 0;
      needToTransmitBeepToDash = 0;
    }
    taskEXIT_CRITICAL(&spinlock);
  }
}

uint8_t updateSpeedMode() {
  uint8_t bitmap;
  bitmap += speedMode;
  bitmap += (isCharging << 3);
  bitmap += (isOff << 4);
  bitmap += (isLocked << 5);
  bitmap += (USE_MPH << 6);
  bitmap += (isOverheated << 7);
  return bitmap;
}

void initializeTxArray(uint8_t* array) {
  array[0] = 0x5a; //ninebot protocol header byte 1
  array[1] = 0xa5; //ninebot protocol header byte 2
  array[2] = 0x08; //payload length in bytes
  array[3] = 0x20; //source is ESC
  array[4] = 0x21; //destination is BLE
  array[5] = 0x64; //command 0x64 - send update from ESC
  array[6] = 0x00; //argument
  array[7] = speedMode; //speed mode (bitmap) - 1/normal, 2/eco, 4/sport, 8/charge, 16/off, 32/lock, 64/mph mode, 128/overheat
  array[8] = 0x00; //battery level
  array[9] = 0x00; //lamp status
  array[10] = 0x00; //beeper
  array[11] = 0x00; //current speed
  array[12] = 0x00; //error codes
  array[13] = 0x06; //don't know what this byte does
  array[14] = 0x2e; //or this one
  array[15] = 0x00; //checksum byte 1
  array[16] = 0x00; //checksum byte 2 
}

void calculateChecksum(uint8_t* data, int len) {
  uint16_t checksum = 0;
  for (uint8_t i = 0; i < len; i++) {
    checksum += data[i];
  }
  checksum = ~checksum;

  //checksum stored as little endian hex in last two bytes of packet
  data[len + 1] = (checksum >> 8);
  data[len] = checksum & 0x00ff;
}

bool verifyChecksum(uint16_t checksum, uint8_t* data, int len) {
  uint16_t sum = 0;
  //sum elements
  for (int i = 0; i < (len); ++i) {
    sum = sum + data[i];
  }
  //sum + checksum should yield bitwise all 1s
  if ((uint16_t)(~(sum + checksum)) == 0x0000) {
    return true;
  }
  return false;
}

uint16_t ninebotChecksumToUint16(uint8_t byteOne, uint8_t byteTwo) {
  return (byteTwo << 8) | byteOne;
}
