;~~~!!!!!!!~~~
;This is NOT for standalone usage with the BLE/dash wired directly into the VESC board!
;This is code to have the VESC communicate via I2C with an interstitial Arduino Nano ESP32 (or other similar microcontroller), which then handles UART communication with the dash
;VESC is I2C controller/master and ESP32 is peripheral/slave because VESC firmware 6.0.5 doesn't seen to have I2C peripheral support in lispBM scripting
;~~~!!!!!!!!~~~

;Limits for throttle and brake Hall sensor data, which will vary from machine to machine
;These are set according to what the BLE sends at zero/full throttle and zero/full brake for my personal Max G2
;Poll VT and monitor vt-throttle-raw and vt-brake-raw to see values for your scooter
;Include decimal so that we can do floating point division later on to normalize to 0-1 range for VESC current input
(def throttle-idle 36.0)
(def throttle-max 191.0)
(def brake-idle 39.0)
(def brake-max 167.0)

;Power limits for motor
(def max-amps 60.0) ;Maximum phase amps when cold
(def current-offset 20.0) ;Number of amps to reduce motor phase current by when hot
(def takeoff-assist-factor 0.15) ;Factor to boost max current by when below 2 m/s
;want to start phasing in current limit when above lower threshold, ramp up to full offset when above upper threshold
(def lower-current-limit-motor-temp 155.0)
(def upper-current-limit-motor-temp 170.0)
(def current-limit-denominator (- upper-current-limit-motor-temp lower-current-limit-motor-temp))
(def vt-current-limit max-amps)
(conf-set 'l-current-max vt-current-limit)

;for tuning, set capture-telemetry to 1 to capture performance data on I2C transaction rate
(def capture-telemetry 0)
(if (= capture-telemetry 1)
    {
        (def start-time (systime))
        (def number-of-tx 0)
        (def vt-tx-per-sec 0)
    }
)

;Bytes for Rx from ESP32
;  0-Throttle (0 to 255)
;  1-Brake (0 to 255)
;  2-Aux (0x40 for normal state, 0x50 for turn signal held for 3 sec., 0x60 for horn on)
;  3-Speed mode (bitmap) - 1/normal, 2/eco, 4/sport, 8/charge, 16/off, 32/lock, 64/mph mode, 128/overheat
;  4-Powered off (0-On/1-Off)
;  5-*not yet used
;  6-Checksum byte 1
;  7-Checksum byte 2
(def rx-buf (array-create 8))
(def checksum-buf (array-create 2))
(def vt-throttle-input 0)
(def vt-brake-input 0)
(def vt-throttle-raw 0)
(def vt-brake-raw 0)
(def vt-current-speed-mode 0)
(def vt-cruise 0)
(def vt-output-disabled 1)

;Store speed in tenths of m/s, rounded to nearest integer to preserve more precision
;(i.e., support up to 25.5 m/s with 8 unsigned bits)
;If we're going more than 57 mph on 10 inch tires, then we have bigger problems...
(def current-speed-encoded)

;If we have a VESC BMS connected via CAN, then use State of Charge reading from that
(def has-vesc-bms 1)
(def battery-soc)

;Temps offset by +50 to support range between -50 and 205 degC to transmit within 8 unsigned bits.
;If we're outside of that range, then we have bigger problems...
(def temp-motor-encoded)
(def temp-fet-encoded)

;Bytes for Tx to ESP32
;  0-Speed (encoded)
;  1-Battery state of charge
;  2-Motor temperature (encoded)
;  3-VESC MOSFET temperature (encoded)
;  4-Aux bitmap (1-charging, 2-cruise)
;  5-*not yet used
;  6-Checksum byte 1
;  7-Checksum byte 2
(def tx-arr (array-create 8))
(def aux-bitmap 0)
(def last-tx-time 0)
(def vt-failed-tx-ct 0)
(def vt-total-tx 0)
(def tx-since-last-refresh 0)

;Initialize brake light
(pwm-start 120 0.33)
(def brakelight-state 1) ;1-low, 2-high, 0-off

(app-adc-detach 3 1)

(defun i2c-comm()
    {
        (var checksum 0)
        (var erpm-when-cruise-enabled)
        (var throttle-when-cruise-enabled)
        (var time-when-cruise-enabled)

        ;need to make sure that we can more than keep up with dashboard inputs (~50Hz)
        (yield 6400)
        (if (= vt-output-disabled 1) (yield 250000))

        (if (= (mod tx-since-last-refresh 5) 0)
            {
                (bufset-u8 tx-arr 0 current-speed-encoded) ;speed
                (bufset-u8 tx-arr 1 battery-soc) ;battery
                (bufset-u8 tx-arr 2 temp-motor-encoded) ;motor temp
                (bufset-u8 tx-arr 3 temp-fet-encoded) ;MOSFET temp
                (bufset-u8 tx-arr 4 aux-bitmap) ;charging state bit 1, cruise state bit 2
                (calculate-checksum tx-arr)
                (set 'tx-since-last-refresh 0)
            }
        )
        (set 'vt-total-tx (+ vt-total-tx 1))
        (if (= (i2c-tx-rx 0x45 tx-arr rx-buf) 1)
            {
                (set 'tx-since-last-refresh (+ tx-since-last-refresh 1))
                ;need to reverse endian order of bytes for checksum
                (bufset-u8 checksum-buf 0 (bufget-u8 rx-buf 7))
                (bufset-u8 checksum-buf 1 (bufget-u8 rx-buf 6))
                (set 'checksum (bufget-u16 checksum-buf 0))
                (if (= (validate-checksum checksum rx-buf) 1)
                    {
                        (if (= capture-telemetry 1) (set 'number-of-tx (+ number-of-tx 1)))
                        (set 'last-tx-time (systime))
                        (let
                            ((throttle-raw (bufget-u8 rx-buf 0))
                             (brake-raw (bufget-u8 rx-buf 1))
                             (aux (bufget-u8 rx-buf 2))
                             (speed-mode (bufget-u8 rx-buf 3))
                             (output-disabled (bufget-u8 rx-buf 4))
                             (throttle-normalized 0.0)
                             (brake-normalized 0.0))
                            {
                                (set 'vt-throttle-raw throttle-raw)
                                (set 'vt-brake-raw brake-raw)
                                (if (= aux 0x50)
                                    {
                                        (set 'vt-cruise 1)
                                        (set 'aux-bitmap (bitwise-or aux-bitmap 0x02))
                                    }
                                )

                                ;Normalize and constrain quantized Hall sensor inputs to 0.0-1.0 decimal range
                                (set 'throttle-normalized (/ (- throttle-raw throttle-idle) (- throttle-max throttle-idle)))
                                (if (< throttle-normalized 0.02) (set 'throttle-normalized 0.0)) ;2% dead zone on start
                                (if (> throttle-normalized 1.0) (set 'throttle-normalized 1.0))

                                (set 'brake-normalized (/ (- brake-raw brake-idle) (- brake-max brake-idle)))
                                (if (< brake-normalized 0.02) (set 'brake-normalized 0.0))
                                (if (> brake-normalized 1.0) (set 'brake-normalized 1.0))

                                (set 'vt-throttle-input throttle-normalized)
                                (set 'vt-brake-input brake-normalized)
                                (set 'vt-output-disabled output-disabled)
                                (if (= output-disabled 0)
                                    {
                                        (handle-inputs throttle-normalized brake-normalized aux)
                                        (manage-speed-mode speed-mode)
                                    }
                                    {
                                        (if (> brakelight-state 0)
                                            {
                                                (pwm-set-duty 0.0)
                                                (set 'brakelight-state 0)
                                            }
                                        )
                                    }
                                )
                            }
                        )
                    }
                    {
                        (set 'vt-failed-tx-ct (+ vt-failed-tx-ct 1))
                        (if (> (- (systime) last-tx-time) (if (= vt-output-disabled 1) 5000 500))
                            {
                                ;fail safe on connection loss
                                ;thresholds of 500 ms when locked/off and 50 ms when active will count failure after ~2 and ~5 successive failures, respectively
                                (set-current-rel 0.0)
                                (set 'vt-throttle-input 0.0)
                                (set 'vt-brake-input 0.0)
                                (app-disable-output 0)
                                (puts "I2C Comm Timeout")
                            }
                        )
                    }
                )
            }
        )
        (i2c-comm)
    }
)

(defun handle-inputs(throttle brake aux) ; Frame 0x65
    {
        (app-disable-output 300)
        (var brakes-on (> brake 0))

        ; Pass through throttle and brake as relative current to VESC
        (if (> throttle 0)
            {
                (if brakes-on
                    (set-current-rel 0) ;throttle and brake at the same time - cut power
                    (set-current-rel throttle) ;throttle and no brake - send forward current
                )
            }
            {
                (if brakes-on
                    (set-brake-rel brake) ;brake and no throttle - send brake current
                    (set-current-rel 0) ;no brake, no throttle - no current output
                )
            }
        )
        (if brakes-on
            {
                (if (!= brakelight-state 2)
                    {
                        (pwm-set-duty 1.0)
                        (set 'brakelight-state 2)
                    }
                )
            }
            {
                (if (!= brakelight-state 1)
                    {
                        (pwm-set-duty 0.33)
                        (set 'brakelight-state 1)
                    }
                )
            }
        )
    }
)

(defun manage-speed-mode(speed-mode)
    {
        (if (= speed-mode vt-current-speed-mode)
            {}
            {
                (set 'vt-current-speed-mode speed-mode)
            }
        )
    }
)

(defun calculate-checksum(array)
    {
        (var checksum 0)
        (looprange i 0 5 (set 'checksum (+ (bufget-u8 array i) checksum)))
        (set 'checksum (bitwise-xor checksum 0xffff))
        (bufset-u8 array 6 checksum)
        (bufset-u8 array 7 (shr checksum 8))
    }
)

(defun validate-checksum(checksum array)
    {
        (var sum 0)
        (looprange i 0 5 (set 'sum (+ (bufget-u8 array i) sum)))

        (if (= (+ sum checksum) 0xffff)
            1
            0
        )
    }
)

(defun update-stats ()
    {
        (set 'current-speed-encoded (abs (round (* (get-speed) 10))))
        (if (= has-vesc-bms 1)
            (set 'battery-soc (get-bms-val 'bms-soc))
            (set 'battery-soc (get-batt))
        )
        (set 'battery-soc (round (* battery-soc 100)))
        (if (> battery-soc 100) (set 'battery-soc 100))
        (set 'temp-motor-encoded (round (+ (get-temp-mot) 50)))
        (set 'temp-fet-encoded (round (+ (get-temp-fet) 50)))
        (if (and (= vt-output-disabled 1) (and (= vt-brake-input 0) (> (get-bms-val 'bms-i-in) 0.1)))
            (set 'aux-bitmap (bitwise-or aux-bitmap 0x01)) ;or 00000001
            (set 'aux-bitmap (bitwise-and aux-bitmap 0xFE)) ;and 11111110
        )
        (yield 100000)
        (update-stats)
    }
)

(defun manage-thermals()
    {
        (var temp-mot-raw (get-temp-mot))
        (var speed-raw (get-speed))
        (if (> temp-mot-raw lower-current-limit-motor-temp)
            {
                (let ((current-limit-factor (/ (- temp-mot-raw lower-current-limit-motor-temp) current-limit-denominator)))
                    {
                        (if (> current-limit-factor 1.0) (set 'current-limit-factor 1.0))
                        (set 'vt-current-limit (- max-amps (* current-offset current-limit-factor)))
                        (conf-set 'l-current-max vt-current-limit)
                    }
                )
            }
            {
                (if (< speed-raw 2.0) ;Amperage boost below 2 m/s (and when not throttling) to assist launch
                    {
                        (let ((current-boost-factor (+ 1.0 (* (/ (- 2.0 speed-raw) 2.0) takeoff-assist-factor))))
                            {
                                (set 'vt-current-limit (* max-amps current-boost-factor))
                                (conf-set 'l-current-max vt-current-limit)
                            }
                        )
                    }
                    {
                        (if (> (conf-get 'l-current-max) max-amps)
                            (set 'vt-current-limit max-amps)
                            (conf-set 'l-current-max vt-current-limit)
                        )
                    }
                )
            }
        )
        (yield 200000)
        (manage-thermals)
    }
)

(defun telemetry()
    {
        (yield 500000)
        (var time-since-start (/ (- (systime) start-time) 10000.0))
        (set 'vt-tx-per-sec (/ number-of-tx time-since-start))
        (set 'start-time (systime))
        (set 'number-of-tx 0)
        (telemetry)
    }
)

;Start "threads" for periodic tasks
(if (= capture-telemetry 1) (spawn 32 telemetry))
(spawn 128 update-stats)
(spawn 128 manage-thermals)

;Start up comms with ESP32 via I2C
(yield 200000)
(i2c-start 'rate-200k)
(i2c-comm)
