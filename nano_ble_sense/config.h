#ifndef _EDUROV_CONFIG_H_
#define _EDUROV_CONFIG_H_

// Set to true if compiling for Arduino nano 33 ble sense.
#define HAS_ONBOARD_SENSORS false

#define RPI_SERIAL Serial1
#define DEBUG_SERIAL Serial
#define DEBUG false

#define frac_map(val, fromLow, fromHigh, toLow, toHigh) ((val - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow)

// ----- Other I/O -----
#define PIN_SENSE_BATTERY A2
#define PIN_SENSE_PRESSURE A1
#define PIN_SENSE_TEMPERATURE A0

#define SENSOR_SCALE 100

#define BATTERY_ADC_TO_VOLT(adc_value)     frac_map(adc_value, 0, 1023, 0, 13.2f)  // 0V to 3V3 in, multiply by 4 for divider.
#define PRESSURE_ADC_TO_KPA(adc_value)     frac_map(adc_value, 0, 1023, 0, 175.0f) // 0V to 3v3 -> 10kPa to 175kPa, P = (Vo / 0.02) + 10
#define TEMPERATURE_ADC_TO_DEGC(adc_value) frac_map(adc_value, 0, 1023, 0, 330.0f)  // 3V3 @ (10mV / degC) = 330 degC  

#define PIN_LED_PWM 3

// ---- Motor I/O -----

#define PIN_SENSE_MOTOR1 A7
#define PIN_SENSE_MOTOR2 A6
#define PIN_SENSE_MOTOR3 A5
#define PIN_SENSE_MOTOR4 A4
#define MOTOR_ADC_TO_A(adc_value)         frac_map(adc_value, 0, 1023, -3.3, 3.3)

// Select motor I/O pins so that we can use hw PWM outputs.
#define MOTOR_DIR_VAL(thrust) (thrust < 0)
#define MOTOR_PWM_VAL(thrust, dir_val)     (dir_val ? map(abs(thrust), 0, 255, 255, 0) : map(abs(thrust), 0, 255, 0, 255))

#define PIN_MOTOR_ENABLE 2

#define PIN_MOTOR1_PWM 4 // IO1A
#define PIN_MOTOR1_DIR 5 // IO1B
#define MOTOR1_DIR_VAL(thrust) MOTOR_DIR_VAL(thrust)

#define PIN_MOTOR2_PWM 6  // IO2A
#define PIN_MOTOR2_DIR 7  // IO2B
#define MOTOR2_DIR_VAL(thrust) MOTOR_DIR_VAL(thrust)

#define PIN_MOTOR3_PWM 8  // IO3A
#define PIN_MOTOR3_DIR 9  // IO3B
#define MOTOR3_DIR_VAL(thrust) MOTOR_DIR_VAL(thrust)

#define PIN_MOTOR4_PWM 10 // IO4A
#define PIN_MOTOR4_DIR 11 // IO4B
#define MOTOR4_DIR_VAL(thrust) MOTOR_DIR_VAL(thrust)

#endif /* _EDUROV_CONFIG_H_ */
