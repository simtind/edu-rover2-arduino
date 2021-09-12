#include "config.h"
#include <ArduinoJson.h>

StaticJsonDocument<256> sensors;

void set_motor_thrust(int dir_pin, int pwm_pin, bool dir_val, int thrust)
{
  if (thrust == 0)
  {
    digitalWrite(dir_pin, 0);
    digitalWrite(pwm_pin, 0);
  }
  else
  {
    digitalWrite(dir_pin, dir_val);
    analogWrite(pwm_pin, MOTOR_PWM_VAL(thrust, dir_val));
  }
}

void setup() {
  // put your setup code here, to run once:

  pinMode(PIN_LED_PWM, OUTPUT);
  pinMode(PIN_MOTOR_ENABLE, OUTPUT);
  pinMode(PIN_MOTOR1_PWM, OUTPUT);
  pinMode(PIN_MOTOR1_DIR, OUTPUT);
  pinMode(PIN_MOTOR2_PWM, OUTPUT);
  pinMode(PIN_MOTOR2_DIR, OUTPUT);
  pinMode(PIN_MOTOR3_PWM, OUTPUT);
  pinMode(PIN_MOTOR3_DIR, OUTPUT);
  pinMode(PIN_MOTOR4_PWM, OUTPUT);
  pinMode(PIN_MOTOR4_DIR, OUTPUT);

  digitalWrite(PIN_LED_PWM, 0);
  digitalWrite(PIN_LED_PWM, 0);
  digitalWrite(PIN_MOTOR_ENABLE, 0);
  digitalWrite(PIN_MOTOR1_PWM, 0);
  digitalWrite(PIN_MOTOR1_DIR, 0);
  digitalWrite(PIN_MOTOR2_PWM, 0);
  digitalWrite(PIN_MOTOR2_DIR, 0);
  digitalWrite(PIN_MOTOR3_PWM, 0);
  digitalWrite(PIN_MOTOR3_DIR, 0);
  digitalWrite(PIN_MOTOR4_PWM, 0);
  digitalWrite(PIN_MOTOR4_DIR, 0);

  sensors["motor1_current"]    = 0.0;
  sensors["motor2_current"]    = 0.0;
  sensors["motor3_current"]    = 0.0;
  sensors["motor4_current"]    = 0.0;
  sensors["battery"]           = 0.0;
  sensors["water_pressure"]    = 0.0;
  sensors["water_temperature"] = 0.0;

  RPI_SERIAL.begin(115200);
  RPI_SERIAL.setTimeout(0);
  RPI_SERIAL.println("EduROV Arduino start");
  
  #if DEBUG
    DEBUG_SERIAL.begin(115200);
    DEBUG_SERIAL.println("EduROV Arduino debug start");
  #endif
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t messageTime = 0;
  static uint32_t sensor_interval = 500; //how long between each sensor update - milliseconds
  static String indata = "";

  int16_t starboard_thrust = 0;
  int16_t port_thrust = 0;
  int16_t vertical_thrust = 0;
  int16_t led_power = 0;
  uint32_t new_sensor_interval = 0;
  bool update_actuators = false;
  auto time_diff = millis() - messageTime;

  if (time_diff >= sensor_interval) {
    messageTime = millis();

    // All ADC measurements are multiplied by 100 to report 2-decimal precision numbers where possible

    sensors["motor1_current"]    = MOTOR_ADC_TO_A(analogRead(PIN_SENSE_MOTOR1));
    sensors["motor2_current"]    = MOTOR_ADC_TO_A(analogRead(PIN_SENSE_MOTOR2));
    sensors["motor3_current"]    = MOTOR_ADC_TO_A(analogRead(PIN_SENSE_MOTOR3));
    sensors["motor4_current"]    = MOTOR_ADC_TO_A(analogRead(PIN_SENSE_MOTOR4));
    sensors["battery"]           = BATTERY_ADC_TO_VOLT    (analogRead(PIN_SENSE_BATTERY));
    sensors["water_pressure"]    = PRESSURE_ADC_TO_KPA    (analogRead(PIN_SENSE_PRESSURE));
    sensors["water_temperature"] = TEMPERATURE_ADC_TO_DEGC(analogRead(PIN_SENSE_TEMPERATURE));
    #if DEBUG
      sensors["time_diff"]         = time_diff;
    #endif

    serializeJson(sensors, RPI_SERIAL);
    RPI_SERIAL.println("");
    
    #if DEBUG
      serializeJson(sensors, DEBUG_SERIAL);
      DEBUG_SERIAL.println("");
    #endif
  }
  
  indata += RPI_SERIAL.readString();
  auto end_at = indata.lastIndexOf('\n');

  if (end_at != -1 && end_at > 1) {
    #if DEBUG
      DEBUG_SERIAL.println("\"" + indata + "\"");
    #endif

    int starboard_thrust = 0;
    int port_thrust = 0;
    int vertical_thrust = 0;
    int led_power = 0;
    int interval = 0;
    int armed = 0;

    auto num_parsed = sscanf(indata.c_str(), "%i,%i,%i,%i,%i,%i", &starboard_thrust, &port_thrust, &vertical_thrust, &led_power, &interval, &armed);
    indata = indata.substring(end_at + 1);
    if (num_parsed != 6)
    {
      #if DEBUG
        DEBUG_SERIAL.println("Failed to deserialize data");
      #endif
      return;
    }

    digitalWrite(PIN_MOTOR_ENABLE, armed);
    set_motor_thrust(
      PIN_MOTOR1_DIR,
      PIN_MOTOR1_PWM,
      MOTOR1_DIR_VAL(starboard_thrust),
      starboard_thrust
    );

    set_motor_thrust(
      PIN_MOTOR2_DIR,
      PIN_MOTOR2_PWM,
      MOTOR2_DIR_VAL(port_thrust),
      port_thrust
    );
    
    set_motor_thrust(
      PIN_MOTOR3_DIR,
      PIN_MOTOR3_PWM,
      MOTOR3_DIR_VAL(vertical_thrust),
      vertical_thrust
    );
    set_motor_thrust(
      PIN_MOTOR4_DIR,
      PIN_MOTOR4_PWM,
      MOTOR4_DIR_VAL(vertical_thrust),
      vertical_thrust
    );
  
    analogWrite(PIN_LED_PWM, led_power);

    if (interval != sensor_interval)
    {
      #if DEBUG
        DEBUG_SERIAL.println(String("Interval = ") + max(30, interval));
      #endif
    }
    sensor_interval = max(30, interval);
  }
}
