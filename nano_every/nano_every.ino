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

  Serial1.begin(115200);
  Serial.begin(115200);

  Serial1.println("EduROV Arduino start");
  Serial.println("EduROV Arduino debug start");
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t messageTime = 0;
  static uint32_t sensor_interval = 500; //how long between each sensor update - milliseconds
  static String indata = "";

  Serial1.setTimeout(sensor_interval - 10);

  int16_t starboard_thrust = 0;
  int16_t port_thrust = 0;
  int16_t vertical_thrust = 0;
  int16_t led_power = 0;
  uint32_t new_sensor_interval = 0;
  bool update_actuators = false;

  if ((millis() - messageTime) > sensor_interval) {
    messageTime = millis();

    // All ADC measurements are multiplied by 100 to report 2-decimal precision numbers where possible

    sensors["motor1_current"]    = MOTOR_ADC_TO_A(analogRead(PIN_SENSE_MOTOR1));
    sensors["motor2_current"]    = MOTOR_ADC_TO_A(analogRead(PIN_SENSE_MOTOR2));
    sensors["motor3_current"]    = MOTOR_ADC_TO_A(analogRead(PIN_SENSE_MOTOR3));
    sensors["motor4_current"]    = MOTOR_ADC_TO_A(analogRead(PIN_SENSE_MOTOR4));
    sensors["battery"]           = BATTERY_ADC_TO_VOLT    (analogRead(PIN_SENSE_BATTERY));
    sensors["water_pressure"]    = PRESSURE_ADC_TO_KPA    (analogRead(PIN_SENSE_PRESSURE));
    sensors["water_temperature"] = TEMPERATURE_ADC_TO_DEGC(analogRead(PIN_SENSE_TEMPERATURE));

    serializeMsgPack(sensors, Serial1);
    Serial1.println("");
  }

  if (Serial1.available()) {
    indata += Serial1.read();
  }

  if (indata.endsWith("\n")) {
    if (indata.length() > 1)
    {
      Serial.println(indata);
    }

    StaticJsonDocument<256> doc;
    auto error = deserializeMsgPack(doc, indata);
    if (error != DeserializationError::Ok)
    {
      Serial.println("Failed to deserialize input");
      return;
    }

    JsonVariant starboard_thrust = doc["starboard"];
    JsonVariant port_thrust = doc["port"];
    JsonVariant vertical_thrust = doc["vertical"];
    JsonVariant led_power = doc["headlight"];
    JsonVariant interval = doc["interval"];

    if (!starboard_thrust.isNull() &&
        !port_thrust.isNull() &&
        !vertical_thrust.isNull())
    {

      digitalWrite(PIN_MOTOR_ENABLE, 1);
    }

    if (!starboard_thrust.isNull())
    {
      float val = starboard_thrust.as<float>();
      set_motor_thrust(
        PIN_MOTOR1_DIR,
        PIN_MOTOR1_PWM,
        MOTOR1_DIR_VAL(val),
        val
      );
    }
    if (!port_thrust.isNull())
    {
      float val = port_thrust.as<float>();
      set_motor_thrust(
        PIN_MOTOR2_DIR,
        PIN_MOTOR2_PWM,
        MOTOR2_DIR_VAL(val),
        val
      );
    }
    if (!vertical_thrust.isNull())
    {
      float val = vertical_thrust.as<float>();
      set_motor_thrust(
        PIN_MOTOR3_DIR,
        PIN_MOTOR3_PWM,
        MOTOR3_DIR_VAL(val),
        val
      );
      set_motor_thrust(
        PIN_MOTOR4_DIR,
        PIN_MOTOR4_PWM,
        MOTOR4_DIR_VAL(val),
        val
      );
    }
    if (!led_power.isNull())
    {
      analogWrite(PIN_LED_PWM, led_power.as<float>());
    }
    if (!interval.isNull())
    {
      sensor_interval = interval.as<uint32_t>();
      Serial.println("Interval = " + sensor_interval);
    }
  }
}
