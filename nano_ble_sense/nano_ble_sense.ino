#include "config.h"
#include <mbed.h>
#include <rtos.h>
#include <ArduinoJson.h>
#include <Arduino_LSM9DS1.h>
#include <Arduino_LPS22HB.h>
#include <Arduino_HTS221.h>

rtos::Mutex sensors_mutex;

std::chrono::milliseconds sensor_interval { 500 };
StaticJsonDocument<256> sensors;

rtos::Thread thread_read_inertia_sensors;
rtos::Thread thread_read_env_sensors;
rtos::Thread thread_read_ext_sensors;
rtos::Thread thread_read_serial_input;


void set_motor_thrust(int dir_pin, int pwm_pin, bool dir_val, float thrust)
{
  if (abs(thrust) < 1)
  {
    digitalWrite(dir_pin, 0);
    analogWrite(pwm_pin, 0);
  }
  else
  {
    digitalWrite(dir_pin, dir_val);
    analogWrite(pwm_pin, MOTOR_PWM_VAL(thrust, dir_val));
  }
}

void read_inertia_sensors()
{
  while (true) {
    Serial.println("read_inertia_sensors");
    rtos::Kernel::Clock::time_point next_time = rtos::Kernel::Clock::now() + sensor_interval;

    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float mag_x;
    float mag_y;
    float mag_z;

    Serial.println("Check sensors and update");

    if (IMU.accelerationAvailable()) {
      Serial.println("Reading acceleration");
      IMU.readAcceleration(acc_x, acc_y, acc_z);
      sensors_mutex.lock();

      sensors["acc_x"]  = acc_x;
      sensors["acc_y"]  = acc_y;
      sensors["acc_z"]  = acc_z;

      sensors_mutex.unlock();
    }

    if (IMU.gyroscopeAvailable()) {
      Serial.println("Reading gyro");
      IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
      sensors_mutex.lock();

      sensors["gyro_x"] = gyro_x;
      sensors["gyro_y"] = gyro_y;
      sensors["gyro_z"] = gyro_z;

      sensors_mutex.unlock();
    }

    if (IMU.magneticFieldAvailable()) {
      Serial.println("Reading compass");
      IMU.readMagneticField(mag_x, mag_y, mag_z);
      sensors_mutex.lock();

      sensors["mag_x"]  = mag_x;
      sensors["mag_y"]  = mag_y;
      sensors["mag_z"]  = mag_z;

      sensors_mutex.unlock();
    }

    Serial.println("sleeping");

    rtos::ThisThread::sleep_until(next_time);
  }
}

void read_env_sensors()
{
  while (true) {
    rtos::Kernel::Clock::time_point next_time = rtos::Kernel::Clock::now() + sensor_interval;

    float pressure;
    float temperature;
    float humidity;

    pressure =    BARO.readPressure();
    temperature = HTS.readTemperature();
    humidity =    HTS.readHumidity();

    sensors_mutex.lock();

    sensors["pressure"]     = pressure;
    sensors["temperature"]  = temperature;
    sensors["humidity"]     = humidity;

    sensors_mutex.unlock();

    rtos::ThisThread::sleep_until(next_time);
  }
}

void read_ext_sensors()
{
  while (true) {
    Serial.println("read_ext_sensors");
    rtos::Kernel::Clock::time_point next_time = rtos::Kernel::Clock::now() + sensor_interval;

    float motor1_current;
    float motor2_current;
    float motor3_current;
    float motor4_current;

    float battery;
    float pressure;
    float temperature;

    motor1_current = MOTOR_ADC_TO_A(analogRead(PIN_SENSE_MOTOR1));
    motor2_current = MOTOR_ADC_TO_A(analogRead(PIN_SENSE_MOTOR2));
    motor3_current = MOTOR_ADC_TO_A(analogRead(PIN_SENSE_MOTOR3));
    motor4_current = MOTOR_ADC_TO_A(analogRead(PIN_SENSE_MOTOR4));

    battery =     BATTERY_ADC_TO_VOLT    (analogRead(PIN_SENSE_BATTERY));
    pressure =    PRESSURE_ADC_TO_KPA    (analogRead(PIN_SENSE_PRESSURE));
    temperature = TEMPERATURE_ADC_TO_DEGC(analogRead(PIN_SENSE_TEMPERATURE));


    sensors_mutex.lock();

    sensors["motor1_current"]    = motor1_current;
    sensors["motor2_current"]    = motor2_current;
    sensors["motor3_current"]    = motor3_current;
    sensors["motor4_current"]    = motor4_current;
    sensors["battery"]           = battery;
    sensors["water_pressure"]    = pressure;
    sensors["water_temperature"] = temperature;

    sensors_mutex.unlock();

    rtos::ThisThread::sleep_until(next_time);
  }
}

void read_serial_input()
{
  while (true) {
    Serial.println("read_serial_input");
    String indata = Serial1.readStringUntil('}');

    indata += "}";
  
    if (indata.length() <= 2)
    {
      continue;
    }
    
    Serial.println("Got data input");
    Serial.println(indata);

    StaticJsonDocument<256> doc;
    auto error = deserializeJson(doc, indata);
    if (error != DeserializationError::Ok)
    {
      Serial.println("Failed to deserialize input");
      continue;
    }

    JsonVariant armed = doc["armed"];
    JsonVariant starboard_thrust = doc["starboard"];
    JsonVariant port_thrust = doc["port"];
    JsonVariant vertical_thrust = doc["vertical"];
    JsonVariant led_power = doc["headlight"];
    JsonVariant interval = doc["interval"];

    if (!armed.isNull())
    {
      digitalWrite(PIN_MOTOR_ENABLE, armed.as<bool>());
    }

    if (!starboard_thrust.isNull())
    {
      Serial.println("Got starboard thrust");
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
      Serial.println("Got port thrust");
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
      Serial.println("Got vertical thrust");
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
      sensor_interval = std::chrono::milliseconds(interval.as<int>());
      Serial.println("Interval = " + sensor_interval.count());
    }
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

  Serial1.begin(115200);
  Serial.begin(115200);

  Serial1.println("EduROV Arduino start");
  Serial.println("EduROV Arduino debug start");

  if (IMU.begin() && HAS_ONBOARD_SENSORS) {
    Serial.print("Accelerometer sample rate = ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println(" Hz");
    Serial.print("Gyroscope sample rate = ");
    Serial.print(IMU.gyroscopeSampleRate());
    Serial.println(" Hz");
    Serial.print("Magnetic field sample rate = ");
    Serial.print(IMU.magneticFieldSampleRate());
    Serial.println(" Hz");
    sensors["acc_x"]  = 0.0;
    sensors["acc_y"]  = 0.0;
    sensors["acc_z"]  = 0.0;
    sensors["gyro_x"] = 0.0;
    sensors["gyro_y"] = 0.0;
    sensors["gyro_z"] = 0.0;
    sensors["mag_x"]  = 0.0;
    sensors["mag_y"]  = 0.0;
    sensors["mag_z"]  = 0.0;
    thread_read_inertia_sensors.start(read_inertia_sensors);
  }
  else {
    Serial.println("Failed to initialize IMU!");
  }

  if (BARO.begin() && HTS.begin() && HAS_ONBOARD_SENSORS) {
    sensors["pressure"]     = 0.0;
    sensors["temperature"]  = 0.0;
    sensors["humidity"]     = 0.0;
    thread_read_env_sensors.start(read_env_sensors);
  }
  else {
    Serial.println("Failed to initialize pressure, humidity or temperature sensor!");
  }

  sensors["motor1_current"]    = 0.0;
  sensors["motor2_current"]    = 0.0;
  sensors["motor3_current"]    = 0.0;
  sensors["motor4_current"]    = 0.0;
  sensors["battery"]           = 0.0;
  sensors["water_pressure"]    = 0.0;
  sensors["water_temperature"] = 0.0;

  thread_read_ext_sensors.start(read_ext_sensors);
  thread_read_serial_input.start(read_serial_input);
}

void loop() {
  rtos::Kernel::Clock::time_point next_time = rtos::Kernel::Clock::now() + sensor_interval;

  Serial.println("Loop");

  sensors_mutex.lock();
  serializeJson(sensors, Serial1);
  
  sensors_mutex.unlock();

  rtos::ThisThread::sleep_until(next_time);
}
