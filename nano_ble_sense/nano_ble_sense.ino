#include "config.h"
#include <mbed.h>
#include <rtos.h>
#include <ArduinoJson.h>
#if HAS_ONBOARD_SENSORS
  #include <Arduino_LSM9DS1.h>
  #include <Arduino_LPS22HB.h>
  #include <Arduino_HTS221.h>
#endif 

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

#if HAS_ONBOARD_SENSORS
  void read_inertia_sensors()
  {
    while (true) {
      #if DEBUG
        DEBUG_SERIAL.println("read_inertia_sensors");
      #endif
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
  
      #if DEBUG
        DEBUG_SERIAL.println("Check sensors and update");
      #endif
  
      if (IMU.accelerationAvailable()) {
        #if DEBUG
          DEBUG_SERIAL.println("Reading acceleration");
        #endif
        IMU.readAcceleration(acc_x, acc_y, acc_z);
        sensors_mutex.lock();
  
        sensors["acc_x"]  = acc_x;
        sensors["acc_y"]  = acc_y;
        sensors["acc_z"]  = acc_z;
  
        sensors_mutex.unlock();
      }
  
      if (IMU.gyroscopeAvailable()) {
        #if DEBUG
          DEBUG_SERIAL.println("Reading gyro");
        #endif
        IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
        sensors_mutex.lock();
  
        sensors["gyro_x"] = gyro_x;
        sensors["gyro_y"] = gyro_y;
        sensors["gyro_z"] = gyro_z;
  
        sensors_mutex.unlock();
      }
  
      if (IMU.magneticFieldAvailable()) {
        #if DEBUG
          DEBUG_SERIAL.println("Reading compass");
        #endif
        IMU.readMagneticField(mag_x, mag_y, mag_z);
        sensors_mutex.lock();
  
        sensors["mag_x"]  = mag_x;
        sensors["mag_y"]  = mag_y;
        sensors["mag_z"]  = mag_z;
  
        sensors_mutex.unlock();
      }
  
      #if DEBUG
        DEBUG_SERIAL.println("sleeping");
      #endif
  
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
#endif 

void read_ext_sensors()
{
  while (true) {
    #if DEBUG
      DEBUG_SERIAL.println("read_ext_sensors");
    #endif
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
    #if DEBUG
      DEBUG_SERIAL.println("read_serial_input");
    #endif
    String indata = RPI_SERIAL.readStringUntil('\n');

    if (indata.length() <= 2)
    {
      continue;
    }
    
    DEBUG_SERIAL.println("Got data input");
    DEBUG_SERIAL.println(indata);

    int starboard_thrust = 0;
    int port_thrust = 0;
    int vertical_thrust = 0;
    int led_power = 0;
    int interval = 0;
    int armed = 0;

    auto num_parsed = sscanf(indata.c_str(), "%i,%i,%i,%i,%i,%i", &starboard_thrust, &port_thrust, &vertical_thrust, &led_power, &interval, &armed);
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

    if (std::chrono::milliseconds(interval) != sensor_interval)
    {
      #if DEBUG
        DEBUG_SERIAL.println(String("Interval = ") + max(30, interval));
      #endif
    }
    sensor_interval = std::chrono::milliseconds(max(30, interval));
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

  RPI_SERIAL.begin(115200);
  RPI_SERIAL.println("EduROV Arduino start");
  
  #if DEBUG
    DEBUG_SERIAL.begin(115200);
    DEBUG_SERIAL.println("EduROV Arduino debug start");
  #endif

  #if HAS_ONBOARD_SENSORS
    if (IMU.begin() && HAS_ONBOARD_SENSORS) {
      #if DEBUG
        DEBUG_SERIAL.print("Accelerometer sample rate = ");
        DEBUG_SERIAL.print(IMU.accelerationSampleRate());
        DEBUG_SERIAL.println(" Hz");
        DEBUG_SERIAL.print("Gyroscope sample rate = ");
        DEBUG_SERIAL.print(IMU.gyroscopeSampleRate());
        DEBUG_SERIAL.println(" Hz");
        DEBUG_SERIAL.print("Magnetic field sample rate = ");
        DEBUG_SERIAL.print(IMU.magneticFieldSampleRate());
        DEBUG_SERIAL.println(" Hz");
      #endif
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
      #if DEBUG
        DEBUG_SERIAL.println("Failed to initialize IMU!");
      #endif
    }

    if (BARO.begin() && HTS.begin() && HAS_ONBOARD_SENSORS) {
      sensors["pressure"]     = 0.0;
      sensors["temperature"]  = 0.0;
      sensors["humidity"]     = 0.0;
      thread_read_env_sensors.start(read_env_sensors);
    }
    else {
      #if DEBUG
        DEBUG_SERIAL.println("Failed to initialize pressure, humidity or temperature sensor!");
      #endif
    }
  #endif

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

  #if DEBUG
    DEBUG_SERIAL.println("Loop");
  #endif

  sensors_mutex.lock();
  serializeJson(sensors, RPI_SERIAL);
  
  sensors_mutex.unlock();

  rtos::ThisThread::sleep_until(next_time);
}
