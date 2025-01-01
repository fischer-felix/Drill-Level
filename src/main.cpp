// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define BUTTON_PIN D7 
#define DEBOUNCE_TIME 250  // milliseconds

Adafruit_MPU6050 mpu;

volatile unsigned long lastButtonPress = 0;
volatile bool calibrating = false;

float accel_x_min = -9.51;
float accel_x_max = 10.26;
float accel_y_min = -9.74;
float accel_y_max = 9.81;
float accel_z_min = 11.17;
float accel_z_max = 9.07;


float gyro_error_x = 0;
float gyro_error_y = 0;
float gyro_error_z = 0;

unsigned long now, last_time;

float gyro_angle_x = 0;
float gyro_angle_y = 0;
float gyro_angle_z = 0;


void calibrate_accel() {

  calibrating = true;

  Serial.println("Calibrating accelerometer...");

  sensors_event_t a, g, temp;

  while (calibrating) {
    
    mpu.getEvent(&a, &g, &temp);

    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(", Z: ");
    Serial.println(a.acceleration.z);

    //delay(250);

  }
  
  Serial.println("Calibration complete!");

}

void IRAM_ATTR handleInterrupt() {
  unsigned long currentTime = millis();
  if ((currentTime - lastButtonPress) >= DEBOUNCE_TIME) {

    lastButtonPress = currentTime;

    calibrate_accel();
  
  }
}


void calibrate_gyro() {

  sensors_event_t a, g, temp;

  for (int i = 0; i < 200; i++) {
    mpu.getEvent(&a, &g, &temp);
    gyro_error_x += g.gyro.x;
    gyro_error_y += g.gyro.y;
    gyro_error_z += g.gyro.z;
    delay(3);
  }

  gyro_error_x /= 200;
  gyro_error_y /= 200;
  gyro_error_z /= 200;

}


void setup(void) {
  
  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(BUTTON_PIN, handleInterrupt, RISING);

  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);

    Serial.println("Calibrating, keep device still...");
  delay(2000);
  calibrate_gyro();
  Serial.print("gyro_error_x: ");
  Serial.println(gyro_error_x);
  Serial.print("gyro_error_y: ");
  Serial.println(gyro_error_y);
  Serial.print("gyro_error_z: ");
  Serial.println(gyro_error_z);

  Serial.println("");
  delay(1000);

}

void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Calculate the gyro angles */
  now = millis();
  float dt = (now - last_time) / 1000.0;
  last_time = now;

  gyro_angle_x += ((g.gyro.x - gyro_error_x) * dt) * (180.0 / M_PI);
  gyro_angle_y += ((g.gyro.y - gyro_error_y) * dt) * (180.0 / M_PI);
  gyro_angle_z += ((g.gyro.z - gyro_error_z) * dt) * (180.0 / M_PI);

  /* Calculate Accelerometer angles */
  float accel_angle_x = atan(a.acceleration.y / sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.z, 2))) * (180.0 / M_PI);

  
  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x - gyro_error_x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y - gyro_error_y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z - gyro_error_z);
  Serial.println(" rad/s");

  Serial.print("Gyro angle X: ");
  Serial.print(gyro_angle_x);
  Serial.print(", Y: ");
  Serial.print(gyro_angle_y);
  Serial.print(", Z: ");
  Serial.println(gyro_angle_z);

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(500);
}

