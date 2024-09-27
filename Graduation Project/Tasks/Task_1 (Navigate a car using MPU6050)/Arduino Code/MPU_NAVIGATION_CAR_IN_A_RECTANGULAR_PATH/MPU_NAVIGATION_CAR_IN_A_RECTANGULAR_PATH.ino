#define motorR1     3
#define motorR2     2
#define motorL1     5
#define motorL2     4
#define motorEN1    6
#define motorEN2    9
#define sensor_pin  8

#include <Wire.h>
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX = 0, AccErrorY = 0, GyroErrorX = 0, GyroErrorY = 0, GyroErrorZ = 0;
float elapsedTime, currentTime, previousTime, value;
int counter = 0;
int steps = 0;
float distance , distanceX, distanceY, reqDistanceX = 1, reqDistanceY = 4.2 , totalDistance = 5.2 , returnDistance = 8.2;
void drive_straight(float yaw);
void first_turn_left(float yaw);
void second_turn_left(float yaw);
void third_turn_left(float yaw);
void stop_motors(void);
void setup() 
{
    pinMode(motorR1, OUTPUT);
    pinMode(motorR2, OUTPUT);
    pinMode(motorL1, OUTPUT);
    pinMode(motorL2, OUTPUT);
    pinMode(motorEN1, OUTPUT);
    pinMode(motorEN2, OUTPUT);

    // Initial motor direction
    digitalWrite(motorR1, LOW); 
    digitalWrite(motorR2, HIGH); 
    digitalWrite(motorL1, LOW); 
    digitalWrite(motorL2, HIGH); 
    Serial.begin(115200);
    pinMode(sensor_pin, INPUT_PULLUP);

  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);  
  calculate_IMU_error();
  delay(3000);

  
}

void loop()
{
    if(digitalRead(sensor_pin))
    {
        steps = steps + 1; 
        while(digitalRead(sensor_pin));
        distanceX = steps / 90.0;
        distanceY = steps / 90.0;
        distance = steps / 90.0;
        Serial.print(steps);
        Serial.print(" - ");
        Serial.println(distanceY);
    }
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - abs(AccErrorX); // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - abs(AccErrorY); // AccErrorY ~(-1.58)
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX - abs(GyroErrorX); // GyroErrorX ~(-0.56)
  GyroY = GyroY - abs(GyroErrorY); // GyroErrorY ~(2)
  GyroZ = GyroZ - abs(GyroErrorZ); // GyroErrorZ ~ (-0.8)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  
  // Print the values on the serial monitor

  Serial.print(roll);
  Serial.print("/");
  Serial.print(pitch);
  Serial.print("/");
  Serial.print(yaw);
  Serial.print("/");
  Serial.print(distanceX);
  Serial.print("/");
  Serial.println(distanceY);


first_turn_left(yaw);




}

void drive_straight(float yaw){
    if(yaw > 0){
    analogWrite(motorEN1, 255); // Increase speed
    analogWrite(motorEN2, 80);
  }
  else if(yaw < 0){
    analogWrite(motorEN1, 100); // Increase speed
    analogWrite(motorEN2, 255);
  }
}

void first_turn_left(float yaw){
  if(distanceX < reqDistanceX){
      drive_straight(yaw);
  }else if(distanceX >= reqDistanceX && yaw < 85){
    analogWrite(motorEN1, 0);
    analogWrite(motorEN2, 180);
  }else{
    second_turn_left(yaw);
  }
}

void second_turn_left(float yaw){
  if(distanceY < reqDistanceY){
    drive_straight(yaw - 85);
  }else if(distanceY >= reqDistanceY && yaw < 175){
    analogWrite(motorEN1, 0);
    analogWrite(motorEN2, 180);
  }else{
    third_turn_left(yaw);
  }
}
void third_turn_left(float yaw){
  if(distance < totalDistance){
    drive_straight(yaw - 175);
  }else if(distance >= totalDistance && yaw < 265){
    analogWrite(motorEN1, 0);
    analogWrite(motorEN2, 180);
  }else{
    if(distance < returnDistance){
       drive_straight(yaw - 265);
    }else if(distance >= returnDistance && yaw < 350){
      analogWrite(motorEN1, 0);
      analogWrite(motorEN2, 100);
    }else{
      stop_motors();
    }
  }
}
void stop_motors(){
  digitalWrite(motorR1, HIGH); 
  digitalWrite(motorR2, HIGH); 
  digitalWrite(motorL1, HIGH); 
  digitalWrite(motorL2, HIGH); 
  analogWrite(motorEN1, 0);
  analogWrite(motorEN2, 0);
}
void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (counter < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    counter++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  counter = 0;
  // Read gyro values 200 times
  while (counter < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    counter++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}