// Motor control pin definitions
#define motorR1     3   // Right motor forward
#define motorR2     2   // Right motor backward
#define motorL1     5   // Left motor forward
#define motorL2     4   // Left motor backward
#define motorEN1    6   // Right motor speed (PWM)
#define motorEN2    9   // Left motor speed (PWM)
#define sensor_pin  8   // Sensor pin (possibly an encoder or distance sensor)

// Include the Wire library for I2C communication
#include <Wire.h>

// MPU6050 sensor I2C address
const int MPU = 0x68; 

// Variables to store accelerometer and gyroscope data
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;

// Error variables for sensor calibration
float AccErrorX = 0, AccErrorY = 0, GyroErrorX = 0, GyroErrorY = 0, GyroErrorZ = 0;

// Variables for timing and distance tracking
float elapsedTime, currentTime, previousTime, value;
int counter = 0;
int steps = 0;
float distance, distance1, distance2;
float reqDistance1 = 1, reqDistance2 = 2.5, reqDistance3 = 3.7, reqDistance4 = 5; // Required distances for each phase of movement

// Function declarations for movement and turning logic
void drive_straight(float yaw);
void first_turn_left(float yaw);
void second_turn_left(float yaw);
void third_turn_left(float yaw);
void stop_motors(void);

void setup() 
{
    // Set motor control pins as outputs
    pinMode(motorR1, OUTPUT);
    pinMode(motorR2, OUTPUT);
    pinMode(motorL1, OUTPUT);
    pinMode(motorL2, OUTPUT);
    pinMode(motorEN1, OUTPUT);
    pinMode(motorEN2, OUTPUT);
    
    // Set sensor pin as input with pull-up resistor
    pinMode(sensor_pin, INPUT_PULLUP);

    // Initialize motors (set both motors to rotate backward by default)
    digitalWrite(motorR1, LOW); 
    digitalWrite(motorR2, HIGH); 
    digitalWrite(motorL1, LOW); 
    digitalWrite(motorL2, HIGH); 

    // Initialize serial communication for debugging
    Serial.begin(115200);
    
    // Initialize the MPU6050
    Wire.begin();                      
    Wire.beginTransmission(MPU);       // Start communication with MPU6050 (0x68 is the address)
    Wire.write(0x6B);                  // Access the power management register
    Wire.write(0x00);                  // Wake up the MPU6050 by clearing the sleep mode bit
    Wire.endTransmission(true);  

    // Calculate IMU errors for more accurate readings
    calculate_IMU_error();

    // Small delay to ensure sensor is stable
    delay(3000);
}

void loop()
{
    // If the sensor pin is triggered (e.g., by an encoder)
    if(digitalRead(sensor_pin))
    {
        // Increment the step count and calculate distances
        steps = steps + 1; 
        while(digitalRead(sensor_pin));  // Wait until the sensor pin is released
        distance1 = steps / 90.0;        // Convert steps to distance (adjust based on your system)
        distance2 = steps / 90.0;
        distance = steps / 90.0;
        Serial.print(steps);
        Serial.print(" - ");
        Serial.println(distance2);       // Print step count and distance for debugging
    }

    // Read accelerometer data
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);                    // Start with the ACCEL_XOUT_H register
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);       // Request 6 registers (2 per axis for X, Y, Z)
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;  // X-axis value
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;  // Y-axis value
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;  // Z-axis value

    // Calculate roll and pitch from accelerometer data
    accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - abs(AccErrorX); 
    accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - abs(AccErrorY); 

    // Read gyroscope data
    previousTime = currentTime;          
    currentTime = millis();              // Update the current time
    elapsedTime = (currentTime - previousTime) / 1000;  // Time in seconds since last update

    Wire.beginTransmission(MPU);
    Wire.write(0x43);                    // Start with the GYRO_XOUT_H register
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);      // Request 6 registers (2 per axis for X, Y, Z)
    GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;   // X-axis value
    GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;   // Y-axis value
    GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;   // Z-axis value

    // Apply error correction to gyroscope data
    GyroX = GyroX - abs(GyroErrorX); 
    GyroY = GyroY - abs(GyroErrorY); 
    GyroZ = GyroZ - abs(GyroErrorZ); 

    // Calculate angular changes (in degrees) based on gyroscope data and elapsed time
    gyroAngleX = gyroAngleX + GyroX * elapsedTime; 
    gyroAngleY = gyroAngleY + GyroY * elapsedTime;
    yaw =  yaw + GyroZ * elapsedTime;

    // Use a complementary filter to combine accelerometer and gyroscope readings for better accuracy
    roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
    pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

    // Print roll, pitch, yaw, and distances for debugging
    Serial.print(roll);
    Serial.print("/");
    Serial.print(pitch);
    Serial.print("/");
    Serial.print(yaw);
    Serial.print("/");
    Serial.print(distance1);
    Serial.print("/");
    Serial.println(distance2);

    // Call the first turn function to start navigation
    first_turn_left(yaw);
}

// Function to drive straight, adjusting motor speeds based on yaw correction
void drive_straight(float yaw){
    if(yaw > 0){
        // If yaw is positive (turning right), slow down the right motor and speed up the left motor
        analogWrite(motorEN1, 255); 
        analogWrite(motorEN2, 80);
    }
    else if(yaw < 0){
        // If yaw is negative (turning left), slow down the left motor and speed up the right motor
        analogWrite(motorEN1, 100); 
        analogWrite(motorEN2, 255);
    }
}

// Function for the first left turn based on distance and yaw
void first_turn_left(float yaw){
    if(distance1 < reqDistance1){
        drive_straight(yaw);  // Drive straight until reaching the first required distance
    }else if(distance1 >= reqDistance1 && yaw < 85){
        analogWrite(motorEN1, 0);        // Stop the right motor and turn left
        analogWrite(motorEN2, 180);      // Left motor continues
    }else{
        second_turn_left(yaw);           // Proceed to the next turn phase
    }
}

// Function for the second left turn
void second_turn_left(float yaw){
    if(distance2 < reqDistance2){
        drive_straight(yaw - 85);        // Continue driving straight after the first turn
    }else if(distance2 >= reqDistance2 && yaw < 175){
        analogWrite(motorEN1, 0);        // Stop the right motor and turn left again
        analogWrite(motorEN2, 180);      // Continue left motor
    }else{
        third_turn_left(yaw);            // Proceed to the third turn phase
    }
}

// Function for the third left turn
void third_turn_left(float yaw){
    if(distance < reqDistance3){
        drive_straight(yaw - 175);       // Drive straight again after the second turn
    }else if(distance >= reqDistance3 && yaw < 265){
        analogWrite(motorEN1, 0);        // Another left turn
        analogWrite(motorEN2, 180);      
    }else{
        if(distance < reqDistance4){
            drive_straight(yaw - 265);   // Continue driving straight after the third turn
        }else if(distance >= reqDistance4){
            stop_motors();               // Stop motors after reaching the final distance
        }
    }
}

// Function to stop the motors
void stop_motors(void){
  digitalWrite(motorR1, HIGH); 
  digitalWrite(motorR2, HIGH); 
  digitalWrite(motorL1, HIGH); 
  digitalWrite(motorL2, HIGH); 
  analogWrite(motorEN1, 0);
  analogWrite(motorEN2, 0);
}

// Function to calculate IMU sensor errors for calibration
void calculate_IMU_error() {
    // Initialize variables to accumulate errors
    int c = 0;
    while (c < 200) {
        // Read accelerometer data
        Wire.beginTransmission(MPU);
        Wire.write(0x3B);               // Starting with ACCEL_XOUT_H register
        Wire.endTransmission(false);
        Wire.requestFrom(MPU, 6, true);
        AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
        AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
        AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
        
        // Sum up the errors
        AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI));
        AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI));

        // Read gyroscope data
        Wire.beginTransmission(MPU);
        Wire.write(0x43);               // Starting with GYRO_XOUT_H register
        Wire.endTransmission(false);
        Wire.requestFrom(MPU, 6, true);
        GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
        GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
        GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

        // Sum up the gyroscope errors
        GyroErrorX = GyroErrorX + GyroX;
        GyroErrorY = GyroErrorY + GyroY;
        GyroErrorZ = GyroErrorZ + GyroZ;
        c++;
    }

    // Average the accumulated errors
    AccErrorX = AccErrorX / 200;
    AccErrorY = AccErrorY / 200;
    GyroErrorX = GyroErrorX / 200;
    GyroErrorY = GyroErrorY / 200;
    GyroErrorZ = GyroErrorZ / 200;
}
