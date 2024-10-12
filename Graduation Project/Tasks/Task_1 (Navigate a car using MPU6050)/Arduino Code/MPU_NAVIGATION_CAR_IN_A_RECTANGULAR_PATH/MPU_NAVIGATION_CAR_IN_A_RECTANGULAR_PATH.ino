#define motorR1     3   // Right motor control pin 1 (Direction)
#define motorR2     2   // Right motor control pin 2 (Direction)
#define motorL1     5   // Left motor control pin 1 (Direction)
#define motorL2     4   // Left motor control pin 2 (Direction)
#define motorEN1    6   // Right motor PWM pin (Speed control)
#define motorEN2    9   // Left motor PWM pin (Speed control)
#define sensor_pin  8   // Sensor pin to track steps or distance

#include <Wire.h>
const int MPU = 0x68; // MPU6050 I2C address
// Variables to store accelerometer and gyroscope data
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
// Variables for angles calculated from sensor data
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
// Variables to store IMU errors
float AccErrorX = 0, AccErrorY = 0, GyroErrorX = 0, GyroErrorY = 0, GyroErrorZ = 0;
float elapsedTime, currentTime, previousTime, value;
// Step and distance variables
int counter = 0;
int steps = 0;
float distance, distance1, distance2;
float reqDistance1 = 1, reqDistance2 = 4.2 , reqDistance3 = 5.2 , reqDistance4 = 8.2;  // Distances at which turns are made

// Function declarations
void drive_straight(float yaw);
void first_turn_left(float yaw);
void second_turn_left(float yaw);
void third_turn_left(float yaw);
void stop_motors(void);

void setup() {
    // Set motor pins as output
    pinMode(motorR1, OUTPUT);
    pinMode(motorR2, OUTPUT);
    pinMode(motorL1, OUTPUT);
    pinMode(motorL2, OUTPUT);
    pinMode(motorEN1, OUTPUT);
    pinMode(motorEN2, OUTPUT);
    pinMode(sensor_pin, INPUT_PULLUP);  // Sensor pin as input with pullup resistor

    // Set initial motor directions to move forward
    digitalWrite(motorR1, LOW); 
    digitalWrite(motorR2, HIGH); 
    digitalWrite(motorL1, LOW); 
    digitalWrite(motorL2, HIGH); 

    Serial.begin(115200);  // Begin serial communication for debugging

    // Initialize MPU6050 communication
    Wire.begin();
    Wire.beginTransmission(MPU);       
    Wire.write(0x6B);  // Write to power management register 6B
    Wire.write(0x00);  // Wake up the MPU6050 by writing 0 to the register
    Wire.endTransmission(true);

    // Calculate IMU errors
    calculate_IMU_error();
    delay(3000);  // Delay to stabilize the setup
}

void loop() {
    // Check if sensor is triggered (used for step/distance counting)
    if(digitalRead(sensor_pin)) {
        steps = steps + 1;  // Increment step counter
        while(digitalRead(sensor_pin));  // Wait until sensor signal is released
        // Calculate distances based on steps (assuming 90 steps per rotation)
        distance1 = steps / 90.0;
        distance2 = steps / 90.0;
        distance = steps / 90.0;
        Serial.print(steps);  // Debug: print step count
        Serial.print(" - ");
        Serial.println(distance2);  // Debug: print distance
    }

    // Read accelerometer data
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  // Start at register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);  // Read 6 bytes (accelerometer X, Y, Z)
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;  // X-axis value
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;  // Y-axis value
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;  // Z-axis value

    // Calculate roll and pitch from accelerometer data
    accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - abs(AccErrorX);
    accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - abs(AccErrorY);

    // Get elapsed time for gyroscope calculations
    previousTime = currentTime;
    currentTime = millis();
    elapsedTime = (currentTime - previousTime) / 1000;  // Convert milliseconds to seconds

    // Read gyroscope data
    Wire.beginTransmission(MPU);
    Wire.write(0x43);  // Start at register 0x43 (GYRO_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);  // Read 6 bytes (gyroscope X, Y, Z)
    GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;  // Convert to degrees/second
    GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

    // Correct gyroscope data with previously calculated errors
    GyroX = GyroX - abs(GyroErrorX);
    GyroY = GyroY - abs(GyroErrorY);
    GyroZ = GyroZ - abs(GyroErrorZ);

    // Integrate gyroscope data to get angles
    gyroAngleX += GyroX * elapsedTime;
    gyroAngleY += GyroY * elapsedTime;
    yaw += GyroZ * elapsedTime;

    // Complementary filter to combine accelerometer and gyroscope readings
    roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
    pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

    // Debug: Print roll, pitch, yaw, and distances
    Serial.print(roll);
    Serial.print("/");
    Serial.print(pitch);
    Serial.print("/");
    Serial.print(yaw);
    Serial.print("/");
    Serial.print(distance1);
    Serial.print("/");
    Serial.println(distance2);

    // Call function for first turn
    first_turn_left(yaw);
}

// Function to drive straight, adjusting speed based on yaw
void drive_straight(float yaw) {
    if(yaw > 0) {
        analogWrite(motorEN1, 255);  // Increase right motor speed
        analogWrite(motorEN2, 80);   // Decrease left motor speed
    } else if(yaw < 0) {
        analogWrite(motorEN1, 100);  // Decrease right motor speed
        analogWrite(motorEN2, 255);  // Increase left motor speed
    }
}

// Function for first left turn
void first_turn_left(float yaw) {
    if(distance1 < reqDistance1) {
        drive_straight(yaw);  // Keep driving straight until reaching the first distance threshold
    } else if(distance1 >= reqDistance1 && yaw < 85) {
        analogWrite(motorEN1, 0);  // Stop right motor
        analogWrite(motorEN2, 180);  // Turn left using the left motor
    } else {
        second_turn_left(yaw);  // Move to the second turn after completing the first
    }
}

// Function for second left turn
void second_turn_left(float yaw) {
    if(distance2 < reqDistance2) {
        drive_straight(yaw - 85);  // Adjust yaw after first turn
    } else if(distance2 >= reqDistance2 && yaw < 175) {
        analogWrite(motorEN1, 0);  // Stop right motor
        analogWrite(motorEN2, 180);  // Turn left using the left motor
    } else {
        third_turn_left(yaw);  // Move to the third turn
    }
}

// Function for third left turn
void third_turn_left(float yaw) {
    if(distance < reqDistance3) {
        drive_straight(yaw - 175);  // Adjust yaw after second turn
    } else if(distance >= reqDistance3 && yaw < 265) {
        analogWrite(motorEN1, 0);  // Stop right motor
        analogWrite(motorEN2, 180);  // Turn left using the left motor
    } else {
        if(distance < reqDistance4) {
            drive_straight(yaw - 265);  // Adjust yaw after third turn
        } else if(distance >= reqDistance4 && yaw < 350) {
            analogWrite(motorEN1, 0);  // Stop right motor
            analogWrite(motorEN2, 100);  // Slow left motor for final adjustment
        } else {
            stop_motors();  // Stop the robot completely
        }
    }
}

// Function to stop all motors
void stop_motors(void) {
    digitalWrite(motorR1, HIGH);
    digitalWrite(motorR2, HIGH);
    digitalWrite(motorL1, HIGH);
    digitalWrite(motorL2, HIGH);
    analogWrite(motorEN1, 0);  // Stop PWM for both motors
    analogWrite(motorEN2, 0);
}

// Function to calculate IMU sensor errors
void calculate_IMU_error() {
    // Loop to gather sensor data and calculate average error
    for (int i = 0; i < 200; i++) {
        // Read accelerometer and gyroscope values (similar to loop function)
        // Accumulate errors for each axis
        AccErrorX += AccX;
        AccErrorY += AccY;
        GyroErrorX += GyroX;
        GyroErrorY += GyroY;
        GyroErrorZ += GyroZ;
    }
    // Average the error over the 200 samples
    AccErrorX /= 200;
    AccErrorY /= 200;
    GyroErrorX /= 200;
    GyroErrorY /= 200;
    GyroErrorZ /= 200;
}