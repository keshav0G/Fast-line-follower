//code for arduino uno with adafruit l293 motor driver sheild

#include <AFMotor.h>

int stopPin = A5;
bool stopflag = 1;

unsigned int wait = 300;
unsigned int wait2 = 500;
// Motor Definitions
AF_DCMotor motorRight(4);
AF_DCMotor motorLeft(3);


const int sensorPins[5] = {A0, A1, A2, A3, A4};
int sensorValues[5];  


const int white_limit = 200;  // Adjust based on calibration
const int black_limit = 100; 

// PID Variables
float Kp = 114.5, Ki = 0.0, Kd =27.5;
float error = 0, previousError = 0, totalError = 0;
int baseSpeed = 200;  

void Forward(){
  // motorLeft.setSpeed(baseSpeed);
  // motorRight.setSpeed(baseSpeed);
  // motorLeft.run(FORWARD);  // Stop the left motor
  // motorRight.run(FORWARD);
  applyPIDControl();
  
}

void STOP(int stoptime) {
  motorLeft.run(RELEASE);  // Stop the left motor
  motorRight.run(RELEASE);
  delay(stoptime); // Stop the right motor
}

// Function to move right at the specified speeds for left and right motors
void moveRight() {
   motorLeft.setSpeed(baseSpeed);
  motorRight.setSpeed(baseSpeed);
  motorLeft.run(FORWARD);               // Turn left motor forward
  motorRight.run(BACKWARD);             // Turn right motor backward
}

// Function to move left at the specified speeds for left and right motors
void moveLeft() {
   motorLeft.setSpeed(baseSpeed);
  motorRight.setSpeed(baseSpeed);
  motorLeft.run(BACKWARD);             
  motorRight.run(FORWARD);              // Turn right motor forward
}

void Backward(){   // Set speed for left motor
 motorLeft.setSpeed(baseSpeed);
  motorRight.setSpeed(baseSpeed);
  motorLeft.run(BACKWARD);           
  motorRight.run(FORWARD);
}

void setup() {
  Serial.begin(9600);
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);  // Initialize Serial communication
  pinMode(A3,INPUT);
  pinMode(A4,INPUT);
  pinMode(A5,INPUT);

}

void readSensorValues() {
  for (int i = 0; i < 5; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }
}

float computeError() {
  // Weights for each sensor: [leftFar, leftNear, Centre, rightNear, rightFar]
  float weights[5] = {-2, -1, 0, 1, 2};  
  float weightedSum = 0;
  float totalActiveSensors = 0;

  for (int i = 0; i < 5; i++) {
    if (sensorValues[i] < black_limit) {  // Sensor on the line
      weightedSum += weights[i];
      totalActiveSensors++;
    }
  }

  if (totalActiveSensors == 0) {
    return (previousError > 0) ? 2 : -2;  // Turn in the direction of previous error
  }

  return weightedSum / totalActiveSensors;  // Average error
}

void applyPIDControl() {
  error = computeError();
  float P = error;
  totalError += error;
  float I = totalError;
  float D = error - previousError;

  float pidOutput = (Kp * P) + (Ki * I) + (Kd * D);

  int leftMotorSpeed = baseSpeed + pidOutput;
  int rightMotorSpeed = baseSpeed - pidOutput;

  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

  motorLeft.setSpeed(leftMotorSpeed);
  motorRight.setSpeed(rightMotorSpeed);

  motorLeft.run(FORWARD);
  motorRight.run(FORWARD);

  previousError = error;
}


// Case-based handling for special scenarios
// void handleSpecialCases() {
//   uint16_t leftFar = sensorValues[0];
//   uint16_t leftNear = sensorValues[1];
//   uint16_t Centre = sensorValues[2];
//   uint16_t rightNear = sensorValues[3];
//   uint16_t rightFar = sensorValues[4];

  
//    if (leftFar > white_limit && leftNear > white_limit && rightFar > white_limit && rightNear > white_limit && Centre < black_limit) {
//     Serial.println("Command: Move Forward");
//     Forward();
//     // delay(wait);
//     // applyPIDControl();
//     // delay(wait2); 
     
//     return;      // No further checks needed after moving forward
//   }

//   // Case 2: If only the left far sensor detects black (line), turn left
//   // if (leftFar < black_limit && leftNear < black_limit && rightFar > white_limit && rightNear > white_limit) {
//   //   Serial.println("Command: Turn Left");
//   //   moveLeft();
//   //   delay(wait); 
//   //   applyPIDControl();
//   //   delay(wait2); 
    
      
//   //   return;    
//   // }

//   // Case 3: If only the right far sensor detects black, turn right
//   if (rightFar < black_limit && rightNear < black_limit && leftFar > white_limit && leftNear > white_limit) {
//     if(Centre > white_limit){
//       Serial.println("Command: Turn Right");
//       moveRight();
//       delay(wait);
//       applyPIDControl();
//       delay(wait2); 
//        // Adjust this delay for a smooth right turn
//       return;      // No further checks needed after turning right
//     }else{
//       Serial.println("Command: Move Forward");
//       Forward();
//       delay(wait);  
//       applyPIDControl();
//       delay(wait2); 
    
//       return;      // No further checks needed after moving forward
//     }

//   }

//   // Case 4: left above all
//   if (leftFar < black_limit && leftNear < black_limit) {
//     Serial.println("left"); 
//     moveLeft();
//     delay(wait);  
//     applyPIDControl();
//     delay(wait2); 
    
//     return;      // No further checks needed after moving forward
//   }

//   // Case 5: If all sensors detect white (off the line), move backward or stop
//   if (leftFar > white_limit && Centre > white_limit &&
//       rightFar > white_limit && leftNear > white_limit && rightNear > white_limit) {
//     Serial.println("Command: Move Backward");
//     Backward();
//     delay(200); 
//     applyPIDControl();
//     delay(wait2); 
    
//       // Move backward for 50ms
//     return;      // No further checks needed after moving backward
//   }

//   // Default Case: If nothing matches, move forward cautiously (fail-safe)
//   Serial.println("Command: Move Forward (Default)");  // Move forward cautiously for 50ms

//   applyPIDControl();
//}

void loop() {
  readSensorValues();

applyPIDControl();
}
