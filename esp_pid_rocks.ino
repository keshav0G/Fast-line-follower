// Motor A Pins (Left Motor)
#define ENA 19   // PWM pin for Motor A speed control
#define IN1 18   // Direction pin 1 for Motor A
#define IN2 5    // Direction pin 2 for Motor A

// Motor B Pins (Right Motor)
#define ENB 15   // PWM pin for Motor B speed control
#define IN3 2    // Direction pin 1 for Motor B
#define IN4 4    // Direction pin 2 for Motor B
// int stopPin = A5;
// bool stopflag = 1;

unsigned int wait = 300;
unsigned int wait2 = 500;
// Motor Definitions


// Define the analog pins for the IR sensors
const int sensorPins[6] = {13,12,14,27,26,25};
int sensorValues[6];  // Store sensor readings

const int white_limit = 100;  // Adjust based on calibration
const int black_limit = 1300;  // Adjust based on calibration

// PID Variables
float Kp = 0.01, Ki = 0, Kd = 0;
float error = 0, previousError = 0, totalError = 0;
int baseSpeed = 120;  // Base speed for both motors


void setup() {
  Serial.begin(9600);
    pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  

}

void readSensorValues() {
  for (int i = 0; i < 6; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }
}

float computeError() {
  // Weights for each sensor: [leftFar, leftNear, Centre, rightNear, rightFar]
  float weights[6] = {-2, -1, 0,0, 1, 2};  
  float weightedSum = 0;
  float totalActiveSensors = 0;

  for (int i = 0; i < 6; i++) {
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

  analogWrite(ENA,leftMotorSpeed);
  analogWrite(ENB,rightMotorSpeed);

  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);

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
//   stopflag =  digitalRead(stopPin);
//   uint16_t leftFar = sensorValues[0];
//   uint16_t leftNear = sensorValues[1];
//   uint16_t Centre = sensorValues[2];
//   uint16_t rightNear = sensorValues[3];
//   uint16_t rightFar = sensorValues[4];
//   if (leftFar < black_limit && leftNear < black_limit) {
// //     Serial.println("left"); 
//        moveLeft();
// //     delay(wait);  
// //     applyPIDControl();
// //     delay(wait2); 
//   }
//   else if (rightFar < black_limit && rightNear < black_limit && leftFar > white_limit && leftNear > white_limit) {
//     moveRight();
//   }
//   if (stopPin == 0) {
//     STOP(100);
//   }
//   else {
//    applyPIDControl();
//  }
  //delay(50);  // Small delay for readability
applyPIDControl();
}
