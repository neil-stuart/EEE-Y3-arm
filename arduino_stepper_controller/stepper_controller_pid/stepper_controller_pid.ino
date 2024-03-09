/*
Author: Neil Stuart
Group: 11
Email: b.stuart3@universityofgalway.ie
Date: March 2024

Project: Arduino Controller for Moveo Robotic Arm with setpoints for steppers and PID control.
University of Galway
*/

#include <PID_v1.h>

const int N_STEPPERS = 4;

const int PULSE_WIDTH = 10; //us

double Kp[N_STEPPERS] = {4.0, 9.0, 5.0, 5.0}; // Example values
double Ki[N_STEPPERS] = {5.0, 4.0, 3.0, 2.0}; // Example values
double Kd[N_STEPPERS] = {1.0, 1.5, 0.5, 1.0}; // Example values



// Stepper pins, given as STEP, CW and EN pins in each entry. 
const int STEPPERS_PINS[N_STEPPERS][3] = {
  {2,3,4}, // BASE ROTATION STEPPER
  {5,6,7}, // BASE ARTICULATION DOUBLE STEPPER
  {8,9,10}, // ELBOW JOINT
  {A0,A1,A2} // GRABBER ARTICULATION
};

const int STEPPER_PARAMS[N_STEPPERS][4] = { 
  // Stepper parameters given as MIN_POSITION, MAX_POSITION, MAX_FREQUENCY, MIN_F
  {-1000,1000,1000,500},
  {-680*4,4*680,3000,500},
  {-1800,1800,1500,450},
  {-500,500,500,50} // Initialize params for here 
};

// PID tuning parameters
double Kp = 2.0, Ki = 5.0, Kd = 1.0;

// PID Input, Output, and Setpoint arrays
double pidSetPoints[N_STEPPERS] = {0};
double pidInputs[N_STEPPERS] = {0};
double pidOutputs[N_STEPPERS] = {0};

// PID objects array
PID PIDArray[N_STEPPERS] = {
  PID(&pidInputs[0], &pidOutputs[0], &pidSetPoints[0], Kp[0], Ki[0], Kd[0], DIRECT),
  PID(&pidInputs[1], &pidOutputs[1], &pidSetPoints[1], Kp[1], Ki[1], Kd[1], DIRECT),
  PID(&pidInputs[2], &pidOutputs[2], &pidSetPoints[2], Kp[2], Ki[2], Kd[2], DIRECT),
  PID(&pidInputs[3], &pidOutputs[3], &pidSetPoints[3], Kp[3], Ki[3], Kd[3], DIRECT)
};



int setPoints[N_STEPPERS] = {0};

// Each steppers status given as 
// {position, direction (1,-1), enabled/disabled}
int stepperStatus[N_STEPPERS][3] = {0};


void setup() {
  Serial.begin(115200);
  for (int stepper = 0; stepper < N_STEPPERS; stepper++) {
    for (int pin = 0; pin < 3; pin++) {
      pinMode(STEPPERS_PINS[stepper][pin], OUTPUT);
    }
    PIDArray[stepper].SetMode(AUTOMATIC); // Sets the PID to automatic mode
    PIDArray[stepper].SetOutputLimits(STEPPER_PARAMS[stepper][3], STEPPER_PARAMS[stepper][2]); // Sets the min and max output
  }
}

void loop() {
  updateSteppers();
}

unsigned long lastStepTime[N_STEPPERS] = {0}; // Initialize all to 0
unsigned long stepIntervals[N_STEPPERS] = {0}; // Initialize all to 0

void updateSteppers() {
  for(int i = 0; i < N_STEPPERS; i++) {
    unsigned long currentMicros = micros();
    
    int next_pos = stepperStatus[i][0] + stepperStatus[i][1];

    if(next_pos > STEPPER_PARAMS[i][0] && next_pos < STEPPER_PARAMS[i][1]){
        // Check if it's time to toggle the step pin
        if (setPoints[i]!=stepperStatus[i][0] && currentMicros - lastStepTime[i] >= stepIntervals[i]) {
          
            // Start pulse
            digitalWrite(STEPPERS_PINS[i][0], HIGH);
            delayMicroseconds(PULSE_WIDTH); // Use blocking delay for the pulse width
            digitalWrite(STEPPERS_PINS[i][0], LOW);

            lastStepTime[i] = currentMicros; // Update the last step time to the end of the pulse

            // Update with PID speed.
            stepperStatus[i][0] += stepperStatus[i][1];
            stepIntervals[i] = 1000000L/getFrequencyPID(i);
        } 
    }

    digitalWrite(STEPPERS_PINS[i][0], LOW);
  }
}


// The serialEvent function runs in the background and is called after loop() if serial data is available.
void serialEvent() {
  static String receivedCommand; 
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == ';') {
      processCommand(receivedCommand);
      receivedCommand = ""; // Clear the command string after processing
    } else {
      receivedCommand += inChar; // Accumulate characters into the command string
    }
  }
}

// Calculate PID Speed based on error (setpoint-current position)
float getFrequencyPID(int stepperID) {
  pidInputs[stepperID] = stepperStatus[stepperID][0];
  pidSetPoints[stepperID] = setPoints[stepperID];
  
  if (PIDArray[stepperID].Compute()) {
    // Ensure the output is within the stepper's frequency limits
    return constrain(pidOutputs[stepperID], STEPPER_PARAMS[stepperID][3], STEPPER_PARAMS[stepperID][2]);
  }
  return 0; // In case PID computation was not performed
}

void processCommand(String command) {
  // Process the incoming command
  int stepperID, value;
  char cmd_char = command.charAt(0); // The command character
  int firstCommaIndex = command.indexOf(',');

  stepperID = command.substring(1,2).toInt();

  // Only accept commands that correspond to existing steppers
  if(stepperID<0 || stepperID>=N_STEPPERS){
    Serial.println("ERR");
    return;
  }

  switch (cmd_char) {
    case 'E': // Enable/disable stepper
      value = command.substring(firstCommaIndex + 1).toInt();
      digitalWrite(STEPPERS_PINS[stepperID][2],  value==0?HIGH:LOW);
      stepperStatus[stepperID][2] = value;
      return;
    case 'D': // Change direction of the stepper.
      value = command.substring(firstCommaIndex + 1).toInt();
      stepperStatus[stepperID][1] = (value==0)?1:-1;
      digitalWrite(STEPPERS_PINS[stepperID][1], value);
      return;
    case 'N':
      value = command.substring(firstCommaIndex + 1).toInt();
      setPoints[stepperID] = stepperStatus[stepperID][0] + (stepperStatus[stepperID][1]==1?value:-value);
      return;
    case 'P': // Request position of stepper.
      int position = stepperStatus[stepperID][0];
      Serial.print("P");
      Serial.print(stepperID);
      Serial.print(",");
      Serial.println(position);
      return;
  }
  Serial.println("NC"); // No command
}
