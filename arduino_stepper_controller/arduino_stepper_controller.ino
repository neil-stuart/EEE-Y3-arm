/*
Author: Neil Stuart
Group: 11
Email: b.stuart3@universityofgalway.ie
Date: March 2024

Project: Arduino Controller for Moveo Robotic Arm
University of Galway
*/

const int N_STEPPERS = 4;

const int PULSE_WIDTH = 10; //us

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

// Each steppers status given as 
// {position, speed (in Hz), direction (1,-1), enabled/disabled, n_steps}
int stepperStatus[N_STEPPERS][5] = {0};

// Use this to know the progress i.e. how many steps have been completed out of the last total as a percentage?
int nStepsLast[N_STEPPERS] = {0};
void setup() {
  
  Serial.begin(115200);
  for (int stepper = 0; stepper < N_STEPPERS; stepper++) {
    for (int pin = 0; pin < 3; pin++) {
      pinMode(STEPPERS_PINS[stepper][pin], OUTPUT);
    }
  }
  
  disableAllSteppers();
  delay(100);
  enableAllSteppers();
}

void loop() {
  updateSteppers();
}

unsigned long lastStepTime[N_STEPPERS] = {0}; // Initialize all to 0
unsigned long stepIntervals[N_STEPPERS] = {0}; // Initialize all to 0

void updateSteppers() {
  for(int i = 0; i < N_STEPPERS; i++) {
    unsigned long currentMicros = micros();
    
    int next_pos = stepperStatus[i][0] + stepperStatus[i][2];

    if(next_pos > STEPPER_PARAMS[i][0] && next_pos < STEPPER_PARAMS[i][1]){
        // Check if it's time to toggle the step pin
        if (stepperStatus[i][4] > 0 && currentMicros - lastStepTime[i] >= stepIntervals[i]) {
          
            // Start pulse
            digitalWrite(STEPPERS_PINS[i][0], HIGH);
            delayMicroseconds(PULSE_WIDTH); // Use blocking delay for the pulse width
            digitalWrite(STEPPERS_PINS[i][0], LOW);

            lastStepTime[i] = currentMicros; // Update the last step time to the end of the pulse
            stepperStatus[i][4]--;

            // Update with ease in out speed
            stepperStatus[i][0] += stepperStatus[i][2];
            float progress = ((float)stepperStatus[i][4])/((float)nStepsLast[i]);
            stepIntervals[i] = 1000000L/(getEaseInOutSpeed(progress)*(STEPPER_PARAMS[i][2]-STEPPER_PARAMS[i][3])+STEPPER_PARAMS[i][3]);
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

float getEaseInOutSpeed(float x){
        if(x > 0.0f && x < 1.0f){
            return (-4.0f * x * x + 4.0f *x);
        }
        return 0;
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
      stepperStatus[stepperID][3] = value;
      return;
    case 'F': // Change frequency of the stepper.
      value = command.substring(firstCommaIndex + 1).toInt();
      stepperStatus[stepperID][1] = (value<=STEPPER_PARAMS[stepperID][2])?value:STEPPER_PARAMS[stepperID][2];
      stepIntervals[stepperID] = 1000000L / value;
      return;
    case 'D': // Change direction of the stepper.
      value = command.substring(firstCommaIndex + 1).toInt();
      stepperStatus[stepperID][2] = (value==0)?1:-1;
      digitalWrite(STEPPERS_PINS[stepperID][1], value);
      return;
    case 'N':
      value = command.substring(firstCommaIndex + 1).toInt();
      nStepsLast[stepperID] = value;
      stepperStatus[stepperID][4] = value;
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

void enableAllSteppers() 
{
  for(int i = 0 ; i< N_STEPPERS; i++){
    digitalWrite(STEPPERS_PINS[i][2], LOW);
    stepperStatus[i][3] = true;
  }
}

void disableAllSteppers() 
{
  for(int i = 0 ; i< N_STEPPERS; i++){
    digitalWrite(STEPPERS_PINS[i][2], HIGH);
    stepperStatus[i][3] = false;
  }
}