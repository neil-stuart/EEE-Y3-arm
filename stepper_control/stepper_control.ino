const int N_STEPPERS = 4;

const int PULSE_WIDTH = 10; //us

// Stepper pins, given as STEP, CW and EN pins in each entry. 
const int STEPPERS_PINS[N_STEPPERS][3] = {
  {2,3,4}, // BASE ROTATION STEPPER
  {5,6,7}, // BASE ARTICULATION DOUBLE STEPPER
  {8,9,10}, // ELBOW JOINT
  {A0,A1,A2} // GRABBER ARTICULATION
};

const int STEPPER_PARAMS[N_STEPPERS][3] = { 
  // Stepper parameters given as MIN_POSITION, MAX_POSITION, MAX_FREQUENCY
  {-1000,1000,1000},
  {-680,680,500},
  {-1500,1500,1500},
  {-500,500,500} // Initialize params for here 
};

// Each steppers status given as 
// {position, speed (in Hz), direction (1,-1), enabled/disabled, n_steps}
int stepper_status[N_STEPPERS][5] = {
  {0,0,1,0,0},
  {0,0,1,0,0},
  {0,0,1,0,0},
  {0,0,1,0,0}
};

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
bool pulseState[N_STEPPERS] = {false}; // Track if the pulse is HIGH or LOW for each stepper
unsigned long stepIntervals[N_STEPPERS] = {0}; // Initialize all to 0

void updateSteppers() {
  for(int i = 0; i < N_STEPPERS; i++) {
    unsigned long currentMicros = micros();
    
    int next_pos = stepper_status[i][0] + stepper_status[i][2];

    if(next_pos > STEPPER_PARAMS[i][0] && next_pos < STEPPER_PARAMS[i][1]){
      // Check if it's time to toggle the step pin
      if (!pulseState[i] && stepper_status[i][4] > 0 && currentMicros - lastStepTime[i] >= stepIntervals[i]) {
        
        // Start pulse
        digitalWrite(STEPPERS_PINS[i][0], HIGH);
        pulseState[i] = true; // Mark pulse as HIGH
        
        lastStepTime[i] = currentMicros; // Update the last step time to the start of the pulse
        stepper_status[i][4]--;

      } else if (pulseState[i] && currentMicros - lastStepTime[i] >= PULSE_WIDTH) { // Ensure pulse width of 10 microseconds
        
        // End pulse
        digitalWrite(STEPPERS_PINS[i][0], LOW);
        pulseState[i] = false; 

        stepper_status[i][0] += stepper_status[i][2];
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
      stepper_status[stepperID][3] = value;
      return;
    case 'F': // Change frequency of the stepper.
      value = command.substring(firstCommaIndex + 1).toInt();
      stepper_status[stepperID][1] = (value<=STEPPER_PARAMS[stepperID][2])?value:STEPPER_PARAMS[stepperID][2];
      stepIntervals[stepperID] = 1000000L / value;
      return;
    case 'D': // Change direction of the stepper.
      value = command.substring(firstCommaIndex + 1).toInt();
      stepper_status[stepperID][2] = (value==0)?1:-1;
      digitalWrite(STEPPERS_PINS[stepperID][1], value);
      return;
    case 'N':
      value = command.substring(firstCommaIndex + 1).toInt();
      stepper_status[stepperID][4] = value;
      return;
    case 'P': // Request position of stepper.
      int position = stepper_status[stepperID][0];
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
    stepper_status[i][3] = true;
  }
}

void disableAllSteppers() 
{
  for(int i = 0 ; i< N_STEPPERS; i++){
    digitalWrite(STEPPERS_PINS[i][2], HIGH);
    stepper_status[i][3] = false;
  }
}
