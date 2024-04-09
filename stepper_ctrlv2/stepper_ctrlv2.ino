#include <PID_v1.h>

const int N_STEPPERS = 4;

const int PULSE_WIDTH = 10; //us

// In the order STEP, CW, EN.
const int STEPPERS_PINS[N_STEPPERS][3] = {
  {2,3,4}, // BASE ROTATION STEPPER
  {5,6,7}, // BASE ARTICULATION DOUBLE STEPPER
  {8,9,10}, // ELBOW JOINT
  {A0,A1,A2} // GRABBER ARTICULATION
};

const int STEPPER_PARAMS[N_STEPPERS][4] = { 
  // Stepper parameters given as MIN_POSITION, MAX_POSITION, MIN_FREQUENCY, MAX_FREQUENCY
  {-1000,1000,300,700}, /// 1500
  {-680*4,4*680,500,1000}, // 3000
  {-1800,1800,300,1000}, // 4000
  {-500,500,300,400} // 1000 
};



// Define variables
double input[N_STEPPERS];
double output[N_STEPPERS];
double setpoint[N_STEPPERS];

// Define PID objects
PID* pidControllers[N_STEPPERS];
double Kp[N_STEPPERS] = {3, 2, 3, 3}; // Example PID parameters for each controller
double Ki[N_STEPPERS] = {0, 0, 0, 0};
double Kd[N_STEPPERS] = {0.1, 0.1, 0.1, 0.1};

void setup() {
  Serial.begin(115200);
  
  for (int stepper = 0; stepper < N_STEPPERS; stepper++) {
    for (int pin = 0; pin < 3; pin++) {
      pinMode(STEPPERS_PINS[stepper][pin], OUTPUT);
    }

    // Initialize PID controllers with individual parameters
    pidControllers[stepper] = new PID(&input[stepper], &output[stepper], &setpoint[stepper], Kp[stepper], Ki[stepper], Kd[stepper], DIRECT);
    pidControllers[stepper]->SetMode(AUTOMATIC); // Set PID to automatic mode
    pidControllers[stepper]->SetOutputLimits(-STEPPER_PARAMS[stepper][3], STEPPER_PARAMS[stepper][3]); // Set output limits based on stepper parameters
    pidControllers[stepper]->SetSampleTime(1);
  }
}

void loop() {
  for (int i = 0; i < N_STEPPERS; i++) {
    pidControllers[i]->Compute(); // Calculate PID output
  }
  updateSteppers(); // Update stepper motors based on PID output
}

unsigned long lastStepTime[N_STEPPERS] = {0};
unsigned long stepIntervals[N_STEPPERS] = {0};

void updateSteppers() {
  for(int i = 0; i < N_STEPPERS; i++) {
    unsigned long currentMicros = micros();
  

    stepIntervals[i] = 1000000L / abs(output[i]);
    if(abs(output[i])<40){
      continue;
    }
    if (currentMicros - lastStepTime[i] >= stepIntervals[i]) {
      lastStepTime[i] = micros(); // Update the last step time

      if(output[i] == 0){
        continue;
      } else if (output[i] > 0) {
        digitalWrite(STEPPERS_PINS[i][1], HIGH); // Set direction pin for CW
      } else {
        digitalWrite(STEPPERS_PINS[i][1], LOW); // Set direction pin for CCW
      }
      
      digitalWrite(STEPPERS_PINS[i][0], HIGH); // Step pulse
      delayMicroseconds(PULSE_WIDTH);
      digitalWrite(STEPPERS_PINS[i][0], LOW);

      input[i] += output[i] > 0 ? 1 :-1; // Update position status

    } 

  }
}

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
    case 'S': // Change the setpoint
      value = command.substring(firstCommaIndex + 1).toInt();
      // Constrain within allowable range
      if(value > STEPPER_PARAMS[stepperID][0] && value < STEPPER_PARAMS[stepperID][1]){ 
        setpoint[stepperID] = value;
      }
      return;
    case 'P': // Request position of stepper.
      int position = input[stepperID];
      Serial.print("P");
      Serial.print(stepperID);
      Serial.print(",");
      Serial.println(position);
      return;
  }
  Serial.println("NC"); // No command
}
