#define S1_ID 1
#define S2_ID 2
#define S3_ID 3
#define S4_ID 4

#define S1_STEP_PIN 2
#define S1_CW_PIN 3
#define S1_EN_PIN 4

#define S2_STEP 5
#define S2_CW_PIN 6
#define S2_EN_PIN 7

#define S3_STEP 8
#define S3_CW_PIN 9
#define S3_EN_PIN 10

#define S4_STEP A0
#define S4_CW_PIN A1
#define S4_EN_PIN A2

// Size of the data received.
// Has to be synchronised with the sending script packet length.
const byte DATA_SIZE = 28;

// An array to hold the received data.
char csv_in[DATA_SIZE];


// TODO : FIgure out max and min rotation angles 
int S1_position = 500;
int S2_position = 500;
int S3_position = 500;
int S4_position = 500;

int S1_speed;
int S2_speed;
int S3_speed;
int S4_speed;

bool S1_dir;
bool S2_dir;
bool S3_dir;
bool S4_dir;

const int S1_MAX_F = 500; //Hz 
const int S2_MAX_F = 1500;
const int S3_MAX_F = 1500;
const int S4_MAX_F = 1500;

// Tracking the last step time for non-blocking delay
unsigned long S1_lastStepTime = 0;
unsigned long S2_lastStepTime = 0;
unsigned long S3_lastStepTime = 0;
unsigned long S4_lastStepTime = 0;

bool S1_EN = false;
bool S2_EN = false;
bool S3_EN = false;
bool S4_EN = false;


void setup() {
  Serial.begin(115200);

  // Set motor control pins as outputs
  pinMode(S1_STEP_PIN, OUTPUT);
  pinMode(S1_CW_PIN, OUTPUT);
  pinMode(S1_EN_PIN, OUTPUT);

  pinMode(S2_STEP, OUTPUT);
  pinMode(S2_CW_PIN, OUTPUT);
  pinMode(S2_EN_PIN, OUTPUT);

  pinMode(S3_STEP, OUTPUT);
  pinMode(S3_CW_PIN, OUTPUT);
  pinMode(S3_EN, OUTPUT);

  pinMode(S4_STEP, OUTPUT);
  pinMode(S4_CW_PIN, OUTPUT);
  pinMode(S4_EN_PIN, OUTPUT);

  disableAllSteppers();
}

void loop() {
  // Update stepper motors
  enableAllSteppers();
  updateStepper(S1_STEP_PIN, S1_CW_PIN, S1_speed, S1_dir, S1_MAX_F, S1_lastStepTime);
  updateStepper(S2_STEP, S2_CW_PIN, S2_speed, S2_dir, S2_MAX_F, S2_lastStepTime);
  updateStepper(S3_STEP, S3_CW_PIN, S3_speed, S3_dir, S3_MAX_F, S3_lastStepTime);
  updateStepper(S4_STEP, S4_CW_PIN, S4_speed, S4_dir, S4_MAX_F, S4_lastStepTime);
}

void updateStepper(int stepPin, int dirPin, int speed, bool dir, int maxFreq, unsigned long &lastStepTime) {
  if (speed == 0) {
    // If speed is 0, no need to step the motor.
    return;
  }

  // Calculate the interval between steps in microseconds.
  // For speed 100, interval is 1/maxFreq in seconds, converted to microseconds.
  // For speed 1, it's much slower, and we avoid division by zero or infinite interval.
  unsigned long stepInterval = map(speed, 0, 100, 1000000L / maxFreq, 1000000L / (maxFreq / 100));

  // Ensure the direction is set
  digitalWrite(dirPin, dir ? HIGH : LOW);

  // Check if it's time to step
  if (micros() - lastStepTime >= stepInterval) {
    // Toggle the step pin to create a pulse
    digitalWrite(stepPin, !digitalRead(stepPin)); // Simplified toggle for clarity

    // Remember the time of this step
    lastStepTime = micros();
  }
}




// The serialEvent function runs in the background and is called after loop() if serial data is available.
void serialEvent() {
  static String receivedCommand; // Holds the incoming command

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
  int motorID, value;
  char cmd = command.charAt(0); // The command character

  int firstCommaIndex = command.indexOf(',');

  if(firstCommaIndex != -1) { // Commands with parameters
    motorID = command.substring(1, firstCommaIndex).toInt();
    if (cmd == 'S' || cmd == 'D') { // Commands that require a value
      value = command.substring(firstCommaIndex + 1).toInt();
    }
  } else { // Commands without additional parameters
    motorID = command.substring(1).toInt();
  }

  switch (cmd) {
    case 'S':
      setSpeed(motorID, value);
      break;
    case 'D':
      setDirection(motorID, value);
      break;
    case 'P':
      int position = getCurrentPosition(motorID);
      Serial.print("P");
      Serial.print(motorID);
      Serial.print(",");
      Serial.println(position);
      break;
    // case 'E':
    //   setEnabled(motorID, value);
    //   int enabled = getEnabled(motorID);
    //   Serial.print("E");
    //   Serial.print(motorID);
    //   Serial.print(",");
    //   Serial.println(enabled);
    //   break;
  }
}

// Adjust your motor control functions to also accept the motorID parameter
void setSpeed(int motorID, int speed) {
  // Set the speed for the specified motor
  switch(motorID){
    case S1_ID:
      S1_speed = speed; 
      break;
    case S2_ID:
      S2_speed = speed; 
      break;
    case S3_ID:
      S3_speed = speed;
      break;
    case S4_ID:
      S4_speed = speed;
      break;
  }
}

void setDirection(int motorID, int direction) {
  // Set the direction for the specified motor
  switch(motorID){
    case S1_ID:
      S1_dir = direction;
      break;
    case S2_ID:
      S2_dir = direction;
      break;
    case S3_ID:
      S3_dir = direction;
      break;
    case S4_ID:
      S4_dir = direction;
      break;
  }
}

int getCurrentPosition(int motorID) {
  // Return the current position for the specified motor
  // Placeholder implementation
  switch(motorID){
    case S1_ID:
      return S1_position;
    case S2_ID:
      return S2_position;
    case S3_ID:
      return S3_position;
    case S4_ID:
      return S4_position;
  }
}

bool getEnabled(int motorID){
  switch(motorID){
    case S1_ID:
      return S1_EN;
    case S2_ID:
      return S2_EN;
    case S3_ID:
      return S3_EN;
    case S4_ID:
      return S4_EN;
  }
}
void setEnabled(int motorID, int val){
  switch(motorID){
    case S1_ID:
      digitalWrite(S1_EN_PIN, val);
      S1_EN = val;
      break;
    case S2_ID:
      digitalWrite(S1_EN_PIN, val);
      S2_EN = val;
      break;
    case S3_ID:
      digitalWrite(S1_EN_PIN, val);
      S3_EN = val;
      break;
    case S4_ID:
      digitalWrite(S1_EN_PIN, val);
      S4_EN = val;
      break;
  }
}

void enableAllSteppers() 
{
  digitalWrite(S1_EN_PIN, LOW);
  digitalWrite(S2_EN_PIN, LOW);
  digitalWrite(S3_EN, LOW);
  digitalWrite(S4_EN_PIN, LOW);
  S1_EN = true;
  S2_EN = true;
  S3_EN = true;
  S4_EN = true;
}

void disableAllSteppers() 
{
  digitalWrite(S1_EN_PIN, HIGH);
  digitalWrite(S2_EN_PIN, HIGH);
  digitalWrite(S3_EN, HIGH);
  digitalWrite(S4_EN_PIN, HIGH);
  S1_EN = false;
  S2_EN = false;
  S3_EN = false;
  S4_EN = false;

}
