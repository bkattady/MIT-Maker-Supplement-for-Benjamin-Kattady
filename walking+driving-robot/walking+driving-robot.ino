#include <Servo.h>

void readEncoder1(); 
void readEncoder2(); 

Servo FRT, FRB, FLT, FLB, BRT, BRB, BLT, BLB;
int FRTPin = 53; int FRBPin = 52;  int FLTPin = 51; int FLBPin = 50;
int BRTPin = 49; int BRBPin = 48;  int BLTPin = 47; int BLBPin = 46;

const int pinLeftRPWM = 8;  
const int pinLeftLPWM = 13; 
const int pinRightRPWM = 6; 
const int pinRightLPWM = 7; 

const int encoderPinA = 2; const int encoderPinB = 3; 
const int encoderPinC = 18; const int encoderPinD = 19; 

volatile long encoderCount1 = 0; 
volatile long encoderCount2 = 0; 
unsigned long lastTelemetryTime = 0;

enum RobotState { CRUISE, APPROACH, WALKING, TRANSITION };
RobotState currentState = CRUISE;

unsigned long stateStartTime = 0;   

const unsigned long cruiseDuration   = 5000;  
const unsigned long approachDuration = 5000;  
const unsigned long walkingDuration  = 10000; 
const unsigned long transitionDuration = 2500; 

float Speed = 0.003;
float maxTop = 20.0;
float maxBot = 20.0;

float defaultFRT = 91;    float defaultFRB = 224.0;
float defaultFLT = 176.0; float defaultFLB = 48.0;
float defaultBRT = 84.0;  float defaultBRB = 220.0;
float defaultBLT = 185.0; float defaultBLB = 43.0;

float horizontalFRT = 223.0; float horizontalFRB = 224.0;
float horizontalFLT = 36.0;  float horizontalFLB = 48.0;
float horizontalBRT = 39.0;  float horizontalBRB = 40.0;
float horizontalBLT = 220.0; float horizontalBLB = 223.0;

float currentFRT, currentFRB, currentFLT, currentFLB;
float currentBRT, currentBRB, currentBLT, currentBLB;

const float downwardSpeedLimit = 9;  
const float upwardSpeedLimit = 0.25;    

void setup() {
  Serial.begin(9600);

  FRT.attach(FRTPin); FRB.attach(FRBPin); FLT.attach(FLTPin); FLB.attach(FLBPin);
  BRT.attach(BRTPin); BRB.attach(BRBPin); BLT.attach(BLTPin); BLB.attach(BLBPin);

  pinMode(pinLeftRPWM, OUTPUT);  pinMode(pinLeftLPWM, OUTPUT);
  pinMode(pinRightRPWM, OUTPUT); pinMode(pinRightLPWM, OUTPUT);

  pinMode(encoderPinA, INPUT_PULLUP); pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(encoderPinC, INPUT_PULLUP); pinMode(encoderPinD, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), readEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinC), readEncoder2, CHANGE);

  currentFRT = horizontalFRT; currentFRB = horizontalFRB;
  currentFLT = horizontalFLT; currentFLB = horizontalFLB;
  currentBRT = horizontalBRT; currentBRB = horizontalBRB;
  currentBLT = horizontalBLT; currentBLB = horizontalBLB;

  setAllLegsHorizontal();
  
  stateStartTime = millis();
  delay(3000); 
}

void loop() {
  float targetFRT, targetFRB, targetFLT, targetFLB;
  float targetBRT, targetBRB, targetBLT, targetBLB;

  unsigned long elapsedInState = millis() - stateStartTime;

  switch (currentState) {
    
    case CRUISE:
      targetFRT = horizontalFRT; targetFRB = horizontalFRB;
      targetFLT = horizontalFLT; targetFLB = horizontalFLB;
      targetBRT = horizontalBRT; targetBRB = horizontalBRB;
      targetBLT = horizontalBLT; targetBLB = horizontalBLB;

      analogWrite(pinLeftRPWM, 200);  analogWrite(pinLeftLPWM, 0);
      analogWrite(pinRightRPWM, 200); analogWrite(pinRightLPWM, 0);

      if (elapsedInState >= cruiseDuration) {
        currentState = APPROACH;
        stateStartTime = millis();
        Serial.println(">>> Time sequence change: Entering APPROACH (Slowing wheels)");
      }
      break;

    case APPROACH:
      targetFRT = horizontalFRT; targetFRB = horizontalFRB;
      targetFLT = horizontalFLT; targetFLB = horizontalFLB;
      targetBRT = horizontalBRT; targetBRB = horizontalBRB;
      targetBLT = horizontalBLT; targetBLB = horizontalBLB;

      analogWrite(pinLeftRPWM, 75);   analogWrite(pinLeftLPWM, 0);
      analogWrite(pinRightRPWM, 75);  analogWrite(pinRightLPWM, 0);

      if (elapsedInState >= approachDuration) {
        currentState = WALKING;
        stateStartTime = millis();
        Serial.println("Starting Walking");
      }
      break;

    case WALKING:
      analogWrite(pinLeftRPWM, 0);  analogWrite(pinLeftLPWM, 0);
      analogWrite(pinRightRPWM, 0); analogWrite(pinRightLPWM, 0);

      unsigned long currentTime = millis();
      float basePhase = currentTime * Speed;

      float frPhase = basePhase;
      float blPhase = basePhase + (3.0 * PI) / 2.0;  // Fixed missing *
      float flPhase = basePhase;
      float brPhase = basePhase + (3.0 * PI) / 2.0;  // Fixed missing *

      targetFRT = defaultFRT + sin(frPhase) * maxTop;
      targetFLT = defaultFLT + sin(flPhase) * maxTop;
      targetBRT = defaultBRT + sin(brPhase) * maxTop;
      targetBLT = defaultBLT + sin(blPhase) * maxTop;

      float frLift = max(0.0f, cos(frPhase));
      float flLift = max(0.0f, -cos(flPhase));
      float brLift = max(0.0f, cos(brPhase));
      float blLift = max(0.0f, -cos(blPhase));

      targetFRB = defaultFRB + frLift * maxBot;
      targetFLB = defaultFLB - flLift * maxBot;
      targetBRB = defaultBRB + brLift * maxBot;
      targetBLB = defaultBLB - blLift * maxBot;

      if (elapsedInState >= walkingDuration) {
        currentState = CRUISE;
        stateStartTime = millis();
        Serial.println("Switching to wheels");
      }
      break;

    case TRANSITION:
      analogWrite(pinLeftRPWM, 0);  analogWrite(pinLeftLPWM, 0);
      analogWrite(pinRightRPWM, 0); analogWrite(pinRightLPWM, 0);

      targetFRT = defaultFRT; targetFRB = defaultFRB;
      targetFLT = defaultFLT; targetFLB = defaultFLB;
      targetBRT = defaultBRT; targetBRB = defaultBRB;
      targetBLT = defaultBLT; targetBLB = defaultBLB;

      if (elapsedInState >= transitionDuration) {
        currentState = CRUISE;
        stateStartTime = millis();
        Serial.println("Making legs horizontal");
      }
      break;
  }

  // Constraints fixed for FL variables
  currentFRT = constrain(targetFRT, currentFRT - ((targetFRT < currentFRT) ? downwardSpeedLimit : upwardSpeedLimit), currentFRT + ((targetFRT > currentFRT) ? upwardSpeedLimit : downwardSpeedLimit));
  currentFRB = constrain(targetFRB, currentFRB - ((targetFRB < currentFRB) ? downwardSpeedLimit : upwardSpeedLimit), currentFRB + ((targetFRB > currentFRB) ? upwardSpeedLimit : downwardSpeedLimit));
  currentFLT = constrain(targetFLT, currentFLT - ((targetFLT < currentFLT) ? downwardSpeedLimit : upwardSpeedLimit), currentFLT + ((targetFLT > currentFLT) ? upwardSpeedLimit : downwardSpeedLimit));
  currentFLB = constrain(targetFLB, currentFLB - ((targetFLB < currentFLB) ? downwardSpeedLimit : upwardSpeedLimit), currentFLB + ((targetFLB > currentFLB) ? upwardSpeedLimit : downwardSpeedLimit));
  currentBRT = constrain(targetBRT, currentBRT - ((targetBRT < currentBRT) ? downwardSpeedLimit : upwardSpeedLimit), currentBRT + ((targetBRT > currentBRT) ? upwardSpeedLimit : downwardSpeedLimit));
  currentBRB = constrain(targetBRB, currentBRB - ((targetBRB < currentBRB) ? downwardSpeedLimit : upwardSpeedLimit), currentBRB + ((targetBRB > currentBRB) ? upwardSpeedLimit : downwardSpeedLimit));
  currentBLT = constrain(targetBLT, currentBLT - ((targetBLT < currentBLT) ? downwardSpeedLimit : upwardSpeedLimit), currentBLT + ((targetBLT > currentBLT) ? upwardSpeedLimit : downwardSpeedLimit));
  currentBLB = constrain(targetBLB, currentBLB - ((targetBLB < currentBLB) ? downwardSpeedLimit : upwardSpeedLimit), currentBLB + ((targetBLB > currentBLB) ? upwardSpeedLimit : downwardSpeedLimit));

  write270(FRT, currentFRT); write270(FRB, currentFRB);
  write270(FLT, currentFLT); write270(FLB, currentFLB);
  write270(BRT, currentBRT); write270(BRB, currentBRB);
  write270(BLT, currentBLT); write270(BLB, currentBLB);

  if (millis() - lastTelemetryTime >= 200) {
    printTelemetry();
    lastTelemetryTime = millis();
  }
}

void setAllLegsHorizontal() {
  write270(FRT, horizontalFRT); write270(FRB, horizontalFRB);
  write270(FLT, horizontalFLT); write270(FLB, horizontalFLB);
  write270(BRT, horizontalBRT); write270(BRB, horizontalBRB);
  write270(BLT, horizontalBLT); write270(BLB, horizontalBLB);
}

void write270(Servo &servoInstance, float degreeTarget) {
  degreeTarget = constrain(degreeTarget, 0.0f, 270.0f);
  long microSecs = 500 + degreeTarget * (2000.0 / 270.0);
  servoInstance.writeMicroseconds(microSecs);
}

void printTelemetry() {
  Serial.print("Stance: ");
  if (currentState == CRUISE) Serial.print("Driving");
  else if (currentState == APPROACH) Serial.print("Transition");
  else Serial.print("Walking");

  unsigned long elapsed = millis() - stateStartTime;
  Serial.print("Timer");
  Serial.print(elapsed / 1000.0);
  Serial.print("s");
  
  Serial.print("Left encoder"); Serial.print(encoderCount1);
  Serial.print("Right encoder"); Serial.println(encoderCount2);
}

void readEncoder1() {
  if (digitalRead(encoderPinA) == digitalRead(encoderPinB)) encoderCount1++;
  else encoderCount1--;
}

void readEncoder2() {
  if (digitalRead(encoderPinC) == digitalRead(encoderPinD)) encoderCount2++;
  else encoderCount2--;
}
