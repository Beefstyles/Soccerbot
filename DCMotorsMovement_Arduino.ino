#include <Servo.h>
Servo mainServo;

int pos = 0;
int startSweepPin = A4;
int startSweepPinVal;
int SweepDoneRelayPin = 8;
int forwardLeftActionPin = A0;
int forwardRightActionPin = A1;
int reverseLeftActionPin = A2;
int reverseRightActionPin = A3;
int forwardLeftActionPinVal, reverseLeftActionPinVal, forwardRightActionPinVal, reverseRightActionPinVal;
bool searchStarted;
int zeroPos = 0;
int maxPos = 160;

int MotorOnPinRight = 13;
int ForwardMotorRight = 6;
int ReverseMotorRight = 7;
int MotorOnPinLeft = 11;
int ForwardMotorLeft = 3;
int ReverseMotorLeft = 2;
int servoValue;
bool forwardMotorRightOn, reverseMotorRightOn, forwardMotorLeftOn, reverseMotorLeftOn;

void setup() {
  //Setup for the servo sweep functionality
  searchStarted = false;
  SetMotorBools(false,false,false,false);

  servoValue = 400;
  pinMode(startSweepPin, INPUT);
  pinMode(SweepDoneRelayPin, OUTPUT);
  digitalWrite(SweepDoneRelayPin, LOW);

  //Setup for the DC motor movement functionality
  pinMode(MotorOnPinRight, OUTPUT);
  pinMode(ForwardMotorRight, OUTPUT);
  pinMode(ReverseMotorRight, OUTPUT);
  pinMode(MotorOnPinLeft, OUTPUT);
  pinMode(ForwardMotorLeft, OUTPUT);
  pinMode(ReverseMotorLeft, OUTPUT);

  pinMode(forwardLeftActionPin, INPUT);
  pinMode(forwardRightActionPin, INPUT);
  pinMode(reverseLeftActionPin, INPUT);
  pinMode(reverseRightActionPin, INPUT);
  
  digitalWrite(MotorOnPinRight, LOW);
  digitalWrite(ForwardMotorRight, LOW);
  digitalWrite(ReverseMotorRight, LOW);
  digitalWrite(MotorOnPinLeft, LOW);
  digitalWrite(ForwardMotorLeft, LOW);
  digitalWrite(ReverseMotorLeft, LOW);

  Serial.begin(9600);
  mainServo.attach(9);
  mainServo.write(90);
  mainServo.detach();
}

void loop() 
{
  startSweepPinVal = analogRead(startSweepPin);
  forwardLeftActionPinVal = analogRead(forwardLeftActionPin);
  reverseLeftActionPinVal = analogRead(reverseLeftActionPin);
  forwardRightActionPinVal = analogRead(forwardRightActionPin);
  reverseRightActionPinVal = analogRead(reverseRightActionPin);
  
  if(startSweepPinVal >= servoValue)
  {
    if(searchStarted == false)
    {
      Serial.println("Search started");
      searchStarted = true;
      RotateServo();
    }
  }
 if(forwardLeftActionPinVal >= servoValue && forwardRightActionPinVal >= servoValue)
  {
    ForwardMotorLeftMethod();
    ForwardMotorRightMethod();
    goto bailFromUpdate;
  }

   if(forwardLeftActionPinVal >= servoValue)
  {
    ForwardMotorLeftMethod();
    ReverseMotorRightMethod();
  }

   if(forwardRightActionPinVal >= servoValue)
  {
    ForwardMotorRightMethod();
    ReverseMotorLeftMethod();
  }

  if(reverseLeftActionPinVal >= servoValue && reverseRightActionPinVal >= servoValue)
  {
    ReverseMotorLeftMethod();
    ReverseMotorRightMethod();
    goto bailFromUpdate;
  }

  if(reverseLeftActionPinVal >= servoValue)
  {
    //StopRightMotor();
    ReverseMotorLeftMethod();
  }
  
   if(reverseRightActionPinVal >= servoValue)
  {
    //StopLeftMotor();
    ReverseMotorRightMethod();
  }
  if(forwardLeftActionPinVal <= servoValue && forwardRightActionPinVal <= servoValue && reverseLeftActionPinVal <= servoValue && reverseRightActionPinVal <= servoValue)
  {
    StopLeftMotor();
    StopRightMotor();
  }
  
  bailFromUpdate:



delay(1);
}

void ForwardMotorRightMethod()
{
  if(!forwardMotorRightOn)
  {
    digitalWrite(MotorOnPinRight, LOW);
    delay(50);
    digitalWrite(ReverseMotorRight, LOW);
    digitalWrite(ForwardMotorRight, HIGH);
    delay(50);
    digitalWrite(MotorOnPinRight, HIGH);
    //SetMotorBools(bool forwardMotorRightOnSet, bool reverseMotorRightOnSet, bool forwardMotorLeftOnSet, bool reverseMotorLeftOnSet)
    forwardMotorRightOn = true;
    //bool forwardMotorRightOn, reverseMotorRightOn, forwardMotorLeftOn, reverseMotorLeftOn;
    SetMotorBools(forwardMotorRightOn,false,false,false);
  }
}

void ReverseMotorRightMethod()
{
  if(!reverseMotorRightOn)
  {
    digitalWrite(MotorOnPinRight, LOW);
    delay(50);
    digitalWrite(ForwardMotorRight, LOW);
    digitalWrite(ReverseMotorRight, HIGH);
    delay(50);
    digitalWrite(MotorOnPinRight, HIGH);
    //SetMotorBools(bool forwardMotorRightOnSet, bool reverseMotorRightOnSet, bool forwardMotorLeftOnSet, bool reverseMotorLeftOnSet)
    reverseMotorRightOn = true;
    SetMotorBools(false,reverseMotorRightOn,false,false);
  }
}

void ForwardMotorLeftMethod()
{
  if(!forwardMotorLeftOn)
  {
    digitalWrite(MotorOnPinLeft, LOW);
  delay(50);
  digitalWrite(ReverseMotorLeft, LOW);
  digitalWrite(ForwardMotorLeft, HIGH);
  delay(50);
  digitalWrite(MotorOnPinLeft, HIGH);
  //SetMotorBools(bool forwardMotorRightOnSet, bool reverseMotorRightOnSet, bool forwardMotorLeftOnSet, bool reverseMotorLeftOnSet)
  forwardMotorLeftOn = true;
  SetMotorBools(false,false,forwardMotorLeftOn,false);
  }  
}

void ReverseMotorLeftMethod()
{
  if(!reverseMotorLeftOn)
  {
    digitalWrite(MotorOnPinLeft, LOW);
    delay(50);
    digitalWrite(ForwardMotorLeft, LOW);
    digitalWrite(ReverseMotorLeft, HIGH);
    delay(50);
    digitalWrite(MotorOnPinLeft, HIGH);
    //SetMotorBools(bool forwardMotorRightOnSet, bool reverseMotorRightOnSet, bool forwardMotorLeftOnSet, bool reverseMotorLeftOnSet)
    reverseMotorLeftOn = true;
    SetMotorBools(false,false,false,reverseMotorLeftOn);
  }
}

void StopLeftMotor()
{
  digitalWrite(MotorOnPinLeft, LOW);
  delay(50);
  digitalWrite(ForwardMotorLeft, LOW);
  digitalWrite(ReverseMotorLeft, LOW);
}

void StopRightMotor()
{
  digitalWrite(MotorOnPinRight, LOW);
  delay(50);
  digitalWrite(ForwardMotorRight, LOW);
  digitalWrite(ReverseMotorRight, LOW);
}

void RotateServo()
{
  mainServo.attach(9);
  for (pos = zeroPos; pos <= maxPos; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    mainServo.write(pos);
    //Serial.print("Pos is currently: ");
    //Serial.println(pos);
    delay(30);                       // waits 30ms for the servo to reach the position
  }
  digitalWrite(SweepDoneRelayPin, HIGH);
  CentreServo(pos);
}

void CentreServo(int startPos)
{
    for (pos = startPos; pos >= zeroPos; pos -= 1) { // goes from 180 degrees to 0 degrees
    mainServo.write(pos);              // tell servo to go to position in variable 'pos'
    //Serial.print("Pos is currently: ");
    //Serial.println(pos);
    delay(15);                       // waits 30ms for the servo to reach the position
  }
  searchStarted = false;
  mainServo.detach();
  digitalWrite(SweepDoneRelayPin, LOW);
}

void SetMotorBools(bool forwardMotorRightOnSet, bool reverseMotorRightOnSet, bool forwardMotorLeftOnSet, bool reverseMotorLeftOnSet)
{
  forwardMotorRightOn = forwardMotorRightOnSet;
  reverseMotorRightOn = reverseMotorRightOnSet;
  forwardMotorLeftOn = forwardMotorLeftOnSet;
  reverseMotorLeftOn = reverseMotorLeftOnSet;
}



