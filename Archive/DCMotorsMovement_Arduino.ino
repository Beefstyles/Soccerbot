#include <Servo.h>
int lightResSensitivity = 450; //LDR cut off value
Servo kickServo;
int lightResValue; //Value for light sensor
int maxServoPos = 65;
int kickServoPos = 0;

const int ldrPin = A4; //LDR analog connection
const int kickServoPin = 5;
const int forwardLeftActionPin = A0;
const int forwardRightActionPin = A1;
const int reverseLeftActionPin = A2;
const int reverseRightActionPin = A3;

int forwardLeftActionPinVal, reverseLeftActionPinVal, forwardRightActionPinVal, reverseRightActionPinVal;

const int MotorOnPinRight = 13;
const int ForwardMotorRight = 6;
const int ReverseMotorRight = 7;
const int MotorOnPinLeft = 11;
const int ForwardMotorLeft = 3;
const int ReverseMotorLeft = 2;
int servoValue;
bool forwardMotorRightOn, reverseMotorRightOn, forwardMotorLeftOn, reverseMotorLeftOn;



void setup() {

  Serial.begin(9600);
  lightResValue = analogRead(ldrPin); //Read ldr and output a value
  SetLightResSens();
  kickServo.attach(kickServoPin);
  kickServo.write(kickServoPos); // Zero out

  SetMotorBools(false,false,false,false);

  servoValue = 400;

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
}

void SetLightResSens()
{
  lightResSensitivity = lightResValue + 350; //Sets LDR cut off value - Min 730
  Serial.print("Sensitivity limit is: ");
  Serial.print(lightResSensitivity);
}

void loop() 
{
  lightResValue = analogRead(ldrPin); //Read ldr and output a value
  Serial.print("Analog read: ");
  Serial.println(lightResValue);
  forwardLeftActionPinVal = analogRead(forwardLeftActionPin);
  reverseLeftActionPinVal = analogRead(reverseLeftActionPin);
  forwardRightActionPinVal = analogRead(forwardRightActionPin);
  reverseRightActionPinVal = analogRead(reverseRightActionPin);

  if (lightResValue >= lightResSensitivity)
  {
    Serial.println("Would kick with servo now");
    KickWithServo();
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
    StopRightMotor();
    //ReverseMotorRightMethod();
  }

   if(forwardRightActionPinVal >= servoValue)
  {
    ForwardMotorRightMethod();
    StopLeftMotor();
    //ReverseMotorLeftMethod();
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

    forwardMotorRightOn = true;
    reverseMotorRightOn = false;
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
     reverseMotorRightOn = true;
     forwardMotorRightOn = false;
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

  forwardMotorLeftOn = true;
  reverseMotorLeftOn = false;

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

    reverseMotorLeftOn = true;
    forwardMotorLeftOn = false;
  }
}

void StopLeftMotor()
{
  digitalWrite(MotorOnPinLeft, LOW);
  delay(50);
  digitalWrite(ForwardMotorLeft, LOW);
  digitalWrite(ReverseMotorLeft, LOW);
  reverseMotorLeftOn = false;
  forwardMotorLeftOn = false;
}

void StopRightMotor()
{
  digitalWrite(MotorOnPinRight, LOW);
  delay(50);
  digitalWrite(ForwardMotorRight, LOW);
  digitalWrite(ReverseMotorRight, LOW);
  reverseMotorRightOn = false;
  forwardMotorRightOn = false;
}

void SetMotorBools(bool forwardMotorRightOnSet, bool reverseMotorRightOnSet, bool forwardMotorLeftOnSet, bool reverseMotorLeftOnSet)
{
  forwardMotorRightOn = forwardMotorRightOnSet;
  reverseMotorRightOn = reverseMotorRightOnSet;
  forwardMotorLeftOn = forwardMotorLeftOnSet;
  reverseMotorLeftOn = reverseMotorLeftOnSet;
}

void KickWithServo()
{
  for (kickServoPos = 0; kickServoPos <= maxServoPos; kickServoPos++)
  {
    kickServo.write(kickServoPos);  
    delay(3);                       // waits for the servo to reach the position
  }
  for (kickServoPos = maxServoPos; kickServoPos >= 0; kickServoPos--)
  {
    kickServo.write(kickServoPos);              
    delay(3);                       // waits for the servo to reach the position
  }
  delay(15);
}



