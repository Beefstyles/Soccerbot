#include <Servo.h>
int lightResSensitivity = 450; //LDR cut off value
Servo trapperArmServo;
int lightResValue; //Value for light sensor
int maxServoPos = 65;

int maxTrapperArmPos = 65;
int trapperArmServoPos = 0;

const int ldrPin = A4; //LDR analog connection

const int kickSolenoidPin = 5;
const int trapperArmServoPin = 4;

const int ballTrappedOutputPin = 12; // Ball is trapped so sned 3.3V to raspberry pi - via relay

bool ballIsTrapped = false;

const int forwardLeftActionPin = A0;
const int forwardRightActionPin = A1;
const int inputToKickPin = A2; // Input for raspberry pi - if high then kick


int forwardLeftActionPinVal, forwardRightActionPinVal, inputToKickPinVal;

const int MotorOnPinRight = 13;
const int ForwardMotorRight = 6;
const int ReverseMotorRight = 7;
const int MotorOnPinLeft = 11;
const int ForwardMotorLeft = 3;
const int ReverseMotorLeft = 2;

int raspberryPiAnalogHighValue;
bool forwardMotorRightOn, reverseMotorRightOn, forwardMotorLeftOn, reverseMotorLeftOn;

void setup() {

  Serial.begin(9600);
  lightResValue = analogRead(ldrPin); //Read ldr and output a value
  SetLightResSens();

  trapperArmServo.attach(trapperArmServoPin);
  trapperArmServo.write(trapperArmServoPos);

  SetMotorBools(false,false,false,false);

  raspberryPiAnalogHighValue = 400;

  //Setup for the DC motor movement functionality
  pinMode(MotorOnPinRight, OUTPUT);
  pinMode(ForwardMotorRight, OUTPUT);
  pinMode(ReverseMotorRight, OUTPUT);
  pinMode(MotorOnPinLeft, OUTPUT);
  pinMode(ForwardMotorLeft, OUTPUT);
  pinMode(ReverseMotorLeft, OUTPUT);
  pinMode(kickSolenoidPin, OUTPUT);

  pinMode(forwardLeftActionPin, INPUT);
  pinMode(forwardRightActionPin, INPUT);

  // Receive input from raspberry pi causing ball release and kick towards target
  pinMode(inputToKickPin, INPUT);

  // Ball is trapped and as such send a 3.3V high to the raspberry pi pin
  pinMode(ballTrappedOutputPin, OUTPUT);
  
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

  forwardLeftActionPinVal = analogRead(forwardLeftActionPin);
  forwardRightActionPinVal = analogRead(forwardRightActionPin);

  if (lightResValue >= lightResSensitivity && !ballIsTrapped)
  {
    Serial.println("Ball is trapped");
    TrapBall();
  }

  inputToKickPinVal = analogRead(inputToKickPin);

  if (ballIsTrapped && inputToKickPinVal >= raspberryPiAnalogHighValue)
  {
    Serial.println("Releasing and kicking ball now: ");
    ReleaseAndKickBall();
  }
  
 if(forwardLeftActionPinVal >= raspberryPiAnalogHighValue && forwardRightActionPinVal >= raspberryPiAnalogHighValue)
  {
    ForwardMotorLeftMethod();
    ForwardMotorRightMethod();
    goto bailFromUpdate;
  }

   if(forwardLeftActionPinVal >= raspberryPiAnalogHighValue)
  {
    ForwardMotorLeftMethod();
    StopRightMotor();
  }

   if(forwardRightActionPinVal >= raspberryPiAnalogHighValue)
  {
    ForwardMotorRightMethod();
    StopLeftMotor();
  }

  if(forwardLeftActionPinVal <= raspberryPiAnalogHighValue && forwardRightActionPinVal <= raspberryPiAnalogHighValue)
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

void AlterKickingArm(bool lowerArm)
{
  if(!lowerArm)
  {
    for (trapperArmServoPos = maxTrapperArmPos; trapperArmServoPos >= 0; trapperArmServoPos--)
    {
    trapperArmServo.write(trapperArmServoPos);  
    delay(3); // waits for the servo to reach the position
    }
  }
  else
  {
    for (trapperArmServoPos = 0; trapperArmServoPos <= maxTrapperArmPos; trapperArmServoPos++)
    {
    trapperArmServo.write(trapperArmServoPos);  
    delay(3);  // waits for the servo to reach the position
    }
  }
}

void KickWithSolenoid()
{
  digitalWrite(kickSolenoidPin, HIGH);
  delay(1000);
  digitalWrite(kickSolenoidPin, LOW);
  delay(1000);
}

void TrapBall()
{
  ballIsTrapped = true;
  AlterKickingArm(true);
  Serial.println("Trapped ball and should look for goal now");
  digitalWrite(ballTrappedOutputPin, HIGH);
  delay(100);
}

void ReleaseAndKickBall()
{
  AlterKickingArm(false);
  delay(5);
  KickWithSolenoid();
  ballIsTrapped = false;
  digitalWrite(ballTrappedOutputPin, LOW);
  delay(2000);
}




