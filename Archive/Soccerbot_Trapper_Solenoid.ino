#include <Servo.h>
int lightResSensitivity = 450; //LDR cut off value
Servo kickServo, trapperArmServo;
int lightResValue; //Value for light sensor
int maxServoPos = 140; //Base is 65
int kickServoPos = 0;

int maxTrapperArmPos = 50; // Base is 65
int trapperArmServoPos = 0;

const int ldrPin = A4; //LDR analog connection

const int kickSolenoidPin = 5;
const int trapperArmServoPin = 4;
const int inputToKickPin = A2; // Input for raspberry pi - if high then kick
const int ballTrappedOutputPin = 9;

int inputToKickPinVal;
bool ballIsTrapped = false;

char receivedChar;



void setup() {

  Serial.begin(9600);
  lightResValue = analogRead(ldrPin); //Read ldr and output a value
  SetLightResSens();

  trapperArmServo.attach(trapperArmServoPin);
  trapperArmServo.write(trapperArmServoPos);


  // Receive input from raspberry pi causing ball release and kick towards target
  pinMode(inputToKickPin,INPUT);

  // Kick with solenoid
  pinMode(kickSolenoidPin, OUTPUT);

  

  // Ball is trapped and as such send a high to relay to output 3.3V to raspberry pi
  pinMode(ballTrappedOutputPin, OUTPUT);
 
}

void SetLightResSens()
{
  lightResSensitivity = lightResValue + 350; //Sets LDR cut off value - Min 730
  Serial.println("Sensitivity limit is: ");
  Serial.println(lightResSensitivity);
}

void loop() 
{
  if (Serial.available() > 0 && ballIsTrapped)
  {
    //Implement kicking when signal received
    /*if(inputToKickPin > 400)
    {
      Serial.println("Releasing and kicking ball now: ");
      ReleaseAndKickBall();
    }
    */
    receivedChar = Serial.read();

    if(receivedChar == 'k')
    {
      Serial.println("Releasing and kicking ball now: ");
      ReleaseAndKickBall();
    }
    else
    {
      Serial.println("Enter k to kick");
    }
  }
  lightResValue = analogRead(ldrPin); //Read ldr and output a value
  inputToKickPinVal = analogRead(inputToKickPinVal);

  if (lightResValue >= lightResSensitivity && !ballIsTrapped)
  {
    Serial.println("Ball is trapped");
    TrapBall();
  }

delay(1);
}


void KickWithSolenoid()
{
  digitalWrite(kickSolenoidPin, HIGH);
  delay(10);
  digitalWrite(kickSolenoidPin, LOW);
}

void SendSignalToPi()
{
  digitalWrite(kickSolenoidPin, HIGH);
  delay(10);
}
void KickWithServo()
{/*
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
  */

  for (kickServoPos = 0; kickServoPos <= maxServoPos; kickServoPos+=3)
  {
    Serial.print("kickPos");
    Serial.println(kickServoPos);
    kickServo.write(kickServoPos);  
    delay(1);                       // waits for the servo to reach the position
  }
  for (kickServoPos = maxServoPos; kickServoPos >= 0; kickServoPos-=5)
  {
    kickServo.write(kickServoPos);              
    delay(3);                       // waits for the servo to reach the position
  }
  delay(15);
}

void AlterKickingArm(bool lowerArm)
{
  if(!lowerArm)
  {
    for (trapperArmServoPos = maxTrapperArmPos; trapperArmServoPos >= 0; trapperArmServoPos--)
    {
    trapperArmServo.write(trapperArmServoPos);  
    delay(3);                       // waits for the servo to reach the position
    }
  }
  else
  {
    for (trapperArmServoPos = 0; trapperArmServoPos <= maxTrapperArmPos; trapperArmServoPos++)
    {
    trapperArmServo.write(trapperArmServoPos);  
    delay(3);                       // waits for the servo to reach the position
    }
  }
}

void TrapBall()
{
  ballTrappedOutputPinVal = 400;
  ballIsTrapped = true;
  AlterKickingArm(true);
  Serial.println("Trapped ball and should look for goal now");
  digitalWrite(ballTrappedOutputPin, HIGH);
  delay(500);
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




