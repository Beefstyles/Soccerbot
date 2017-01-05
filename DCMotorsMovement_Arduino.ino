int MotorOnPinRight = 13;
int ForwardMotorRight = 6;
int ReverseMotorRight = 7;
int MotorOnPinLeft = 11;
int ForwardMotorLeft = 3;
int ReverseMotorLeft = 2;

void setup() {
  pinMode(MotorOnPinRight, OUTPUT);
  pinMode(ForwardMotorRight, OUTPUT);
  pinMode(ReverseMotorRight, OUTPUT);

  pinMode(MotorOnPinLeft, OUTPUT);
  pinMode(ForwardMotorLeft, OUTPUT);
  pinMode(ReverseMotorLeft, OUTPUT);

  Serial.begin(9600);

  digitalWrite(MotorOnPinRight, LOW);
  digitalWrite(ForwardMotorRight, LOW);
  digitalWrite(ReverseMotorRight, LOW);

  digitalWrite(MotorOnPinLeft, LOW);
  digitalWrite(ForwardMotorLeft, LOW);
  digitalWrite(ReverseMotorLeft, LOW);
}

void loop() 
{
  Serial.println("Forward Right Wheel!");
  ForwardMotorRightMethod();
  StopLeftMotor();
  delay(3000);
  Serial.println("Backward Right Wheel!!");
  ReverseMotorRightMethod();
  delay(3000);
  Serial.println("Forward Left Wheel!");
  ForwardMotorLeftMethod();
  StopRightMotor();
  delay(3000);
  Serial.println("Backward Left Wheel!!");
  ReverseMotorLeftMethod();
  delay(3000);
  Serial.println("Full power forward!");
  ForwardMotorLeftMethod();
  ForwardMotorRightMethod();
  delay(3000);
  Serial.println("Full power reverse!");
  ReverseMotorLeftMethod();
  ReverseMotorRightMethod();
  delay(3000);
  Serial.println("Turn to the left!");
  ForwardMotorLeftMethod();
  ReverseMotorRightMethod();
  delay(3000);
  Serial.println("Turn to the right!");
  ReverseMotorLeftMethod();
  ForwardMotorRightMethod();
  delay(3000);
}

void ForwardMotorRightMethod()
{
  digitalWrite(MotorOnPinRight, LOW);
  delay(50);
  digitalWrite(ReverseMotorRight, LOW);
  digitalWrite(ForwardMotorRight, HIGH);
  delay(50);
  digitalWrite(MotorOnPinRight, HIGH);
}

void ReverseMotorRightMethod()
{
  digitalWrite(MotorOnPinRight, LOW);
  delay(50);
  digitalWrite(ForwardMotorRight, LOW);
  digitalWrite(ReverseMotorRight, HIGH);
  delay(50);
  digitalWrite(MotorOnPinRight, HIGH);
}

void ForwardMotorLeftMethod()
{
  digitalWrite(MotorOnPinLeft, LOW);
  delay(50);
  digitalWrite(ReverseMotorLeft, LOW);
  digitalWrite(ForwardMotorLeft, HIGH);
  delay(50);
  digitalWrite(MotorOnPinLeft, HIGH);
}

void ReverseMotorLeftMethod()
{
  digitalWrite(MotorOnPinLeft, LOW);
  delay(50);
  digitalWrite(ForwardMotorLeft, LOW);
  digitalWrite(ReverseMotorLeft, HIGH);
  delay(50);
  digitalWrite(MotorOnPinLeft, HIGH);
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





