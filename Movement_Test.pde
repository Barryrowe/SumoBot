#include <PololuQTRSensors.h>

#define oneSecond  1000        //One Second in milliseconds
#define leftWheelDirection 8
#define leftWheelSpeed 9
#define rightWheelDirection 11
#define rightWheelSpeed 10

#define ledPin  13
#define irSensorPin 3
#define lefttLineSensor 18     //Corresponds to analog pin 4
#define rightLineSensor 19     //corresponds to analog pin 5

int forward = HIGH;
int backward = LOW;
int maxSpeed = 255;
int currentMotorSpeed = 0;

PololuQTRSensorsRC qtr((unsigned char[]) {
  19,18}
, 2, 2000, 255); //declares two line sensors on pins 18 and 19 this corresponds to analog pins 4 and 5
unsigned int sensors[2];
const int linethreshold = 300; 

void setup() { // put your setup code here, to run once:
  //motor control outputs
  pinMode(leftWheelDirection, OUTPUT);
  pinMode(leftWheelSpeed, OUTPUT);
  pinMode(rightWheelDirection, OUTPUT);
  pinMode(rightWheelSpeed, OUTPUT);
  // AV outputs
  pinMode(ledPin, OUTPUT);
  //pinMode(buzzerPin, OUTPUT);
  pinMode(irSensorPin, INPUT);
  Serial.begin(9600); // set up Serial library at 9600 bps for debugging
  delay (3000); //wait for everything and for the match to start  
  blink(ledPin, 3, 100);
  // blink the LED 3 times. This should happen only once. 
  // if you see the LED blink three times, it means that 
  // the module reset itself,. probably because the motor 
  // caused a brownout or a short. 
} //end setup

void loop(){
    Serial.println("StartLoop!");
    
    //Test Move Forward
    moveForward(maxSpeed);
    delay(5*oneSecond);
    stopMotors();
    delay(oneSecond/2);
    
    //Test Turn Left
    turnLeft(maxSpeed/2);
    delay(2*oneSecond);
    stopMotors();

    //Test Move Backward
    moveBackward(maxSpeed);
    delay(5*oneSecond);
    stopMotors();
    delay(oneSecond/2);

    //Test Turn Right
    turnRight(maxSpeed/2);
    delay(3*oneSecond);
    stopMotors();
    Serial.println("End Loop!");
}

//ROBOT MOVEMENT API
//CALL THROUGH THESE FUNCTIONS
void moveForward(int targetSpeed){
    setMotorSpeed(targetSpeed);
    move(forward, forward);
}

void moveBackward(int targetSpeed){
    setMotorSpeed(targetSpeed);  
    move(backward, backward);
}

void turnLeft(int targetSpeed){
   setMotorSPeed(targetSpeed);
   move(backward, forward); 
}

void turnRight(int targetSpeed){
   setMotorSpeed(targetSpeed);
   move(forward, backward); 
}

void stopMotors(){
   setMotorSpeed(0);
   Serial.println("Stopped Motors!");
}

//UTILITY FUNCTIONS TO BE CALLED ONLY BY OUR MOVEMENT API
void setMotorSpeed(int targetSpeed){
  Serial.println("Set Motor Speed");
  Serial.print("Current Speed: ");
  Serial.println(currentMotorSpeed);
  int spd = targetSpeed;
  if(spd > 255){ spd = spd % 255; }
  if(currentMotorSpeed != spd){
    analogWrite(leftWheelSpeed, spd);
    analogWrite(rightWheelSpeed, spd);
    currentMotorSpeed = spd;
    Serial.print("Current Speed: ");
    Serial.println(currentMotorSpeed);
  }
}

void move(int leftWheelDirection, int rightWheelDirection){
  digitalWrite(leftWheelDirection, leftWheelDirection);
  digitalWrite(rightWheelDirection, rightWheelDirection); 
  Serial.print("Moving with Left Wheel: ");
  Serial.print(leftWheelDirection);
  Serial.print(" and Right Wheel: ");
  Serial.println(rightWheelDirection);
}
//This function blinks an LED
void blink(int whatPin, int howManyTimes, int milliSecs) {
  int i = 0;
  for ( i = 0; i < howManyTimes; i++) {
    digitalWrite(whatPin, HIGH);
    delay(milliSecs/2);
    digitalWrite(whatPin, LOW);
    delay(milliSecs/2);
    Serial.println("I blinked!");
  }
}// end blink
