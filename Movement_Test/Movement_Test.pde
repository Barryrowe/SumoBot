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
int lastKnownDistance = 1024;
int currentDirection = backward;

PololuQTRSensorsRC qtr((unsigned char[]) {
  19,18}
, 2, 2000, 255); //declares two line sensors on pins 18 and 19 this corresponds to analog pins 4 and 5
unsigned int sensors[2];
const int linethreshold = 300; 
const int tooCloseThreshold = 200;
const int tooFarThreshold = 500;

const int somethingAheadThreshold = 300;

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
  //blink(ledPin, 3, 100);
  // blink the LED 3 times. This should happen only once. 
  // if you see the LED blink three times, it means that 
  // the module reset itself,. probably because the motor 
  // caused a brownout or a short. 
} //end setup

void loop(){
    Serial.println("StartLoop!");
    
    lastKnownDistance = readDistance(10);
    Serial.print("LastKnownDistance:");
    Serial.println(lastKnownDistance);
    
    if(lastKnownDistance < somethingAheadThreshold){
      if(currentDirection != forward){
        //Serial.println("Nothing Ahead!! Moving Forward!!");
        //moveForward(maxSpeed);
        scanForEnemy();
      }
    }else if(lastKnownDistance > somethingAheadThreshold){
      if(currentDirection != backward){
        //Serial.println("Too Close!! Run Away!");
        //turnLeft(maxSpeed/2);
        //delay(oneSecond/4);
        //moveBackward(maxSpeed);
        
        Serial.println("EnemyFound!! Full Steam Ahead!!");
        moveForward(maxSpeed);
      }
    }
    
    /*
    //Test Move Forward
    moveForward(maxSpeed);
    delay(5*oneSecond);
    stopMotors();
    delay(oneSecond/2);
    
    //Test Turn Left
    turnLeft(maxSpeed);
    delay(oneSecond);
    stopMotors();
    delay(oneSecond);
    //Test Move Backward
    moveBackward(maxSpeed);
    delay(5*oneSecond);
    stopMotors();
    delay(oneSecond/2);

    //Test Turn Right
    turnRight(maxSpeed);
    delay(oneSecond);
    stopMotors();
    Serial.println("End Loop!");*/
}

void scanForEnemy(){  
  lastKnownDistance = readDistance(10);
  while(lastKnownDistance < somethingAheadThreshold){  
    turnRight(maxSpeed/2);
    lastKnownDistance = readDistance(10);
  }  
}

//ROBOT MOVEMENT API
//CALL THROUGH THESE FUNCTIONS
void moveForward(int targetSpeed){
    setMotorSpeed(targetSpeed);
    move(forward, forward);
    currentDirection = forward;
}

void moveBackward(int targetSpeed){
    setMotorSpeed(targetSpeed);  
    move(backward, backward);
    currentDirection = backward;
}

void turnLeft(int targetSpeed){
   Serial.println("Turning Left");
   setMotorSpeed(targetSpeed);
   move(backward, forward); 
}

void turnRight(int targetSpeed){
   Serial.println("Turning Right");
   setMotorSpeed(targetSpeed);
   move(forward, backward); 
}

void stopMotors(){
   setMotorSpeed(0);
   Serial.println("Stopped Motors!");
}

//SENSOR 'API'
int readDistance(int sampleSize){
  if(sampleSize == NULL){
    sampleSize = 10;
  }
  float total = 0;
  for(int i=0;i<sampleSize;i++){
    int reading = analogRead(irSensorPin);
    total += reading;
    Serial.print("Reading: ");
    Serial.println(reading);
    Serial.print("Total: ");
    Serial.println(total);
  }
  return total/sampleSize;  
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

void move(int leftDirection, int rightDirection){
  digitalWrite(leftWheelDirection, leftDirection);
  digitalWrite(rightWheelDirection, rightDirection); 
  Serial.print("Moving with Left Wheel: ");
  Serial.print(leftDirection);
  Serial.print(" and Right Wheel: ");
  Serial.println(rightDirection);
}
/*
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
}// end blink*/
