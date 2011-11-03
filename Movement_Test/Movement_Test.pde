#include <PololuQTRSensors.h>

#define oneSecond  1000        //One Second in milliseconds
#define leftWheelDirection 8
#define leftWheelSpeed 9
#define rightWheelDirection 11
#define rightWheelSpeed 10

#define ledPin  13
#define irSensorPin 3
#define rightLineSensorAPin 4
#define leftLineSensorAPin 5
#define leftLineSensorPin 18     //Corresponds to analog pin 4
#define rightLineSensorPin 19     //corresponds to analog pin 5

int forward = HIGH;
int backward = LOW;
int maxSpeed = 255;
int currentMotorSpeed = 0;
int lastKnownDistance = 1024;
int currentDirection = backward;
boolean leftOutOfBounds = false;
boolean rightOutOfBounds = false;

PololuQTRSensorsRC qtr((unsigned char[]) {
  leftLineSensorPin, rightLineSensorPin}
, 2, 2000, 255); //declares two line sensors on pins 18 and 19 this corresponds to analog pins 4 and 5
unsigned int sensors[2];
const int boundaryThreshold = 300; 
//IR Sensor Info:
//    farther--------------------------------closer
//          45   100  200  300  400  500  650
const int somethingAheadThreshold = 200;

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
  pinMode(leftLineSensorAPin, INPUT);
  pinMode(rightLineSensorAPin, INPUT);
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
    if(currentDirection == forward && lastKnownDistance > somethingAheadThreshold){
      attack();
    }else{
    
    checkIfOutOfBounds();
    
    if(leftOutOfBounds || rightOutOfBounds){
      //if(leftOutOfBounds && rightOutOfBounds){
        Serial.println("We're Headed Out of Bounds! Backup and Turn Around!!!");
        moveBackward(maxSpeed);
        delay(oneSecond/2);
        turnLeft(maxSpeed);
        delay(oneSecond/3);
      }/*else if(leftOutOfBounds){
        Serial.println("We're too close to the border on our Left!! Turn Right!");
        moveBackward(maxSpeed);
        delay(oneSecond/8);        
        turnRight(maxSpeed);
        delay(oneSecond/3);
        stopMotors();
      }else if(rightOutOfBounds){
        Serial.println("We're too close to the border on our Right!! Turn Left!");
        moveBackward(maxSpeed);
        delay(oneSecond/8);        
        turnLeft(maxSpeed);
        delay(oneSecond/3);
        stopMotors();
      }*/
      
      //checkIfOutOfBounds();
    //}
    
    //Serial.print("LastKnownDistance:");
    //Serial.println(lastKnownDistance);
    if(lastKnownDistance > somethingAheadThreshold){
      //if(currentDirection != backward){
        //Serial.println("Too Close!! Run Away!");
        //turnLeft(maxSpeed/2);
        //delay(oneSecond/4);
        //moveBackward(maxSpeed);
        
        attack();        
      //}
    }else if(lastKnownDistance < somethingAheadThreshold){
      //if(currentDirection != forward){
        //Serial.println("Nothing Ahead!! Moving Forward!!");
        //moveForward(maxSpeed);
        Serial.print("LastKnownDistance:");
        Serial.println(lastKnownDistance);
        moveBackward(maxSpeed);
        delay(oneSecond/8);
        stopMotors();
        scanForEnemy();       
      //}
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

void checkIfOutOfBounds(){
  sensors[0] = analogRead(leftLineSensorAPin);
  sensors[1] = analogRead(rightLineSensorAPin);
  Serial.print("Left Line Sensor Reading: ");
  Serial.println(sensors[0]);
  Serial.print("Right Line Sensor Reading: ");
  Serial.println(sensors[1]);
  leftOutOfBounds = sensors[0] > boundaryThreshold;
  rightOutOfBounds = sensors[1] > boundaryThreshold;
}
    
void scanForEnemy(){  
  lastKnownDistance = readDistance(10);
  boolean isLeft = false;
  int scanCounter = 0;
  while(++scanCounter < 20 && lastKnownDistance < somethingAheadThreshold){  
    if(scanCounter % 5 == 0){
      isLeft = !isLeft;
    }
    if(isLeft){
      turnLeft(maxSpeed/2);
    }else{
      turnRight(maxSpeed/2);
    }
    stopMotors();
    delay(oneSecond/8);
    lastKnownDistance = readDistance(10);
  }  
}

void attack(){
  Serial.print("Attacking - LastKnownDistance:");
  Serial.println(lastKnownDistance);        
  if(currentDirection != forward){
    Serial.println("EnemyFound!! Full Steam Ahead!!");
    moveForward(maxSpeed);
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
   currentDirection = NULL;
}

void turnRight(int targetSpeed){
   Serial.println("Turning Right");
   setMotorSpeed(targetSpeed);
   move(forward, backward);
   currentDirection = NULL; 
}

void stopMotors(){
   setMotorSpeed(0);
   currentDirection = NULL;
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
    //Serial.print("Reading: ");
    //Serial.println(reading);
    //Serial.print("Total: ");
    //Serial.println(total);
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
