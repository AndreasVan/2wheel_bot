
// Control of a 2wheels robot - Last update: AndreasVan 2014-05-01 Vers. 2.3
// Selfmade Chassis + L293D IC H-brighde 
// Micro controller = Arduino UNO
// Detecting obstacles with an SR04 ultrasonic sensor mounted on servo
// this code is public domain, enjoy!
 
#include <NewPing.h>        //library SR04 ultrasonic sensor
#include <Servo.h>          //library Servo (SR04 on servo)

#define TRIGGER_PIN 2       // SR04 sensor  
#define ECHO_PIN    3       // SR04 sensor
#define MAX_DISTANCE 200    //sensor distance
Servo myservo;  
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

#define runEvery(t) for (static typeof(t) _lasttime;(typeof(t))((typeof(t))millis() - _lasttime) > (t);_lasttime += (t))

const int PWN1 = 6;         //right PWN
const int DIR1 = 7;         //right DIR
const int PWN2 =5;         //left PWN
const int DIR2 =4;         //left DIR
const int redLed = 10;         //robot drives forward
const int greenLed = 11;       //robot drives backward
const int yellowLed = 12;      //robot turns left
const int blueLed = 13;        //robot turns right

int uS;                     //value of SR04 ultrasonic sensor
int distance;               //distance in cm of ultrasonic sensor 
int pos = 90;               //start position of servo = 90
int servoDirection = 0;     //sweeping left or right
int MoveDirection = 0;      //0 = forward, 1 = backward, 2 = left, 3 = right
int lastMoveDirection;      //last direction of the robot
int distanceCenter;     
int distanceLeft;
int distanceRight;
int servoDelay = 55;        //servo sweep speed
const int speedLeft = 150;  //motor speed left
const int speedRight = 150; //motor speed right
long previousMillis = 0;
const int interval = 1000;  // 1000 time to switch between the MoveDirections, turntime right/leeft when it detects an obstacle

void setup() {                
  pinMode(PWN1, OUTPUT);  
  pinMode(DIR1, OUTPUT); 
  pinMode(PWN2, OUTPUT); 
  pinMode(DIR2, OUTPUT);
  analogWrite(PWN1, 0);
  analogWrite(PWN2, 0);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(redLed, OUTPUT); 
  pinMode(greenLed, OUTPUT);
  pinMode(yellowLed, OUTPUT);
  pinMode(blueLed, OUTPUT);
  Serial.begin(9600);       //to use the serial monitor
  myservo.attach(8);       //servo on pin 10
  myservo.write(pos);       //center servo
  
  delay(1000);              // start delay, some time to put the robot on the floor :-)
}

void loop() {

  sweepServo();             //servo sweep function
  
  getDistance();            //ultrasonic sensor distance 
  
    if (pos >= 15 && pos <= 45)
  {
    distanceRight = distance;  //measured distance servo is to the right = distanceRight
  }
  if (pos >= 135 && pos <= 165)
  {
    distanceLeft = distance;   //measured distance servo is to the left = distanceLeft
  }   
  if (pos > 70 && pos < 110)
  {
    distanceCenter = distance; //measured distance servo in centred  = distanceCenter
  } 

  if (distanceCenter >= 25)    // coast is clear, full power forward
  {
    MoveDirection = 0;         //move forward
  }
  else                         //obstacle detected, turn left or right?
  {
    if (distanceLeft > distanceRight)  
    {
      MoveDirection = 2;      //turn left = 2
    }
    if (distanceLeft < distanceRight)
    {
      MoveDirection = 3;      //turn right = 3
    }
    if (distanceLeft <= 5 && distanceCenter <= 5 || distanceRight <= 5 && distanceCenter <= 5)
    {
      MoveDirection = 1;      //  turn back
    }
  }

  unsigned long currentMillis = millis();  //set a timer

  if(MoveDirection == 0 && MoveDirection == lastMoveDirection)  
  {
    forward();
    lastMoveDirection = MoveDirection;
  }
  if(MoveDirection == 0 && MoveDirection != lastMoveDirection && currentMillis - previousMillis > interval )
  {  
    forward();
    lastMoveDirection = MoveDirection;
    previousMillis = currentMillis;
  }
  if(MoveDirection == 1 && MoveDirection == lastMoveDirection)
  {
    backward();
    lastMoveDirection = MoveDirection;
  }
  if(MoveDirection == 1 && MoveDirection != lastMoveDirection && currentMillis - previousMillis > interval )
  {  
    backward();
    lastMoveDirection = MoveDirection;
    previousMillis = currentMillis;
  }
  if(MoveDirection == 2 && MoveDirection == lastMoveDirection)
  {
    left();
    lastMoveDirection = MoveDirection;
  }
  if(MoveDirection == 2 && MoveDirection != lastMoveDirection && currentMillis - previousMillis > interval )
  {  
    left();
    lastMoveDirection = MoveDirection;
    previousMillis = currentMillis;
  }
  if(MoveDirection == 3 && MoveDirection == lastMoveDirection)
  {
    right();
    lastMoveDirection = MoveDirection;
  }
  if(MoveDirection == 3 && MoveDirection != lastMoveDirection && currentMillis - previousMillis > interval )
  {  
    right();
    lastMoveDirection = MoveDirection;
    previousMillis = currentMillis;
  }
}

void forward()
{
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);   
  analogWrite(PWN1, speedLeft);
  analogWrite(PWN2, speedRight);
  digitalWrite(redLed, HIGH);
  digitalWrite(greenLed, LOW);
  digitalWrite(yellowLed, LOW);
  digitalWrite(blueLed, LOW);
}

void stop()
{
  digitalWrite(DIR1, LOW);   
  digitalWrite(DIR2, LOW);   
  analogWrite(PWN1, 0);
  analogWrite(PWN2, 0);
  digitalWrite(redLed, LOW);
  digitalWrite(greenLed, LOW);
  digitalWrite(yellowLed, LOW);
  digitalWrite(blueLed, LOW);
}

void backward()
{
  digitalWrite(DIR1, HIGH);  
  digitalWrite(DIR2, HIGH);   
  analogWrite(PWN1, speedLeft-10); 
  analogWrite(PWN2, speedRight-10); 
  digitalWrite(redLed, LOW);
  digitalWrite(greenLed, HIGH);
  digitalWrite(yellowLed, LOW);
  digitalWrite(blueLed, LOW);
}

void left()
{
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, HIGH);   
  analogWrite(PWN1, speedLeft-20); 
  analogWrite(PWN2, speedRight+20); 
  digitalWrite(redLed, LOW);
  digitalWrite(greenLed, LOW);
  digitalWrite(yellowLed, HIGH);
  digitalWrite(blueLed, LOW);
}

void right()
{
  digitalWrite(DIR1, HIGH);
  digitalWrite(DIR2, LOW);   
  analogWrite(PWN1, speedLeft+20);  
  analogWrite(PWN2, speedRight-20); 
  digitalWrite(redLed, LOW);
  digitalWrite(greenLed, LOW);
  digitalWrite(yellowLed, LOW);
  digitalWrite(blueLed, HIGH);
}

void getDistance()
{
  runEvery(20)                      //loop for ultrasonic measurement
  {
    uS = sonar.ping();
    distance = uS / US_ROUNDTRIP_CM;
    if (uS == NO_ECHO)              // if the sensor did not get a ping        
    {
      distance = MAX_DISTANCE;      //so the distance must be bigger then the max vaulue of the sensor
    }
    Serial.print("Ping: ");         //to check distance on the serial monitor
    Serial.print(distance); 
    Serial.println("cm");
  }
}

void sweepServo()
{
  runEvery(servoDelay)                      //this loop determines the servo position
  {
    if(pos < 165 && servoDirection == 0)    // 165 = servo to the left
    {                                  
      pos = pos + 5;                        
    } 
    if(pos > 15 && servoDirection == 1)     // 15 = servo to the right
    {                                
      pos = pos - 5;
    }    
  }   
  if (pos == 165 )    
  {
    servoDirection = 1;                     //changes direction
  }
  if (pos == 15 )     
  {
    servoDirection = 0;                     //changes direction
  }   
  myservo.write(pos);                       //move servo!
}
