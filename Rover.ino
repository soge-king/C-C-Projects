// Define pin numbers for sensors and actuators
#include <Servo.h>
#include <SoftwareSerial.h> // do i even need this ? im too fucking tired to care
#include <Adafruit_TCS230.h>
#include Ultrasonic.h

Ultrasonic ultrasonic(,);
const int csoS0 = 4; // Color sensor output frequency scaling
const int csoS1 = 3;
const int csoS2 = 8;
const int csoS3 = 12;
const int csfS0 = 23;
const int csfS1 = 24;
const int csfS2 = 25;
const int csfS3 = 26;
const int csoOut = 22; // Color sensor output pin for object detection
const int csfOut = 27; // Color sensor output pin for floor detection

Adafruit_TCS230 csf(csfS0, csfS1, csfS2, csfS3, csfOut);
Adafruit_TCS230 cso(csoS0, csoS1, csoS2, csoS3, csoOut);
Servo gripper;
SoftwareSerial irReceiver(irPin, -1); // -1 is used for an unused receive pin  (???? no se si es necesario)
//IR reciever
const int irReceiverPin = 2; //fucking hope it works, jesus h christ
//Motor Pins
const int IN1 = 5;
const int IN2 = 6;
const int IN3 = 9;
const int IN4 = 10;
//Servo Pin
const int servoPin = 3;
//Echo sensor Pin
const int echoPin = 7;
const int trigPin = 8;
//RBG LED pins
const int redPin = 11;
const int greenPin = 12;
const int bluePin = 13;
// Define variables
int floorSensorValue = 0;
int objectSensorValue = 0;
int irValue = 0;
int distance = 0;
//RBG LED
int redValue = 0;
int greenValue = 0;
int blueValue = 0;
// Set up the gripper servo
Servo gripper;

void setup() {
  //Color sensor setup (object)
  pinMode(csoS0, INPUT);
  pinMode(csoS1, INPUT);
  pinMode(csoS2, INPUT);
  pinMode(csoS3, INPUT);
  pinMode(csoOut, INPUT);
  //color sensor setup (floor)
  pinMode(csfS0, INPUT);
  pinMode(csfS1, INPUT);
  pinMode(csfS2, INPUT);
  pinMode(csfS3, INPUT);
  pinMode(csfOut, INPUT);
  //Servo Setup
  pinMode(servoPin, OUTPUT);
  pinMode(irReceiverPin, INPUT);
  //echo pin setup
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
  //RGB LED output 
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  // Set the motor pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Set the servo pin as an output
  gripper.attach(servoPin);

  // Set up the IR receiver
  pinMode(irReceiverPin, INPUT);

  // Set up the echo sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  //map color scheme
  redValue = map(red, 0, 1000, 0, 255);
  greenValue = map(green, 0, 1000, 0, 255);
  blueValue = map(blue, 0, 1000, 0, 255);

  // Set the baud rate for serial communication
  Serial.begin(9600);
}

void loop() {
  //read color sensor values
  digitalWrite(csoS2Pin, LOW);
  digitalWrite(csoS3Pin, LOW);
  int red = pulseIn(csOut, LOW);

  digitalWrite(csoS2Pin, HIGH);
  digitalWrite(csoS3Pin, HIGH);
  int blue = pulseIn(csoOut, LOW);

  digitalWrite(csoS2Pin, LOW);
  digitalWrite(csoS3Pin, HIGH);
  int green = pulseIn(csoOut, LOW);

  //fuuuuuuuck tengo que hacerlo para el csf (color sensor floor).......i wanna die
  // Map sensor values
  redValue = map(red, 0, 1000, 0, 255);
  greenValue = map(green, 0, 1000, 0, 255);
  blueValue = map(blue, 0, 1000, 0, 255);

  // Read distance from echo sensor
 distance = ultrasonic.read();
  // Set the gripper to an initial open position
  gripper.write(0);

  // If there is no object within 20cm, spin
  if (distance > 20) {
    SPIN(200);
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin, HIGH);
  }
  // If there is an object within 20cm, move forward
  else {
    FORWARD(200);
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, HIGH);
    digitalWrite(bluePin, LOW);
    // If the object is within 1cm, close the gripper
    if (distance < 1) {

  gripper.write(0);
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, HIGH);
  unsigned long startTime = millis(); // record the current time
  unsigned long duration = 1000; // the duration of the action in milliseconds, set to 1 sec, useful for later adjustment if needed
      while (millis() - startTime < (duration*3)) {//Duration of action, set at 3s 
  analogWrite(redPin, redValue);
  analogWrite(greenPin, greenValue);
  analogWrite(bluePin, blueValue);
      }

  
  // Wait for 3 seconds with LED on before starting again

  delay(3000);
      // Spin until IR signal is detected
      SPIN(200);
      digitalWrite(redPin, LOW);
      digitalWrite(greenPin, LOW);
      digitalWrite(bluePin, HIGH);
      while (irValue == 0) {
        irValue = digitalRead(irReceiverPin);
      }
      // Move forward until blue line is detected
      FORWARD(200);
      digitalWrite(greenPin, LOW);
      digitalWrite(bluePin, HIGH);
      while (floorSensorValue < 1000) {//editar esto
        floorSensorValue = analogRead(floorSensorPin);
      }
      // Drop the object, move back, and spin
      gripper.write(0);
      BACK(200);
      digitalWrite(redPin, LOW);
      digitalWrite(greenPin, LOW);
      digitalWrite(bluePin, HIGH);
      delay(1000);
      analogWrite(motor1Pin1, 255);
      analogWrite(motor1Pin2, 0);
      analogWrite(motor2Pin1, 0);
      analogWrite(motor2Pin2, 255);
      // Wait for 3 seconds before starting again
      delay(3000);
    }
  }
  // If the floor sensor detects blue without an object and IR signal detected, go back and continue spinning
  else if (floorSensorValue > 1000 && objectSensorValue < 100 && irValue == 0) {//no se que hice aqui.....editar esto
    analogWrite(motor1Pin1, 0);
    analogWrite(motor1Pin2, 255);
    analogWrite(motor2Pin1, 0);
    analogWrite(motor2Pin2, 255);
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin, HIGH);
  }
}

//Case states for motors
void FORWARD (int Speed){

  analogWrite(IN1,Speed);
  analogWrite(IN2,0);
  analogWrite(IN3,0);
  analogWrite(IN4,Speed);
}
void BACK (int Speed){

  analogWrite(IN1,0);
  analogWrite(IN2,Speed);
  analogWrite(IN3,Speed);
  analogWrite(IN4,0);
}
void SPIN (int Speed){

  analogWrite(IN1,Speed);
  analogWrite(IN2,0);
  analogWrite(IN3,Speed);
  analogWrite(IN4,0);
}
void STOP (int Speed){

  analogWrite(IN1,0);
  analogWrite(IN2,0);
  analogWrite(IN3,0);
  analogWrite(IN4,0);
}

//does it work, idk.... i need a drink

