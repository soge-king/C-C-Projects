#include <Ultrasonic.h>
#include <Servo.h>
#include <MD_TCS230.h>
#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define sensorOut 8

Ultrasonic ultrasonic(9,10);
Servo gripperServo;


int redPW = 0;
int greenPW = 0;
int bluePW = 0;

const int IN1=5;
const int IN2=6;
const int IN3=9;
const int IN4=10;
const int SERVO_PIN = 11;  // Servo connected to pin 11
const int LED_PIN = 12;  // LED connected to pin 12
unsigned long previousMillis = 0;
const long interval = 2000;  // Interval set for 2 seconds
bool slowMode = false;  // To keep track if we are in slow mode

void setup() 
{
  Serial.begin(9600);
  gripperServo.attach(SERVO_PIN);
  pinMode(LED_PIN, OUTPUT);
	pinMode(S0, OUTPUT);
	pinMode(S1, OUTPUT);
	pinMode(S2, OUTPUT);
	pinMode(S3, OUTPUT);
  digitalWrite(S0,HIGH);
	digitalWrite(S1,LOW);
  pinMode(sensorOut, INPUT);

	// Setup Serial Monitor
	Serial.begin(9600);
  delay(5000);
}


 
void loop() {
  // Read the sensor
 

	redPW = getRedPW();
	// Delay to stabilize sensor
	delay(200);

	// Read Green Pulse Width
	greenPW = getGreenPW();
	// Delay to stabilize sensor

	bluePW = getBluePW();
	// Delay to stabilize sensor
	delay(200);

	// Print output to Serial Monitor
	Serial.print("Red PW = ");
	Serial.print(redPW);
	Serial.print(" - Green PW = ");
	Serial.print(greenPW);
	Serial.print(" - Blue PW = ");
	Serial.println(bluePW);

// Function to read Red Pulse Widths
int getRedPW() {
	// Set sensor to read Red only
	digitalWrite(S2,LOW);
	digitalWrite(S3,LOW);
	// Define integer to represent Pulse Width
	int PW;
	// Read the output Pulse Width
	PW = pulseIn(sensorOut, LOW);
	// Return the value
	return PW;
}

// Function to read Green Pulse Widths
int getGreenPW() {
	// Set sensor to read Green only
	digitalWrite(S2,HIGH);
	digitalWrite(S3,HIGH);
	// Define integer to represent Pulse Width
	int PW;
	// Read the output Pulse Width
	PW = pulseIn(sensorOut, LOW);
	// Return the value
	return PW;
}

// Function to read Blue Pulse Widths
int getBluePW() {
	// Set sensor to read Blue only
	digitalWrite(S2,LOW);
	digitalWrite(S3,HIGH);
	// Define integer to represent Pulse Width
	int PW;
	// Read the output Pulse Width
	PW = pulseIn(sensorOut, LOW);
	// Return the value
	return PW;
}


      // Color-based actions
      if (bluePW > greenPW && bluePW > redPW) { //azul
        digitalWrite(LED_PIN, HIGH);
      } else if (bluePW > greenPW && greenPW > redPW) { //amarillo
        slowMode = true;
        previousMillis = millis();
      } else if (redPW > bluePW && redPW > greenPW) { //rojo
        delay(5000);
      } else if (redPW > 220 && bluePW > 220 && greenPW > 220) { //negro
        gripperServo.write(-90);
        STOP();
      }
  

  // Slow mode handling
  if (slowMode) {
    if (millis() - previousMillis >= interval) {
      slowMode = false;
    } else {
      FORWARD(100);
      return;
    }
  }

  // Ultrasonic sensor handling
  int distance = ultrasonic.read();

  if (distance < 20) {
    Stop();
    gripperServo.write(-90);
    while (distance < 20) {
      FORWARD(150); 
      distance = ultrasonic.read();
      
      if (distance <= 1) {
        Stop();
        gripperServo.write(90);
        break;
      }
      delay(10); 
    }
  } else {
    ROTATE(150);
  }
}

void FORWARD (int Speed) {
  analogWrite(IN1,Speed);
  analogWrite(IN2,0);
  analogWrite(IN3,0);
  analogWrite(IN4,Speed);
}

void BACKWARD (int Speed){
  analogWrite(IN1,0);
  analogWrite(IN2,Speed);
  analogWrite(IN3,Speed);
  analogWrite(IN4,0);
}

void ROTATE (int Speed) {
  analogWrite(IN1,Speed);
  analogWrite(IN2,0);
  analogWrite(IN3,Speed);
  analogWrite(IN4,0);
}

void STOP() {
  analogWrite(IN1,0);
  analogWrite(IN2,0);
  analogWrite(IN3,0);
  analogWrite(IN4,0);
}
