#include <Servo.h>
#include <SoftwareSerial.h>

#define SERVO_PIN 9
#define MOTOR_ENA 10
#define MOTOR_ENB 11
#define MOTOR_PWM 6
#define TRIG_PIN 2
#define ECHO_PIN 3

#define FORWARD 'F'
#define BACKWARD 'B'
#define LEFT 'L'
#define RIGHT 'R'
#define STOP 'S'

// Define the gesture sensor pins
// ...

Servo servo;
SoftwareSerial btSerial(4, 5); // RX, TX

void setup() {
  servo.attach(SERVO_PIN);

  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_ENB, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Setup the gesture sensor pins
  // ...

  btSerial.begin(9600);

  servo.write(90);
  delay(500);
}

void loop() {
  // Read the ultrasonic sensor
  long duration, distance;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration / 29 / 2;

  // Read the gesture sensor
  // ...

  // Map the sensor data to a servo angle
  int servoAngle = map(gestureData, minSensorValue, maxSensorValue, 0, 180);
  servo.write(servoAngle);

  // Use the ultrasonic and gesture data to control the car
  if (distance > 10) {
    if (gestureData == FORWARD) {
      digitalWrite(MOTOR_ENA, HIGH);
      digitalWrite(MOTOR_ENB, LOW);
      analogWrite(MOTOR_PWM, 255);
    } else if (gestureData == BACKWARD) {
      digitalWrite(MOTOR_ENA, LOW);
      digitalWrite(MOTOR_ENB, HIGH);
      analogWrite(MOTOR_PWM, 255);
    } else if (gestureData == LEFT) {
      digitalWrite(MOTOR_ENA, HIGH);
      digitalWrite(MOTOR_ENB, LOW);
      analogWrite(MOTOR_PWM, 128);
    } else if (gestureData == RIGHT) {
      digitalWrite(MOTOR_ENA, LOW);
      digitalWrite(MOTOR_ENB, HIGH);
      analogWrite(MOTOR_PWM, 128);
    } else if (gestureData == STOP) {
      digitalWrite(MOTOR_ENA, LOW);
      digitalWrite(MOTOR_ENB, LOW);
      analogWrite(MOTOR_PWM, 0);
    }
  } else {
    digitalWrite(MOTOR_ENA, LOW);
    digitalWrite(MOTOR_ENB, LOW);
    analogWrite(MOTOR_PWM, 0);
  }

  // Send data to the Bluetooth module
  btSerial.print(distance);
  btSerial.print(",");
  btSerial.println(gestureData);
}
