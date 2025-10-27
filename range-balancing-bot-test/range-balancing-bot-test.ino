// Ultrasonic sensor pins
#define trigPin 6
#define echoPin 5

// Motor driver pins
#define enA 10
#define in1 9
#define in2 8
#define enB 3
#define in3 4
#define in4 2

// Motor speed (0–255)
int motorSpeed = 200;

float duration;
float distance;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  distance = getDistance();

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" cm\t");
  Serial.print("Direction: ");

  if (distance < 1.0) {
    stopMotors();
    Serial.println("Error: Distance sensor not responding.");
  } else {

    if (distance > 17.0) {
      forward();
    } else if(distance >= 13.0 && distance <= 17.0) {
      stopMotors();
    } else {
      backward();
    }
  }
}

// ---- Distance Function ----
float getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  return duration * 0.034 / 2.0; // in cm
}

// ---- Motor Control Functions ----
void forward() {
  Serial.println("Forward");

  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void backward() {
  Serial.println("Backward");

  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void stopMotors() {
  Serial.println("Stopped");
  
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}
