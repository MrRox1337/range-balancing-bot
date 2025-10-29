// === Pin Definitions ===
#define trigPin 6
#define echoPin 5

#define enA 10
#define in1 9
#define in2 8
#define enB 3
#define in3 4
#define in4 2

// === PID Variables ===
// double Kp = 4.0-0.05-0.05-0.1-0.01-0.01-0.01;    // Proportional gain
double Kp = 10;    // Proportional gain
double Ki = 0.85;     // Integral gain
double Kd = 4.4;    // Derivative gain

double setPoint = 12.0;  // Target distance (cm)
double setpointVariance = 0.3; // Acceptable deviation (cm)
double Input;            // Measured distance
double Output;           // Motor speed (0–255)

double propError;
double lastPropError = 0;
double integralError = 0;
double diffError;
const double outputLimitMin = 105;
const double outputLimitMax = 255;

unsigned long currentTime;
unsigned long previousTime = 0;
const float SAMPLE_TIME_SEC = 0.1;

// === Function Declarations ===
float getDistance();
double computePID();
void moveForward(double speed);
void moveBackward(double speed);
void stopMotors();

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  previousTime = millis();
  Serial.println("PID Distance Controller Initialized");
}

void loop() {
  currentTime = millis();

  if ((currentTime - previousTime) >= (SAMPLE_TIME_SEC * 1000)) {
    Input = getDistance();        // Get current distance
    Output = computePID();        // Compute speed output (105–255)

    previousTime = currentTime;

    Serial.print("Dist: "); Serial.print(Input);
    Serial.print(" cm | Error: "); Serial.print(propError);
    Serial.print(" | PWM: "); Serial.print(Output);
    Serial.print(" | Motion: ");

    // --- Control logic based on distance ---
    if (Input < 1.0) { // No reading, safety system
      stopMotors();
    } 
    else if (Input > setPoint + setpointVariance) {
      moveForward(Output); // Move forward faster when far
    } 
    else if (Input < setPoint - setpointVariance) {
      moveBackward(Output); // Move backward gently when too close
    } 
    else {
      stopMotors(); // Stay still near target
    }
  }
}

// === Distance Function ===
float getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  float duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2.0;
}

// === PID Computation ===
double computePID() {
  double deltaTime = (double)(currentTime - previousTime) / 1000.0;

  propError = setPoint - Input;
  double pTerm = fabs(Kp * propError);

  integralError += propError * deltaTime;
  integralError = constrain(integralError, outputLimitMin, outputLimitMax);
  double iTerm = Ki * integralError;

  diffError = (propError - lastPropError) / deltaTime;
  double dTerm = Kd * diffError;

  double output = pTerm + iTerm + dTerm;
  output = constrain(output, outputLimitMin, outputLimitMax);

  lastPropError = propError;
  return output;
}

// === Motor Control ===
void moveForward(double speed) {
  analogWrite(enA, speed);
  analogWrite(enB, speed);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  Serial.println("Forward");
}

void moveBackward(double speed) {
  analogWrite(enA, speed);
  analogWrite(enB, speed);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  Serial.println("Forward");
}

void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);

  Serial.println("Stopped");
}
