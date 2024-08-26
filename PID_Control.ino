#include <SimplyAtomic.h>
#include <NewPing.h>

/*********** ULTRASONIC  ***********/

const int LEFT_ECHO_PIN = 8;
const int LEFT_TRIG_PIN = 9;

const int RIGHT_ECHO_PIN = 10;
const int RIGHT_TRIG_PIN = 11;

const int FRONT_ECHO_PIN = 12;

const int FRONT_TRIG_PIN = 13;

NewPing sonarR(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN, 300);
NewPing sonarL(LEFT_TRIG_PIN, LEFT_ECHO_PIN, 300);
NewPing sonarF(FRONT_TRIG_PIN, FRONT_ECHO_PIN, 300);

// static bool left_turning = false;
// static bool right_turning = false;

int encoder_count[] = {0, 0};

unsigned long startTime;
bool running = false;

long durationL;
int distanceL;

long durationR;
int distanceR;

long durationF;
int distanceF;

// A class to compute the control signal
class SimplePID
{
private:
  float kp, kd, ki, umax; // Parameters
  float eprev, eintegral; // Storage

public:
  // Constructor
  SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0) {}

  // A function to set the parameters
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn)
  {
    kp = kpIn;
    kd = kdIn;
    ki = kiIn;
    umax = umaxIn;
  }

  float sign(float n)
  {
    if (n > 0)
      return 1;
    else if (n < 0)
      return -1;
    else
      return 0;
  }

  // A function to compute the control signal
  void evalu(float speed, float target, float deltaT, int &pwr, int &dir)
  {
    // error
    float e = target - speed;

    // derivative
    float dedt = (e - eprev) / (deltaT);

    // integral
    eintegral = eintegral + e * deltaT;

    // control signal
    float u = kp * e + kd * sign(dedt) / (fabs(dedt) + 0.1) + ki * eintegral;

    // motor power
    pwr = (int)fabs(u);
    if (pwr > umax)
    {
      pwr = umax;
    }

    ////Serial.print("PWR correction input : ");
    ////Serial.println(pwr);

    // motor direction
    dir = 1;
    if (u < 0)
    {
      dir = -1;
    }

    ////Serial.print("direction : ");
    ////Serial.println(dir);

    // store previous error
    eprev = e;
  }
};

// How many motors
#define NMOTORS 2

// Pins
const int enca[] = {18, 20};
const int encb[] = {19, 21};
const int pwm[] = {6, 7};
const int in1[] = {2, 4};
const int in2[] = {3, 5};

// Globals
long prevT = 0;
// volatile int posi[] = {0,0};
volatile int duration[] = {0.0, 0.0};

// PID class instances
SimplePID pid[NMOTORS];

void setup()
{
  Serial.begin(9600);

  for (int k = 0; k < NMOTORS; k++)
  {
    pinMode(enca[k], INPUT);
    pinMode(encb[k], INPUT);
    pinMode(pwm[k], OUTPUT);
    pinMode(in1[k], OUTPUT);
    pinMode(in2[k], OUTPUT);

    pid[k].setParams(10, 100, 1, 255); // Example parameters, adjust as needed
  }

  attachInterrupt(digitalPinToInterrupt(enca[0]), wheelSpeed<0>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enca[1]), wheelSpeed<1>, CHANGE);

  attachInterrupt(digitalPinToInterrupt(enca[0]), onPulse<0>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enca[1]), onPulse<1>, CHANGE);

  /*****UltraS*****/

  pinMode(LEFT_TRIG_PIN, OUTPUT); // Sets the trigPin as an Output
  pinMode(LEFT_ECHO_PIN, INPUT);

  pinMode(RIGHT_TRIG_PIN, OUTPUT); // Sets the trigPin as an Output
  pinMode(RIGHT_ECHO_PIN, INPUT);

  pinMode(FRONT_TRIG_PIN, OUTPUT); // Sets the trigPin as an Output
  pinMode(FRONT_ECHO_PIN, INPUT);
}

void loop()
{

  digitalWrite(FRONT_TRIG_PIN, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(FRONT_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(FRONT_TRIG_PIN, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  durationF = pulseIn(FRONT_ECHO_PIN, HIGH);
  // Calculating the distance
  distanceF = durationR * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Front Distance: ");
  Serial.println(distanceF);

  if(distanceF > 10){
    forward();
  }   

  else {

    stopAll();
    digitalWrite(LEFT_TRIG_PIN, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(LEFT_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(LEFT_TRIG_PIN, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    durationL = pulseIn(LEFT_ECHO_PIN, HIGH);
    // Calculating the distance
    distanceL = durationL * 0.034 / 2;
    // Prints the distance on the Serial Monitor
    Serial.print("Left Distance: ");
    Serial.println(distanceL);

    digitalWrite(RIGHT_TRIG_PIN, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(RIGHT_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(RIGHT_TRIG_PIN, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    durationR = pulseIn(RIGHT_ECHO_PIN, HIGH);
    // Calculating the distance
    distanceR = durationR * 0.034 / 2;
    // Prints the distance on the Serial Monitor
    Serial.print("Right Distance: ");
    Serial.println(distanceR);

  }


  if(distanceL > 4){

    if (!running) {
      // Start the function and record the start time
      startTime = millis();
      running = true;
    }

    // Check if 5 seconds have passed
    if (millis() - startTime < 50000) {
      
      // Run the function for 5 seconds
      left_turn();
    }

  }

  if(distanceR > 4){

    if (!running) {
      // Start the function and record the start time
      startTime = millis();
      running = true;
    }

    // Check if 5 seconds have passed
    if (millis() - startTime < 50000) {
      
      // Run the function for 5 seconds
      right_turn();
    }

  }
}

void forward()
{
  

  attachInterrupt(digitalPinToInterrupt(enca[0]), wheelSpeed<0>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enca[1]), wheelSpeed<1>, CHANGE);


  while (distanceF > 9)
  {

    // Set target speed (e.g., in RPM)
    float targetSpeed[NMOTORS];
    targetSpeed[0] = 40; // target speed in terms of pulses
    targetSpeed[1] = 38; // Same target for both motors

    // Time difference
    long currT = micros();
    long deltaT = (currT - prevT);
    prevT = currT;

    // Read the velocity in an atomic block to avoid potential misreads
    float currentSpeed[NMOTORS];
    ATOMIC()
    {
      for (int k = 0; k < NMOTORS; k++)
      {
        currentSpeed[k] = -duration[k] * 1e6 / deltaT / 45.07;
        duration[k] = 0;
        ////Serial.println(currentSpeed[k]);
      }
    }

    // Loop through the motors
    for (int k = 0; k < NMOTORS; k++)
    {
      int pwr, dir;
      // Evaluate the control signal
      pid[k].evalu(currentSpeed[k], targetSpeed[k], deltaT / 1e6, pwr, dir);
      // Signal the motor
      setMotor(dir, pwr, pwm[k], in1[k], in2[k]);
    }
  }
}


void right_turn(){


  encoder_count[0] = 0;
  encoder_count[1] = 0;

  interrupts(); // Re-enable interrupts

  attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder<1>, RISING);

  // attachInterrupt(digitalPinToInterrupt(enca[0]), onPulse<0>, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(enca[1]), onPulse<1>, CHANGE);

  bool shouldExit = true;
  while(shouldExit){
    ATOMIC()
    {
      shouldExit = encoder_count[0] < 50 && encoder_count[1] > -50;
    }

    // Serial.print("first: ");
    // Serial.print(encoder_count[0]);
    // Serial.print("; second: ");
    // Serial.println(encoder_count[1]);


    float targetSpeed[NMOTORS];
    targetSpeed[0] = 10;
    targetSpeed[1] = -5;

    // Time difference
    long currT = micros();
    long deltaT = (currT - prevT);
    prevT = currT;

    float currentSpeed[NMOTORS];
    ATOMIC()
    {
      for (int k = 0; k < NMOTORS; k++)
      {
        currentSpeed[k] = -duration[k] * 1e6 / deltaT / 45.07;
        duration[k] = 0;
        ////Serial.println(currentSpeed[k]);
      }
    }

    // Loop through the motors
    for (int k = 0; k < NMOTORS; k++)
    {
      int pwr, dir;
      // Evaluate the control signal
      pid[k].evalu(currentSpeed[k], targetSpeed[k], deltaT / 1e6, pwr, dir);
      // Signal the motor
      setMotor(dir, pwr, pwm[k], in1[k], in2[k]);
    }
  }

  // Serial.print("s0:");
  // Serial.println(currentSpeed[0]);
  // Serial.print("s1:");
  // Serial.println(currentSpeed[1]);


  // Reset the encoder count and detach interrupt for motor 0
  noInterrupts(); // Temporarily disable interrupts
  // encoder_count[0] = 0;
  // encoder_count[1] = 0;
  detachInterrupt(digitalPinToInterrupt(enca[0]));
  detachInterrupt(digitalPinToInterrupt(enca[1]));
  // interrupts(); // Re-enable interrupts
}

void left_turn()
{

  encoder_count[0] = 0;
  encoder_count[1] = 0;

  interrupts(); // Re-enable interrupts

  attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder<1>, RISING);


  bool shouldExit = true;
  while (shouldExit)
  {
    ATOMIC()
    {
      shouldExit = encoder_count[0] > -50 && encoder_count[1] < 50;
    }

    Serial.print("first: ");
    Serial.print(encoder_count[0]);
    Serial.print("; second: ");
    Serial.println(encoder_count[1]);

    float targetSpeed[NMOTORS];
    targetSpeed[0] = -5;
    targetSpeed[1] = 10;

    // Time difference
    long currT = micros();
    long deltaT = (currT - prevT);
    prevT = currT;

    float currentSpeed[NMOTORS];
    ATOMIC()
    {
      for (int k = 0; k < NMOTORS; k++)
      {
        currentSpeed[k] = -duration[k] * 1e6 / deltaT / 45.07;
        duration[k] = 0;
        ////Serial.println(currentSpeed[k]);
      }
    }

    // Loop through the motors
    for (int k = 0; k < NMOTORS; k++)
    {
      int pwr, dir;
      // Evaluate the control signal
      pid[k].evalu(currentSpeed[k], targetSpeed[k], deltaT / 1e6, pwr, dir);
      // Signal the motor
      setMotor(dir, pwr, pwm[k], in1[k], in2[k]);
    }

    // Serial.print("s0:");
    // Serial.println(currentSpeed[0]);
    // Serial.print("s1:");
    // Serial.println(currentSpeed[1]);
  }

  // Reset the encoder count and detach interrupt for motor 0
  noInterrupts(); // Temporarily disable interrupts
  // encoder_count[0] = 0;
  // encoder_count[1] = 0;
  detachInterrupt(digitalPinToInterrupt(enca[0]));
  detachInterrupt(digitalPinToInterrupt(enca[1]));
  // interrupts(); // Re-enable interrupts

}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2)
{

  analogWrite(pwm, pwmVal);
  if (dir == 1)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else if (dir == -1)
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void stopAll()
{
  analogWrite(pwm[0], 0);
  analogWrite(pwm[1], 0);

  digitalWrite(in1[0], LOW);
  digitalWrite(in2[0], LOW);

  digitalWrite(in1[1], LOW);
  digitalWrite(in2[1], LOW);
}

template <int j>
void readEncoder()
{

  int b = digitalRead(encb[j]);
  if (b > 0)
  {
    encoder_count[j]++;

    Serial.println(encoder_count[j]);
  }
  else
  {
    encoder_count[j]--;
    Serial.println(encoder_count[j]);
  }
}



template <int j>
void onPulse()
{
  static bool lasta_onpulse = false;
  static bool Direction_onpulse = false;
  bool Lstate = digitalRead(enca[j]);
  if (!lasta_onpulse && Lstate)
  {
    bool val = digitalRead(encb[j]);
    if (!val && Direction_onpulse)
    {
      Direction_onpulse = false; // Reverse
    }
    else if (val && !Direction_onpulse)
    {
      Direction_onpulse = true; // Forward
    }
  }
  lasta_onpulse = Lstate;

  if (!Direction_onpulse)
  {
    // Positive pulse
    encoder_count[j]++;
  }
  else
  {
    // Negative pulse
    encoder_count[j]--;
  }

  // Serial.print("first:");
  // Serial.print(encoder_count[0]);
  // Serial.print(",second:");
  // Serial.println(encoder_count[1]);
}



bool lasta[2];
bool Direction[2];

template <int j>
void wheelSpeed()
{
  bool Lstate = digitalRead(enca[j]);
  if (!lasta[j] && Lstate)
  {
    bool val = digitalRead(encb[j]);
    if (!val && Direction[j])
    {
      Direction[j] = false; // Reverse
    }
    else if (val && !Direction[j])
    {
      Direction[j] = true; // Forward
    }
  }
  lasta[j] = Lstate;

  if (!Direction[j])
    duration[j]++;
  else
    duration[j]--;

  
  // Serial.print("first:");
  // Serial.print(duration[0]);
  // Serial.print(",second:");
  // Serial.println(duration[1]);

}
