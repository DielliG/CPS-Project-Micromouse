// #include <NewPing.h>

const int enca[] = {18, 20};
const int encb[] = {19, 21};
const int pwm[] = {6, 7};
const int in1[] = {2, 4};
const int in2[] = {3, 5};

long encoder_pulses[2] = {};

template <int j>
void start_wheel(int pwmVal = 100)
{
	int dir = 0;
	if (pwmVal > 0)
		dir = 1;
	else if (pwmVal < 0)
	{
		dir = -1;
		pwmVal = -pwmVal;
	}
	analogWrite(pwm[j], pwmVal);
	if (dir == 1)
	{
		digitalWrite(in1[j], LOW);
		digitalWrite(in2[j], HIGH);
	}
	else if (dir == -1)
	{
		digitalWrite(in1[j], HIGH);
		digitalWrite(in2[j], LOW);
	}
	else
	{
		digitalWrite(in1[j], LOW);
		digitalWrite(in2[j], LOW);
	}
}

template <int j>
void stop_wheel() {
	analogWrite(pwm[j], 0);
	digitalWrite(in1[j], LOW);
	digitalWrite(in2[j], LOW);
}

char *BUFFER[] = {

"f1050",
"f1050",
"f1100",
"f1150",
// "f1500"
"r840",
"f1900",
"l840",
"f1500",
"r920",
"f1300",
"f1300",
"r900",
"f1300"
// "b500",
// "b500",
// "l600",
// "f500"
};

void setup() {
	Serial.begin(9600);

	for (int k = 0; k < 2; k++)
	{
		pinMode(enca[k], INPUT);
		pinMode(encb[k], INPUT);
		pinMode(pwm[k], OUTPUT);
		pinMode(in1[k], OUTPUT);
		pinMode(in2[k], OUTPUT);

		// pid[k].setParams(10, 100, 1, 255); // Example parameters, adjust as needed
	}
  // attachInterrupt(digitalPinToInterrupt(enca[0]), wheelSpeed<0>, RISING);
  // attachInterrupt(digitalPinToInterrupt(enca[1]), wheelSpeed<1>, RISING);
	// start_wheel<0>(100);
  // start_wheel<1>(-100);
  // delay(500);

  const long size = sizeof(BUFFER) / sizeof(char*);
  for(int i = 0; i < size; i++) {
    execute(BUFFER[i]);
  }
}

void execute(char* buffer) {
  char mode = buffer[0];
	int delta_pulses = atoi(buffer + 1);

  const int speed = 150;
  if (mode == 'f') {
		start_wheel<0>(speed-15);
		start_wheel<1>(speed);
	}
	else if (mode == 'l') {
		start_wheel<0>(-speed+15);
		start_wheel<1>(speed);
	}
	else if (mode == 'r') {
		start_wheel<0>(speed-15);
		start_wheel<1>(-speed);
	}
  else if (mode == 'b') {
		start_wheel<0>(-speed+15);
		start_wheel<1>(-speed);
	}
	else return;
  delay(delta_pulses);
  stop_wheel<0>();
  stop_wheel<1>();
}

void loop() {
  // start_wheel<1>(150);
  if (!Serial.available()) return;
	char cur = '\0';
	char buffer[10];
	for (int i = 0; i < 10; i++) {
		while (!Serial.available());
		cur = Serial.read();
    // Serial.println((int)cur);
		if (cur == 13) {
			buffer[i] = '\0';
			break;
		}
		buffer[i] = cur;
	}
  Serial.print("\"");
  Serial.print(buffer);
  Serial.println("\",");
  
  execute(buffer);
}

template <int j>
void wheelSpeed()
{
	if (digitalRead(encb[j])) encoder_pulses[j]++;
	else encoder_pulses[j]--;

  if (j) {
    Serial.print("left:");
    Serial.print(encoder_pulses[0]);
    Serial.print(",right:");
    Serial.println(encoder_pulses[1]);
  }
}