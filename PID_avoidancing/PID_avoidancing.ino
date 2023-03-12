

float kp = 20, ki = 0.009, kd = 350;
float error = 0, p = 0, i = 0, d = 0, pid = 0;
float pre_e = 0;
int ss[5] = { 0, 0, 0, 0, 0 };
int initial_speed = 80;
int right_pid, left_pid;
int obj_distance = 14;


int servopin = 8;

int left_Distance;
int right_Distance;
float curve_value = 1.75;


unsigned long duration;
float distance;

#define in1 7
#define in2 11
#define in3 12
#define in4 13
#define ena 9
#define enb 10
int trig = 6;
int echo = 5;
int setpos = 90;

void setup() {
  // put your setup code here, to run once:
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(servopin, OUTPUT);

  Serial.begin(9600);

  Servo_set_point(90);
}

void loop() {



  distance = get_distance(trig, echo);
  if (distance >= obj_distance) {

    ss_read();
    cal();
    motor();

  } else {
    stop(1000);
    check();
  }

  Serial.print(distance);
  Serial.println("cm");
}

void servoPulse(int pin, int angle) {
  int pwm = (angle * 11) + 500;  // Convert angle to microseconds
  digitalWrite(pin, HIGH);
  delayMicroseconds(pwm);
  digitalWrite(pin, LOW);
  delay(50);  // Refresh cycle of servo
}

void ss_read() {
  int a = 1;
  int b = 0;
  ss[0] = digitalRead(A0);
  ss[1] = digitalRead(A1);
  ss[2] = digitalRead(A2);
  ss[3] = digitalRead(A3);
  ss[4] = digitalRead(A4);



  if ((ss[0] == a) && (ss[1] == a) && (ss[2] == a) && (ss[3] == a) && (ss[4] == b))
    error = 4;
  if ((ss[0] == a) && (ss[1] == a) && (ss[2] == a) && (ss[3] == b) && (ss[4] == b))
    error = 3;
  if ((ss[0] == a) && (ss[1] == a) && (ss[2] == a) && (ss[3] == b) && (ss[4] == a))
    error = 2;
  if ((ss[0] == a) && (ss[1] == a) && (ss[2] == b) && (ss[3] == b) && (ss[4] == a))
    error = 1;
  if ((ss[0] == a) && (ss[1] == a) && (ss[2] == b) && (ss[3] == a) && (ss[4] == a))
    error = 0;
  if ((ss[0] == a) && (ss[1] == b) && (ss[2] == b) && (ss[3] == a) && (ss[4] == a))
    error = -1;
  if ((ss[0] == a) && (ss[1] == b) && (ss[2] == a) && (ss[3] == a) && (ss[4] == a))
    error = -2;
  if ((ss[0] == b) && (ss[1] == b) && (ss[2] == a) && (ss[3] == a) && (ss[4] == a))
    error = -3;
  if ((ss[0] == b) && (ss[1] == a) && (ss[2] == a) && (ss[3] == a) && (ss[4] == a))
    error = -4;
  if ((ss[0] == a) && (ss[1] == a) && (ss[2] == b) && (ss[3] == b) && (ss[4] == b)) {
    analogWrite(ena, 0);
    analogWrite(enb, initial_speed + 100);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    delay(500);
    Serial.println("Turn L45");
  }
}



void cal() {
  p = error;
  i = constrain(i + error, -35, 35);
  d = error - pre_e;
  pid = (kp * p) + (ki * i) + (kd * d);
  pre_e = error;
}

void motor() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  int right_pid = initial_speed - pid;
  int left_pid = initial_speed + pid;
  right_pid = constrain(right_pid, 0, 255);
  left_pid = constrain(left_pid, 0, 255);
  analogWrite(ena, right_pid);
  analogWrite(enb, left_pid);
}

int get_distance(int trig, int echo) {
  int distance, duration;
  digitalWrite(trig, 0);
  delayMicroseconds(2);
  digitalWrite(trig, 1);
  delayMicroseconds(5);
  digitalWrite(trig, 0);
  duration = pulseIn(echo, HIGH);
  distance = duration * 0.034 / 2;
  return distance;
}

void Servo_set_point(int pos) {
  servoPulse(servopin, pos);
  delay(20);
}

void check() {

  for (int pos = 90; pos <= 180; pos += 5) {
    servoPulse(servopin, pos);
    delay(15);
    left_Distance = get_distance(trig, echo);
    delay(50);
  }
  delay(100);

  for (int pos = 180; pos >= setpos; pos -= 5) {
    servoPulse(servopin, pos);
    delay(15);
  }

  for (int pos = setpos; pos >= 0; pos -= 5) {
    servoPulse(servopin, pos);
    delay(15);
    right_Distance = get_distance(trig, echo);
    delay(50);
  }
  delay(100);

  for (int pos = 0; pos <= 90; pos += 5) {
    servoPulse(servopin, pos);
    delay(15);
  }

  delay(300);
  compareDistance();
}

void compareDistance() {

  if ((right_Distance > left_Distance) && right_Distance > 20) {
    Serial.print("Turn Right!!!!!!!!");
    go_BackWard(500);
    stop(500);
    turnRight(250);
    stop(500);
    go_Forward(500);
    stop(200);
    turnLeft(170);
    go_Forward(500);
    
    while (ss[0] != 0) {
      analogWrite(ena, initial_speed + 10);
      analogWrite(enb, initial_speed - 15);
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      ss[0] = digitalRead(A0);
    }
    stop(500);
    
    //right 1, left 0

  } else if ((right_Distance < left_Distance) && left_Distance > 20) {
    Serial.print("Turn Left!!!!!!!!");
    go_BackWard(400);
    stop(500);
    turnLeft(320);
    stop(500);
    curve(0);
    delay(350);  //right 1, left 0
  }
}
void stop(int time_stop) {
  analogWrite(ena, 0);
  analogWrite(enb, 0);
  digitalWrite(in1, LOW);  //Left Motor backword Pin
  digitalWrite(in2, LOW);  //Left Motor forword Pin
  digitalWrite(in3, LOW);  //Right Motor forword Pin
  digitalWrite(in4, LOW);  //Right Motor backword Pin
  delay(time_stop);
}
void go_Forward(int delay_time) {
  analogWrite(ena, initial_speed);
  analogWrite(enb, initial_speed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  delay(delay_time);
}
void go_BackWard(int delay_time) {
  analogWrite(ena, initial_speed);
  analogWrite(enb, initial_speed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(delay_time);
}
void turnLeft(int delay_time) {
  analogWrite(ena, initial_speed);
  analogWrite(enb, initial_speed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  delay(delay_time);
}
void turnRight(int delay_time) {
  analogWrite(ena, initial_speed);
  analogWrite(enb, initial_speed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(delay_time);
}
void curve(int side) {
  if (side == 0) {
    analogWrite(ena, 50);
    analogWrite(enb, 100);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  } else if (side == 1) {
    analogWrite(ena, 50);
    analogWrite(enb, 100);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
}

void avoidance() {
  distance = get_distance(trig, echo);

  while (distance <= obj_distance) {
    stop(1000);
    check();
  }

  Serial.print(distance);
  Serial.println("cm");
}