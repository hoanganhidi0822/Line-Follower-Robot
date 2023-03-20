float kp = 27, ki = 0.009, kd = 350;  //kp18
float error = 0, p = 0, i = 0, d = 0, pid = 0;
float pre_e = 0;
int ss[5] = { 0, 0, 0, 0, 0 };
int initial_speed = 140;  // 80
int right_pid, left_pid;

int turnSpeed = 80;  //80

int servopin = 8;
int left_Distance;
int right_Distance;
int Forward_Distance;
float curve_value = 1.75;

boolean side;

unsigned long duration;
double Distance = 0;
double distance1 = 0;
int obj_distance = 18;

double kaldist;
double distance = 0;

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

  Distance = get_distance(trig, echo);
  distance = kalman(Distance);

  if (distance >= obj_distance) {
    ss_read();
    cal();
    motor();
  } else if (distance <= obj_distance && error <= 3) {
    stop(300);
    check();
  }

  Serial.print("Chua loc: ");
  Serial.println(Distance); 
  
  Serial.print("Da loc: ");
  Serial.println(distance); 
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
  double distance, duration;
  digitalWrite(trig, 0);
  delayMicroseconds(2);
  digitalWrite(trig, 1);
  delayMicroseconds(5);
  digitalWrite(trig, 0);

  duration = pulseIn(echo, HIGH);
  distance = duration * 0.034 / 2;

  if (distance >= 40) {
    distance = 40;
  }
  delay(20);
  return distance;
}

void Servo_set_point(int pos) {
  servoPulse(servopin, pos);
  delay(20);
}

void check() {
  delay(50);
  Forward_Distance = get_distance(trig, echo);
  delay(20);
  if (Forward_Distance > obj_distance + 10) {
    loop();
  } else {
    for (int pos = 90; pos <= 160; pos += 10) {
      servoPulse(servopin, pos);
      delay(30);
    }
    delay(50);
    left_Distance = get_distance(trig, echo);
    delay(100);

    for (int pos = 160; pos >= setpos; pos -= 10) {
      servoPulse(servopin, pos);
      delay(30);
    }
    delay(50);
    Forward_Distance = get_distance(trig, echo);
    delay(100);
    for (int pos = setpos; pos >= 20; pos -= 10) {
      servoPulse(servopin, pos);
      delay(30);
    }
    delay(50);
    right_Distance = get_distance(trig, echo);
    delay(100);

    for (int pos = 20; pos <= 90; pos += 10) {
      servoPulse(servopin, pos);
      delay(30);
    }
    delay(100);
    compareDistance();
  }
}

void compareDistance() {

  if ((right_Distance > left_Distance) && right_Distance > 20) {
    Serial.print("Turn Right!!!!!!!!");
    avoid(true);

  } else if ((right_Distance < left_Distance) && left_Distance > 20) {
    Serial.print("Turn Left!!!!!!!!");
    side = false;
    avoid(side);

  } else {
    avoid(true);
  }
}

double kalman(double U) {
  static const double R = 40;
  static const double H = 1.00;
  static double Q = 10;
  static double P = 0;
  static double U_hat = 0;
  static double K = 0;
  K = P * H / (H * P * H + R);
  U_hat += +K * (U - H * U_hat);
  P = (1 - K * H) * P + Q;
  return U_hat;
}

void avoid(int side) {
  if (side == true) {

    go_BackWard(500);
    stop(500);
    turnRight(280);
    stop(700);
    go_Forward(700);
    stop(200);
    turnLeft(250);
    stop(250);

    distance1 = get_distance(trig, echo);
    delay(15);

    if (distance1 < obj_distance + 10) {
      stop(500);
      check();
    } else {
      go_Forward(800);
      turnLeft(170);
      while (ss[0] != 0) {
        analogWrite(ena, turnSpeed + 20);
        analogWrite(enb, turnSpeed - 15);
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        ss[0] = digitalRead(A0);
      }
    }
    stop(140);
    error = 2;
    loop();

  } else if (side == false) {  //Left_avoid
    go_BackWard(500);
    stop(500);
    turnLeft(280);
    stop(700);
    go_Forward(700);
    stop(200);
    turnRight(250);
    stop(250);

    distance1 = get_distance(trig, echo);
    delay(15);

    if (distance1 < obj_distance + 10) {
      stop(500);
      side = false;
      check();
    } else {
      go_Forward(800);
      turnRight(170);
      while (ss[4] != 0) {
        analogWrite(ena, turnSpeed - 15);
        analogWrite(enb, turnSpeed + 20);
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        ss[4] = digitalRead(A4);
      }
    }
    stop(140);
    error = -2;
    loop();
  }
}

void stop(int time_stop) {
  analogWrite(ena, 0);
  analogWrite(enb, 0);
  digitalWrite(in1, LOW); 
  digitalWrite(in2, LOW); 
  digitalWrite(in3, LOW); 
  digitalWrite(in4, LOW); 
  delay(time_stop);
}
void go_Forward(int delay_time) {
  analogWrite(ena, turnSpeed);
  analogWrite(enb, turnSpeed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  delay(delay_time);
}
void go_BackWard(int delay_time) {
  analogWrite(ena, turnSpeed);
  analogWrite(enb, turnSpeed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(delay_time);                                                          
}
void turnLeft(int delay_time) {
  analogWrite(ena, turnSpeed);
  analogWrite(enb, turnSpeed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  delay(delay_time);
}
void turnRight(int delay_time) {
  analogWrite(ena, turnSpeed);
  analogWrite(enb, turnSpeed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(delay_time);
}