float kp = 35, ki = 0.009, kd = 300;
float error = 0 , p = 0 , i = 0 , d = 0, pid = 0;
float pre_e  = 0;
int ss[5] = {0, 0, 0, 0, 0};
int initial_speed = 96;
int right_pid, left_pid;
#define in1 5
#define in2 4
#define in3 7
#define in4 6
#define ena 9
#define enb 10
#define IR 8

void setup() {
  // put your setup code here, to run once:
  pinMode(ena , OUTPUT);
  pinMode(enb , OUTPUT);
  pinMode(in1 , OUTPUT);
  pinMode(in2 , OUTPUT);
  pinMode(in3 , OUTPUT);
  pinMode(in4 , OUTPUT);
  pinMode(IR , INPUT);
  Serial.begin(9600);

  while (digitalRead(IR) == 1) {
    Serial.println("Waiting....");
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  ss_read();
  cal();
  motor();
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
  if ((ss[0] == b) && (ss[1] == b) && (ss[2] == b) && (ss[3] == b) && (ss[4] == b)) {
    while (true) {
      analogWrite(ena, initial_speed);
      analogWrite(enb, initial_speed);
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
      Serial.println("Back");
    }
  }
}

void cal() {
  p = error;
  i = constrain( i + error, -35, 35);
  d = error - pre_e;
  pid = (kp * p) + (ki * i) + (kd * d);
  pre_e = error;
}

void motor() {
  digitalWrite(in1 , HIGH);
  digitalWrite(in2 , LOW);
  digitalWrite(in3 , LOW);
  digitalWrite(in4 , HIGH);

  int right_pid = initial_speed - pid;
  int left_pid = initial_speed + pid;
  right_pid = constrain(right_pid, 0, 255);
  left_pid = constrain(left_pid, 0, 255);
  analogWrite(ena, right_pid);
  analogWrite(enb, left_pid);

  Serial.print(digitalRead(A0));
  Serial.print(digitalRead(A1));
  Serial.print(digitalRead(A2));
  Serial.print(digitalRead(A3));
  Serial.print(digitalRead(A4));
  Serial.println(d);
}
