// Define pins for the IR sensors
#define LEFT_SENSOR A0
#define RIGHT_SENSOR A1

// Define pins for the robot motors
#define LEFT_MOTOR 5
#define RIGHT_MOTOR 6

// Define PID parameters
double kp = 0.5;
double ki = 0.1;
double kd = 0.2;

// Define setpoint for the PID controller
double setpoint = 0.0;

// Initialize PID controller
double last_error = 0.0;
double integral = 0.0;
double derivative = 0.0;

// Define motor speeds
int base_speed = 150;
int max_speed = 255;
int min_speed = 50;

// Define error thresholds for the sensors
int left_threshold = 800;
int right_threshold = 800;

// Define function to read sensor values
void read_sensors(int* left, int* right) {
  *left = analogRead(LEFT_SENSOR);
  *right = analogRead(RIGHT_SENSOR);
}

// Define function to set motor speeds
void set_motors(int left_speed, int right_speed) {
  analogWrite(LEFT_MOTOR, left_speed);
  analogWrite(RIGHT_MOTOR, right_speed);
}

// Define PID function to control the robot
void pid_controller() {
  // Read sensor values
  int left_sensor, right_sensor;
  read_sensors(&left_sensor, &right_sensor);

  // Calculate error value
  double error = (double)(right_sensor - left_sensor);

  // Calculate PID terms
  integral += error;
  derivative = error - last_error;
  double output = kp * error + ki * integral + kd * derivative;

  // Update last error
  last_error = error;

  // Set motor speeds based on output value
  int left_speed = base_speed - output;
  int right_speed = base_speed + output;

  // Apply speed limits
  if (left_speed > max_speed) {
    left_speed = max_speed;
  }
  if (left_speed < min_speed) {
    left_speed = min_speed;
  }
  if (right_speed > max_speed) {
    right_speed = max_speed;
  }
  if (right_speed < min_speed) {
    right_speed = min_speed;
  }

  // Set motor speeds
  set_motors(left_speed, right_speed);
}

// Setup function
void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Set pin modes for sensors and motors
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);
  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);

  // Set initial motor speeds
  set_motors(base_speed, base_speed);
}

// Loop function
void loop() {
  // Call PID controller function
  pid_controller();

  // Print sensor values and error to serial monitor
  int left_sensor, right_sensor;
  read_sensors(&left_sensor, &right_sensor);
  double error = (double)(right_sensor - left_sensor);
  Serial.print("Left sensor: ");
  Serial.print(left_sensor);
  Serial.print(", Right sensor: ");
  Serial.print(right_sensor);
  Serial.print(", Error: ");
  Serial.println(error);

  // Wait for a short time before repeating
  delay(10);
}