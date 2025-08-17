  
  // --- L293D Motor Driver Pin Definitions ---
  // Motor 1 (for example, left motor)
  #define MOTOR1_IN1 8
  #define MOTOR1_IN2 10
  #define MOTOR1_ENA 9

  // Motor 2 (for example, right motor)
  #define MOTOR2_IN1 12
  #define MOTOR2_IN2 13
  #define MOTOR2_ENB 11


  // --- Ultrasonic Sensor Pin Definitions ---
  // Left sensor
  #define TRIG_LEFT 7
  #define ECHO_LEFT 6
  // Front sensor
  #define TRIG_FRONT 5
  #define ECHO_FRONT 4
  // Right sensor
  #define TRIG_RIGHT 3
  #define ECHO_RIGHT 2

  bool firstRun = true;   //Counter to check for First Run

  // Lap timing: every 10 seconds counts as one lap.
  const unsigned long lapInterval = 20000; // 20,000 ms - 6000 ms(for the initial delay) = 14000ms = 14 seconds for each lap
  unsigned long lapStartTime = 0;    // Time stamp for the start of the current lap
  unsigned int lapCount = 0;         // Lap counter

  

  void setup() {
    // Initialise Arduino Mega Digital Pin 22 to provide 5V Output for the Ultrasonic Sensors
    pinMode(22, OUTPUT);
    // Initialize motor driver pins as outputs
    pinMode(MOTOR1_IN1, OUTPUT);
    pinMode(MOTOR1_IN2, OUTPUT);
    pinMode(MOTOR1_ENA, OUTPUT);

    pinMode(MOTOR2_IN1, OUTPUT);
    pinMode(MOTOR2_IN2, OUTPUT);
    pinMode(MOTOR2_ENB, OUTPUT);
  
    // Initialize ultrasonic sensor pins
    pinMode(TRIG_LEFT, OUTPUT);
    pinMode(ECHO_LEFT, INPUT);
    pinMode(TRIG_FRONT, OUTPUT);
    pinMode(ECHO_FRONT, INPUT);
    pinMode(TRIG_RIGHT, OUTPUT);
    pinMode(ECHO_RIGHT, INPUT);
  
    // Set the initial lap start time
    lapStartTime = millis();
  }

  void loop() {
    digitalWrite(22, HIGH); // Send 5V to the digital pin 22
    if (firstRun) {
      delay(60000);  // Add a 60 second delay for the first run to help setup the motor driver
      firstRun = false;  // 
    }
    unsigned long currentTime = millis();
    unsigned long lapTime = currentTime - lapStartTime;
  
    // --- Lap Timing ---
    if (lapTime >= lapInterval) {
      lapCount++;

      // After 10 laps, stop the vehicle
      if (lapCount >= 10) {
        stopMotors();
        while (true) {
        // Halt further execution
        }
      }
      // Reset lap timer for next lap
      lapStartTime = currentTime;
    }
  
    // --- Read Ultrasonic Sensors ---
    int distanceFront = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
    int distanceLeft  = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
    int distanceRight = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);
  
    // --- Decision Making Using Sensor Readings ---
    // If a wall (or corner) is detected by the front sensor, perform a pivot turn.
    // The turning direction is chosen by comparing left and right sensor readings.
    if (distanceFront <= 18) {
      // Pivot turn using differential motor speeds
      if (distanceLeft > distanceRight) {
        // Turn (swerve) to the left: left motor rotates backward, right motor rotates forward.
        setMotorSpeeds(0, 1);
        delay(30);
      }
      if (distanceLeft > 50 && distanceRight > 50) {
        // Stops both the motors rotation and halts the movement of the vehicle
        setMotorSpeeds(0, 0);
      }
    } else {
        // Otherwise, continue moving straight ahead at the set speed.
        setMotorSpeeds(1, 1);
        if(distanceLeft < 15) {
          // Turn (swerve) to the right: left motor rotates forward, right motor rotates backward.
          setMotorSpeeds(1, 0);
          delay(20);
        }
        if(distanceRight < 15) {
          // Turn (swerve) to the left: left motor rotates backward, right motor rotates forward.
          setMotorSpeeds(0, 1);
          delay(20);
        }
      }
  }

  // --- Function to Read Distance from an Ultrasonic Sensor ---
  // Returns the distance in centimeters.
  int readUltrasonic(int trigPin, int echoPin) {
    long duration, distance;
  
    // Ensure the trigger is low
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
  
    // Send a 10-microsecond pulse to trigger the sensor
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
  
    // Read the echo time
    duration = pulseIn(echoPin, HIGH);
  
    // Calculate the distance in centimeters:
    // (duration in µs) * (speed of sound 0.034 cm/µs) / 2
    distance = duration * 0.0343 / 2;
  
    return distance;
  }

  // --- Motor Control Function ---
  // The below function helps set the rotational direction of both motors 1 and 2 depending upon the variables speed1 and speed2
  void setMotorSpeeds(int speed1, int speed2) {
    // Control Motor 1
    if (speed1 == 0) {
      // Motor 1(say the left motor) rotates backward at speed1 = 0
      digitalWrite(MOTOR1_IN1, LOW);
      digitalWrite(MOTOR1_IN2, HIGH);
      analogWrite(MOTOR1_ENA, 255);
    } else {
      // Motor 1(say the left motor) rotates forward at speed1 = 1
        digitalWrite(MOTOR1_IN1, HIGH);
        digitalWrite(MOTOR1_IN2, LOW);
        analogWrite(MOTOR1_ENA, 255);
      }
  
    // Control Motor 2
    if (speed2 == 0) {
      // Motor 2(say the right motor) rotates backward at speed2 = 0
      digitalWrite(MOTOR2_IN1, LOW);
      digitalWrite(MOTOR2_IN2, HIGH);
      analogWrite(MOTOR2_ENB, 255);
    } else {
        // Motor 2(say the right motor) rotates forward at speed2 = 1
        digitalWrite(MOTOR2_IN1, HIGH);
        digitalWrite(MOTOR2_IN2, LOW);
        analogWrite(MOTOR2_ENB, 255);
      }
  }

  
  // --- Function to Stop Both Motors ---
  void stopMotors() {
    // Both motor 1 and motor 2 stop their rotation as soon as this function is called within the loop
    digitalWrite(MOTOR1_IN1, LOW);
    digitalWrite(MOTOR1_IN2, LOW);
    analogWrite(MOTOR1_ENA, 0);
    digitalWrite(MOTOR2_IN1, LOW);
    digitalWrite(MOTOR2_IN2, LOW);
    analogWrite(MOTOR2_ENB, 0);
  }