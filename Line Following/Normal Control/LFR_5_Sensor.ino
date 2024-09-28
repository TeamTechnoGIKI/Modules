// Motor pins setup
int pin1 = 3, pin2 = 5, pin3 = 9, pin4 = 6;

// IR sensor pins setup
int leftIR = 4, centerIR = 7, rightIR = 8, leftIR2 = 11, rightIR2 = 10;

// Speed control
int base_speed = 130;          // Normal forward speed
int turn_speed = 150;          // Speed for sharp turns
int slight_turn_speed = 90;     // Speed for slight turns
int stability_delay = 40;      // Delay to stabilize movement

void setup() {
  // Motor pins setup
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(pin3, OUTPUT);
  pinMode(pin4, OUTPUT);

  // IR sensor pins setup
  pinMode(leftIR, INPUT);
  pinMode(centerIR, INPUT);
  pinMode(rightIR, INPUT);
  pinMode(leftIR2, INPUT);
  pinMode(rightIR2, INPUT);
  
  Serial.begin(9600); // Debugging
}

void loop() {
  // Read sensor values
  int left2 = digitalRead(leftIR2);    // 1 if on black line, 0 if on white
  int left1 = digitalRead(leftIR);      // 1 if on black line, 0 if on white
  int center = digitalRead(centerIR);   // 1 if on black line, 0 if on white
  int right1 = digitalRead(rightIR);    // 1 if on black line, 0 if on white
  int right2 = digitalRead(rightIR2);   // 1 if on black line, 0 if on white

  // Combine sensor readings into a single value
  int sensorState = (left2 << 4) | (left1 << 3) | (center << 2) | (right1 << 1) | right2; // L2 L1 C R1 R2 in binary
  Serial.print("Sensor State: ");
  Serial.println(sensorState, BIN); // Print the state in binary

  // Implement logic based on sensorState
  switch (sensorState) {
    case 0b00000: // All sensors on white (Lost line)
      Serial.println("Lost line, searching...");
      search_for_line();
      break;

    case 0b00001: // Rightmost on black (Turn left)
      Serial.println("Sharp Left Turn");
      turn_left();
      break;

    case 0b00010: // Right on black (Slight left turn)
      Serial.println("Slight Left Turn");
      slight_left();
      break;

    case 0b00011: // Center and right on black (Slight left turn)
      Serial.println("Slight Left Turn");
      slight_left();
      break;

    case 0b00100: // Center on black (Move forward)
      Serial.println("Moving Forward");
      forward();
      break;

    case 0b00101: // Center and rightmost on black (Move forward/slight left)
      Serial.println("Moving Forward");
      forward();
      break;

    case 0b00110: // Center and right on black (Move forward)
      Serial.println("Moving Forward");
      forward();
      break;

    case 0b00111: // Center, right, and rightmost on black (Slight left turn)
      Serial.println("Slight Left Turn");
      slight_left();
      break;

    case 0b01000: // Left on black (Slight right turn)
      Serial.println("Slight Right Turn");
      slight_right();
      break;

    case 0b01001: // Left and rightmost on black (Move forward/slight right)
      Serial.println("Moving Forward");
      forward();
      break;

    case 0b01010: // Left and right on black (Move forward/slight right)
      Serial.println("Moving Forward");
      forward();
      break;

    case 0b01011: // Left, right, and rightmost on black (Slight left turn)
      Serial.println("Slight Left Turn");
      slight_left();
      break;

    case 0b01100: // Left and center on black (Slight right turn)
      Serial.println("Slight Right Turn");
      slight_right();
      break;

    case 0b01101: // Left, center, and rightmost on black (Slight right turn)
      Serial.println("Slight Right Turn");
      slight_right();
      break;

    case 0b01110: // Left, center, and right on black (Move forward)
      Serial.println("Moving Forward");
      forward();
      break;

    case 0b01111: // Left, center, right, and rightmost on black (Move forward/slight right)
      Serial.println("Moving Forward");
      forward();
      break;

    case 0b10000: // Leftmost on black (Sharp right turn)
      Serial.println("Sharp Right Turn");
      turn_right();
      break;

    case 0b10001: // Leftmost and rightmost on black (Move forward/slight turn)
      Serial.println("Moving Forward");
      forward();
      break;

    case 0b10010: // Leftmost and right on black (Move forward/slight left)
      Serial.println("Moving Forward");
      forward();
      break;

    case 0b10011: // Leftmost, right, and rightmost on black (Slight left turn)
      Serial.println("Slight Left Turn");
      slight_left();
      break;

    case 0b10100: // Leftmost and center on black (Move forward/slight right)
      Serial.println("Moving Forward");
      forward();
      break;

    case 0b10101: // Leftmost, center, and rightmost on black (Move forward/slight right)
      Serial.println("Moving Forward");
      forward();
      break;

    case 0b10110: // Leftmost, center, and right on black (Move forward/slight right)
      Serial.println("Moving Forward");
      forward();
      break;

    case 0b10111: // Leftmost, center, right, and rightmost on black (Slight right turn)
      Serial.println("Slight Right Turn");
      slight_right();
      break;

    case 0b11000: // Leftmost and left on black (Slight right turn)
      Serial.println("Slight Right Turn");
      slight_right();
      break;

    case 0b11001: // Leftmost, left, and rightmost on black (Slight right turn)
      Serial.println("Slight Right Turn");
      slight_right();
      break;

    case 0b11010: // Leftmost, left, and right on black (Move forward/slight right)
      Serial.println("Moving Forward");
      forward();
      break;

    case 0b11011: // Leftmost, left, right, and rightmost on black (Slight left turn)
      Serial.println("Slight Left Turn");
      slight_left();
      break;

    case 0b11100: // Leftmost, left, and center on black (Slight right turn)
      Serial.println("Slight Right Turn");
      slight_right();
      break;

    case 0b11101: // Leftmost, left, center, and rightmost on black (Moving forward/slight right)
      Serial.println("Moving Forward");
      forward();
      break;

    case 0b11110: // Leftmost, left, center, and right on black (Moving forward/slight right)
      Serial.println("Moving Forward");
      forward();
      break;

    case 0b11111: // All sensors on black (Move forward)
      Serial.println("Moving Forward");
      forward();
      break;

    default:
      Serial.println("Default movement");
      forward();
      break;
  }

  delay(stability_delay);  // Allow for smoother motor movements
}

// Functions for robot movement
void forward() {
  analogWrite(pin1, base_speed);
  analogWrite(pin2, LOW);
  analogWrite(pin3, LOW);
  analogWrite(pin4, base_speed);
}

void turn_left() {
  analogWrite(pin1, turn_speed);
  analogWrite(pin2, LOW);
  analogWrite(pin3, LOW);
  analogWrite(pin4, LOW);
}

void slight_left() {
  analogWrite(pin1, slight_turn_speed);
  analogWrite(pin2, LOW);
  analogWrite(pin3, LOW);
  analogWrite(pin4, LOW);
}

void turn_right() {
  analogWrite(pin1, LOW);
  analogWrite(pin2, LOW);
  analogWrite(pin3, LOW);
  analogWrite(pin4, turn_speed);
}

void slight_right() {
  analogWrite(pin1, LOW);
  analogWrite(pin2, LOW);
  analogWrite(pin3, LOW);
  analogWrite(pin4, slight_turn_speed);
}

void stop_motors() {
  analogWrite(pin1, LOW);
  analogWrite(pin2, LOW);
  analogWrite(pin3, LOW);
  analogWrite(pin4, LOW);
}

// Searching for the line
void search_for_line() {
  turn_left();
}
