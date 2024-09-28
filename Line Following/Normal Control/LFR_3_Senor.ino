// Motor pins setup
int pin1 = 3, pin2 = 5, pin3 = 9, pin4 = 6;

// IR sensor pins setup
int leftIR = 4, centreIR = 7, rightIR = 8;

// Speed control
int base_speed = 130;          // Normal forward speed
int turn_speed = 150;           // Speed for sharp turns
int slight_turn_speed = 90;    // Speed for slight turns
int stability_delay = 40;      // Delay to stabilize movement

void setup() {
  // Motor pins setup
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(pin3, OUTPUT);
  pinMode(pin4, OUTPUT);

  // IR sensor pins setup
  pinMode(leftIR, INPUT);
  pinMode(centreIR, INPUT);
  pinMode(rightIR, INPUT);
  
  Serial.begin(9600); // Debugging
}

void loop() {
  // Read sensor values
  int left = digitalRead(leftIR);      // 1 if on black line, 0 if on white
  int centre = digitalRead(centreIR);  // 1 if on black line, 0 if on white
  int right = digitalRead(rightIR);    // 1 if on black line, 0 if on white

  // Combine sensor readings into a single value
  int sensorState = (left << 2) | (centre << 1) | right; // L C R in binary
  Serial.print("Sensor State: ");
  Serial.println(sensorState, BIN); // Print the state in binary

  // Implement logic based on sensorState
  switch (sensorState) {
    case 0b000: // 000: All sensors on white (Lost line)
      Serial.println("Lost line, searching...");
      search_for_line();
      break;

    case 0b001: // 001: Right on black (Turn left)
      Serial.println("Full Left Turn");
      turn_left();
      break;

//    case 0b010: // 010: Center on black (Move forward)
//      Serial.println("Moving Forward");
//      forward();
//      break;sensorState

    case 0b011: // 011: Center and right on black (Slight left turn)
      Serial.println("Slight Left Turn");
      slight_left();
      break;

    case 0b100: // 100: Left on black (Turn right)
      Serial.println("Full Right Turn");
      turn_right();
      break;

//    case 0b101: // 101: Left and right on black (Move forward)
//      Serial.println("Moving Forward");
//      forward();
//      break;

    case 0b110: // 110: Left and center on black (Slight right turn)
      Serial.println("Slight Right Turn");
      slight_right();
      break;

//    case 0b111: // 111: All sensors on black (Move forward)
//      Serial.println("Moving Forward");
//      forward();
//      break;
      default:
      Serial.println("deafult movement");
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
//  delay(400);  // Turn left for 400 ms
//  turn_right();
//  delay(400);  // Then turn right for 400 ms
//  forward();
//  delay(200);  // Move forward briefly
}
