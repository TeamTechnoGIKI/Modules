// Motor speed control for forward-backward and left-right.
int FB_speed = 80, LR_speed = 220;

// Pins to control the direction for motors rotation
int pin1 = 5, pin2 = 3, pin3 = 9, pin4 = 6;

// Ultrasonic sensors pins
//right=1, forward=2, left=3 
int trig1 = 10, echo1 = 11, trig2 = 7, echo2 = 8, trig3 = 2, echo3 = 4;
//leftIR, rightIR
int leftIR=12, rightIR=13;
float duration;
float distance1, distance2, distance3;
// Threshold distances for all three sensors 
const float set_distance1 = 13, set_distance2 = 20, set_distance3 = 13;

void setup() {
    Serial.begin(9600);
    // Set motor control pins as outputs
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    pinMode(pin3, OUTPUT);
    pinMode(pin4, OUTPUT);

    // Set ultrasonic sensor pins
    pinMode(trig1, OUTPUT);
    pinMode(trig2, OUTPUT);
    pinMode(trig3, OUTPUT); 
    pinMode(echo1, INPUT);
    pinMode(echo2, INPUT);
    pinMode(echo3, INPUT);

    //ir setup
    pinMode(leftIR,INPUT);
    pinMode(rightIR,INPUT);

    Serial.println("Setup Complete");
}

void loop() {
    self_driving();
    IR();
}

void self_driving() {
    // Measure distances from ultrasonic sensors
    distance1 = dist_measure(trig1, echo1);
    distance2 = dist_measure(trig2, echo2);
    distance3 = dist_measure(trig3, echo3);

//     Debugging: Print sensor readings
//    Serial.print("Distance Left: ");
//    Serial.println(distance1);
//    Serial.print("Distance Forward: ");
//    Serial.println(distance2);
//    Serial.print("Distance Right: ");
//    Serial.println(distance3);
//    delay(1000);

    // Make decision based on distances
    if (distance2 > set_distance2) {
//        Serial.println("Moving Forward"); // Debugging
        move_forward();              // Forward
    }
    else if (distance1 > set_distance1 && distance1 > distance3) {
//        Serial.println("Turning Right"); // Debugging
        turn_right();                // Right turn
    }
    else if (distance3 > set_distance3) {
//        Serial.println("Turning Left"); // Debugging
        turn_left();                 // Left turn
    }
    else {
//        Serial.println("No Clear Path, Moving Backward"); // Debugging
        move_backward_until_clear();  // No clear path, move backward until there is space
    }
}

void move_forward() {
    analogWrite(pin1, FB_speed);
    analogWrite(pin2, LOW);
    analogWrite(pin3, LOW);
    analogWrite(pin4, FB_speed);
    
//    Serial.println("Action: Forward"); // Debugging
}

void turn_right() {
    analogWrite(pin1, LOW);
    analogWrite(pin2, 100);
    analogWrite(pin3, LOW);
    analogWrite(pin4, LR_speed);

//    Serial.println("Action: Right Turn"); // Debugging
}

void turn_left() {
    analogWrite(pin1, LR_speed);
    analogWrite(pin2, LOW);
    analogWrite(pin3, 100);
    analogWrite(pin4, LOW);
    
//    Serial.println("Action: Left Turn"); // Debugging
}

void move_backward_until_clear() {
    while (true) {
        // Measure distances again
        distance1 = dist_measure(trig1, echo1);
        distance3 = dist_measure(trig3, echo3);

        // Debugging: Print distances while moving backward
//        Serial.print("Checking Distances while moving backward - Left: ");
//        Serial.print(distance1);
//        Serial.print(", Right: ");
//        Serial.println(distance3);

        if (distance1 > set_distance1 || distance3 > set_distance3)
            break;

        // Move backward
        analogWrite(pin1, LOW);
        analogWrite(pin2, FB_speed);
        analogWrite(pin3, FB_speed);
        analogWrite(pin4, LOW);

//        Serial.println("Action: Reverse"); // Debugging
    }

    // Decide which way to turn after moving backward
    if (distance1 > set_distance1 && distance1 > distance3) {
//        Serial.println("After Reverse, Turning Right"); // Debugging
        turn_right();
    } else if (distance3 > set_distance3) {
//        Serial.println("After Reverse, Turning Left"); // Debugging
        turn_left();
    }
    
}

float dist_measure(int trig, int echo) {
    duration = 0;
    // Send a pulse to the trigger pin
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    // Measure the duration of the pulse on the echo pin
    duration = pulseIn(echo, HIGH);
    
    // Calculate the distance based on the duration of the pulse
    float distance = ((duration * 0.034) / 2);

//    // Debugging: Print the measured distance
//    Serial.print("Measured Distance for trig ");
//    Serial.print(trig);
//    Serial.print(" and echo ");
//    Serial.print(echo);
//    Serial.print(": ");
//    Serial.println(distance);
//    
    return distance;
}

void IR(){
  if (!digitalRead(leftIR)){
//    Serial.println("Action: left edge detected ");
    turn_right();
  }
  else if(!digitalRead(rightIR)){
//        Serial.println("Action: right edge detected ");
    turn_left();
  }
}
