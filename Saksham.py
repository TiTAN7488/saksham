//experiment 1 ka code 
Define pin numbers
const int buttonPin = 2;   // Push button connected to digital pin 2
const int ledPin = 3;      // LED connected to digital pin 3

void setup() {
    pinMode(buttonPin, INPUT);   // Set pin 2 as INPUT to read button state
    pinMode(ledPin, OUTPUT);     // Set pin 3 as OUTPUT to control the LED
    Serial.begin(9600);          // Start Serial Monitor at 9600 baud
}

void loop() {
    int buttonState = digitalRead(buttonPin);  // Read the button state

    if (buttonState == HIGH) {                // If button is pressed
        digitalWrite(ledPin, HIGH);           // LED ON
        Serial.println("Button Pressed - LED ON");
    } else {
        digitalWrite(ledPin, LOW);            // LED OFF
        Serial.println("Button Released - LED OFF");
    }

    delay(100);  // 100ms delay for debounce
}



2nd ka 
// Define pin number
const int sensorPin = A0;    // LM35 connected to analog pin A0
float temperatureC = 0;      // Variable to store temperature in Celsius

void setup() {
    Serial.begin(9600);      // Start Serial Monitor at 9600 baud rate
}

void loop() {
    int sensorValue = analogRead(sensorPin);          // Read analog value (0–1023)

    float voltage = sensorValue * (5.0 / 1024.0);     // Convert to voltage (0–5V)

    temperatureC = voltage / 0.01;                    // LM35 = 10mV per °C

    Serial.print("Temperature: ");
    Serial.print(temperatureC);
    Serial.println(" °C");

    delay(1000);                                      // 1 sec delay
}


3rd ka code
#include <Servo.h>        // Include Servo library

Servo myServo;            // Create servo object

const int potPin = A0;    // Potentiometer on analog pin A0
int potValue = 0;         // To store analog value
int angle = 0;            // To store servo angle (0–180)

void setup() {
    myServo.attach(9);    // Attach servo to digital PWM pin 9
    Serial.begin(9600);   // Start Serial Monitor
}

void loop() {
    potValue = analogRead(potPin);           // Read analog value (0–1023)
    angle = map(potValue, 0, 1023, 0, 180);  // Convert to 0–180 degrees
    myServo.write(angle);                    // Rotate servo

    // Serial output
    Serial.print("Potentiometer: ");
    Serial.print(potValue);
    Serial.print(" -> Servo Angle: ");
    Serial.println(angle);

    delay(15);   // Smooth movement
}


4th ka code
const int trigPin = 9;      // Trig pin
const int echoPin = 10;     // Echo pin

long duration;              // Time for echo
float distanceCm;           // Distance in cm

void setup() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    Serial.begin(9600);
}

void loop() {
    // Trigger pulse
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Read echo
    duration = pulseIn(echoPin, HIGH);

    // Convert to distance (cm)
    distanceCm = (duration * 0.0343) / 2;

    // Print distance
    Serial.print("Distance: ");
    Serial.print(distanceCm);
    Serial.println(" cm");

    delay(500);
}



5th ka code
// Motor driver pins
#define ENA 5     // Enable pin (PWM)
#define IN1 8     // Direction pin 1
#define IN2 9     // Direction pin 2
#define POT A0    // Potentiometer pin

int potValue = 0;
int motorSpeed = 0;

void setup() {
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(POT, INPUT);
}

void loop() {
    potValue = analogRead(POT);                 // Read potentiometer (0–1023)
    motorSpeed = map(potValue, 0, 1023, 0, 255); // Convert to PWM (0–255)

    Motor_CW();                                 // Rotate motor clockwise
    analogWrite(ENA, motorSpeed);               // Apply PWM speed
    delay(100);
}

// Rotate clockwise
void Motor_CW() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
}

// Rotate counter-clockwise
void Motor_CCW() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
}

// Stop motor
void Motor_Stop() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
}


6th ka code
#define LM35 A0       // LM35 connected to analog pin A0

int sensorValue = 0;
float temperature = 0.0;

void setup() {
    Serial.begin(9600);
    while (!Serial) delay(50);   // Wait for Serial
    
    Initialize_PlxDaq();         // Setup PLX-DAQ headers
}

void loop() {
    sensorValue = analogRead(LM35);

    // Convert ADC reading to °C
    temperature = (sensorValue * 5.0 * 100.0) / 1024.0;

    sendDataToPlxDaq();          // Send data to Excel

    delay(1000);                 // Log every 1 second
}

void sendDataToPlxDaq() {
    Serial.print("DATA,");
    Serial.print(millis() / 1000.0);   // Time in seconds
    Serial.print(",");
    Serial.println(temperature);       // Temperature value
}

void Initialize_PlxDaq() {
    Serial.println("CLEARDATA");                 
    Serial.println("LABEL,Time (s),Temperature (°C)");
    Serial.println("RESETTIMER");
}


7th ka code
#define IN1 8
#define IN2 9
#define IN3 10
#define IN4 11

int period = 20;   // Delay between steps (controls speed)

void setup() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

void loop() {
    // Rotate Clockwise
    for (int i = 0; i < 30; i++) {
        fullStepCW();
    }
    delay(1000);

    // Rotate Anti-Clockwise (optional)
    /*
    for (int i = 0; i < 30; i++) {
        fullStepCCW();
    }
    delay(1000);
    */
}

// Clockwise full-step sequence
void fullStepCW() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    delay(period);

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    delay(period);

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(period);

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(period);
}

// Anti-clockwise full-step sequence
void fullStepCCW() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(period);

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(period);

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    delay(period);

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    delay(period);
}

