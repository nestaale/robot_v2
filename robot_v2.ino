/* 
--------------------------------
United Nations Robot Competition
     Brindisi 29th May 2025
--------------------------------

    Basic demonstration robot

*/

#include <WiFi.h>
#include <WebServer.h>
#include "Adafruit_TCS34725.h"
#include <ESP32Servo.h>

// ------------------------------------------
//             VERY IMPORTANT!!!
// ------------------------------------------
// Replace with your network credentials
// Do not use this default credential
// to avoid confusion during the competition!
// ------------------------------------------
const char* ssid     = "UN-Robot";
const char* password = "123456789!";

// Initialize TCS34725 sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);

// Set web server port number to 80
WebServer server(80);

// Motor pins
const int motor1Pin1 = 27; 
const int motor1Pin2 = 26; 
const int enable1Pin = 14;

const int motor2Pin1 = 33; 
const int motor2Pin2 = 25; 
const int enable2Pin = 32;

// Ultrasonic Sensor pins
const int trigPin = 5;
const int echoPin = 18;

//define sound speed in cm/uS
#define SOUND_SPEED 0.034

long duration;
float distanceCm;

// Servo Pin
static const int servoPin = 13;
Servo servo1;

// Red Green Blue Clear variables for color sensor
uint16_t r, g, b, c;

bool movingForward=false;

bool followLine=false;

// PID parameters
float Kp = 24.0;  // Proportional gain
float Ki = 1.0;   // Integral gain
float Kd = 1.0;   // Derivative gain
int baseSpeed = 60;  // Base speed
 
// Middle value for black/white
int middleValue = 300; // Adjusted based on measured values
 
// Global variables for PID
float error = 0, previousError = 0, sumError = 0;
 

// Onboard LED pin (usually GPIO 2 on ESP32)
const int onboardLedPin = 2;

// PWM properties
const int freq = 30000;
const int resolution = 8;
int dutyCycle = 0;
int dutyCycleLeft = 0;
int dutyCycleRight = 0;

String valueString = String(0);

// HTML content
const char html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no" />
  <link rel="icon" href="data:,">
  <style>
    html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center; }
    body { touch-action: none; }
    .button { -webkit-user-select: none; -moz-user-select: none; -ms-user-select: none; user-select: none; background-color: #4CAF50; border: none; color: white; padding: 12px 28px; text-decoration: none; font-size: 26px; margin: 1px; cursor: pointer; }
    .button2 {background-color: #555555;}
  </style>
  <script>
    function moveForward() { fetch('/forward'); }
    function moveLeft() { fetch('/left'); }
    function rotateLeft() { fetch('/rotateleft'); }
    function stopRobot() { fetch('/stop'); }
    function moveRight() { fetch('/right'); }
    function rotateRight() { fetch('/rotateright'); }
    function moveReverse() { fetch('/reverse'); }
    function followLine() { fetch('/followline'); }

    function updateMotorSpeed(pos) {
      document.getElementById('motorSpeed').innerHTML = pos;
      fetch(`/speed?value=${pos}`);
    }

    function updateServoPos(pos) {
      document.getElementById('servoPos').innerHTML = pos;
      fetch(`/servo?value=${pos}`);
    }    

    function switchUltrasonic() { 
      let sta = cb.checked ? 1 : 0;
      document.getElementById('statusUltra').innerHTML = sta;
      fetch('/ultrasonic?value=${sta}');
    }  

    setInterval(() => {
      fetch('/sensor')
        .then(response => response.json())
        .then(data => {
          document.getElementById('distanceValue').innerText = data.distance;
          document.getElementById('rValue').innerText = data.r;
          document.getElementById('gValue').innerText = data.g;
          document.getElementById('bValue').innerText = data.b;
          document.getElementById('cValue').innerText = data.c;
          document.getElementById('colorName').innerText = data.color;          
        })
        .catch(error => console.error('Sensor fetch error:', error));
    }, 200); // every 200 millisecond

  </script>
</head>
<body>
  <h1>UN Robot</h1>
    <p>
      <button class="button" ontouchstart="rotateLeft()" ontouchend="stopRobot()">RL</button>
      <button class="button" ontouchstart="moveForward()" ontouchend="stopRobot()">Fwd</button>
      <button class="button" ontouchstart="rotateRight()" ontouchend="stopRobot()">RR</button>
    </p>
  <div style="clear: both;">
    <p>
      <button class="button" ontouchstart="moveLeft()" ontouchend="stopRobot()">L</button>
      <button class="button button2" ontouchstart="stopRobot()">Stop</button>
      <button class="button" ontouchstart="moveRight()" ontouchend="stopRobot()">R</button>
    </p>
  </div>
  <p><button class="button" ontouchstart="moveReverse()" ontouchend="stopRobot()">Back</button></p>
  <p>Motor Speed: <span id="motorSpeed">0</span></p>
  <input type="range" min="0" max="100" step="25" id="motorSlider" oninput="updateMotorSpeed(this.value)" value="0"/>
  <p>Servo position: <span id="servoPos">90</span></p>
  <input type="range" min="0" max="180" step="15" id="servoSlider" oninput="updateServoPos(this.value)" value="90"/>
  <h3>Sensor Data</h3>
  <p>Distance: <span id="distanceValue">0</span> cm</p>
  <p>Red: <span id="rValue">0</span> | Green: <span id="gValue">0</span> | Blue: <span id="bValue">0</span> | Clear: <span id="cValue">0</span></p>
  <p>Detected Color: <span id="colorName">Unknown</span></p>
  <!-- <p>Ultrasonic: <span id="statusUltra">0</span></p>
  <input type="checkbox" onclick="switchUltrasonic(this)"> -->
  <p><button class="button" onclick="followLine()">Follow Line</button></p>
</body>
</html>)rawliteral";

void handleRoot() {
  server.send(200, "text/html", html);
}

void setMotorPins(int m1p1, int m1p2, int m2p1, int m2p2) {
  digitalWrite(motor1Pin1, m1p1);
  digitalWrite(motor1Pin2, m1p2);
  digitalWrite(motor2Pin1, m2p1);
  digitalWrite(motor2Pin2, m2p2);
}

void handleForward() {
  Serial.println("Forward");
  setMotorPins(LOW, HIGH, LOW, HIGH);
  digitalWrite(onboardLedPin, HIGH); // Turn on LED
  movingForward = true;
  server.send(200);
}

void handleLeft() {
  Serial.println("Left");
  setMotorPins(LOW, LOW, LOW, HIGH);
  digitalWrite(onboardLedPin, HIGH); // Turn on LED
  movingForward = true;
  server.send(200);
}

void handleRotateLeft() {
  Serial.println("Left");
  setMotorPins(HIGH, LOW, LOW, HIGH);
  digitalWrite(onboardLedPin, HIGH); // Turn on LED
  movingForward = false;
  server.send(200);
}

void handleStop() {
  Serial.println("Stop");
  setMotorPins(LOW, LOW, LOW, LOW);
  digitalWrite(onboardLedPin, LOW); // Turn off LED
  movingForward = false;
  followLine=false;
  server.send(200);
}

void handleRight() {
  Serial.println("Right");
  setMotorPins(LOW, HIGH, LOW, LOW);
  digitalWrite(onboardLedPin, HIGH); // Turn on LED
  movingForward = true;  
  server.send(200);
}

void handleRotateRight() {
  Serial.println("Right");
  setMotorPins(LOW, HIGH, HIGH, LOW);
  digitalWrite(onboardLedPin, HIGH); // Turn on LED
  movingForward = false;
  server.send(200);
}

void handleReverse() {
  Serial.println("Reverse");
  setMotorPins(HIGH, LOW, HIGH, LOW);
  digitalWrite(onboardLedPin, HIGH); // Turn on LED
  movingForward = false;
  server.send(200);
}

void handleSpeed() {
  if (server.hasArg("value")) {
    valueString = server.arg("value");
    int value = valueString.toInt();
    dutyCycle = map(value, 25, 100, 200, 255);
    ledcWrite(enable1Pin, dutyCycle);
    ledcWrite(enable2Pin, dutyCycle);
    Serial.printf("Motor speed set to %d\n", value);
  }
  server.send(200);
}

void handleServo() {
  if (server.hasArg("value")) {
    valueString = server.arg("value");
    int value = valueString.toInt();
    servo1.write(value);
    Serial.printf("Servo set to %d\n", value);
  }
  server.send(200);
}

void handleUltrasonic() {
  if (server.hasArg("value")) {
    valueString = server.arg("value");
    int value = valueString.toInt();

    Serial.printf("Ultrasonic set to %d\n", value);
  }
  server.send(200);
}

void handleFollowLine() {
  Serial.println("Start Follow Line");
  setMotorPins(LOW, HIGH, LOW, HIGH);
  digitalWrite(onboardLedPin, HIGH); // Turn on LED
  movingForward = true;
  followLine = true;
  server.send(200);
}

void handleSensorData() {
  String colorName = detectColor(r, g, b, c);  
  // Build JSON
  String json = "{";
  json += "\"r\":" + String(r) + ",";
  json += "\"g\":" + String(g) + ",";
  json += "\"b\":" + String(b) + ",";
  json += "\"c\":" + String(c) + ",";
  json += "\"distance\":" + String(distanceCm, 2) + ",";
  json += "\"color\":\"" + colorName + "\"";
  json += "}";

  server.send(200, "application/json", json);
}


String detectColor(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  if (c < 50) return "Black";
  if (c > 600 && r > 170 && g > 170 && b > 170) return "White";

  if (r > g && r > b) {
    if (g > b) return "Yellow"; // Red + Green = Yellow
    return "Red";
  }
  if (g > r && g > b) return "Green";
  if (b > r && b > g) return "Blue";

  return "Unknown";
}




void setup() {
  Serial.begin(115200);

  // Set the Motor pins as outputs
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  // Configure PWM Pins
  ledcAttach(enable1Pin, freq, resolution);
  ledcAttach(enable2Pin, freq, resolution);
    
  // Initialize PWM with 0 duty cycle
  ledcWrite(enable1Pin, 0);
  ledcWrite(enable2Pin, 0);

  // Set Ultrasonic Pins
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  // Initialize onboard LED pin
  pinMode(onboardLedPin, OUTPUT);
  digitalWrite(onboardLedPin, LOW); // Ensure LED is off initially

  // Initialize Servo Pin
  servo1.attach(servoPin);
  servo1.write(90);

  // Initialize TCS34725 sensor
  if (!tcs.begin()) {
    Serial.println("Error: TCS34725 not detected. Check wiring!");
    while (1);
  }
  Serial.println("TCS34725 sensor detected!");

  //calibrateSensor();

  // Setup Access Point
  Serial.println("Setting AP (Access Point)...");
  if (!WiFi.softAP(ssid, password)) {
    Serial.println("Failed to setup AP");
    while (1);
  }

  IPAddress IP = WiFi.softAPIP();
  Serial.printf("AP IP address: %s\n", IP.toString().c_str());

  // Define routes
  server.on("/", handleRoot);
  server.on("/forward", handleForward);
  server.on("/left", handleLeft);
  server.on("/rotateleft", handleRotateLeft);
  server.on("/stop", handleStop);
  server.on("/right", handleRight);
  server.on("/rotateright", handleRotateRight);
  server.on("/reverse", handleReverse);
  server.on("/speed", handleSpeed);
  server.on("/servo", handleServo);
  server.on("/ultrasonic", handleUltrasonic);
  server.on("/followline", handleFollowLine);
  server.on("/sensor", handleSensorData);


  // Start the server
  server.begin();
}

void loop() {
  server.handleClient();

  // Variables for Color Sensor

  // Get Color Sensor Data
  tcs.getRawData(&r, &g, &b, &c);

  // Output the sensor data to the serial console
/*  Serial.print("Red: "); 
  Serial.print(r); 
  Serial.print(" | Green: "); 
  Serial.print(g);
  Serial.print(" | Blue: "); 
  Serial.print(b); 
  Serial.print(" | Clear: "); 
  Serial.println(c);
*/
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance
  distanceCm = duration * SOUND_SPEED/2;
  if ((distanceCm<10)&&movingForward) { handleStop(); }

  // Prints the distance in the Serial Monitor
 // Serial.print("Distance (cm): ");
 // Serial.println(distanceCm);

  if (followLine) {
    // Calculate error
    error = middleValue - (float)c ;
 
    // PID calculation
    sumError += error;
    sumError = constrain(sumError, -4, 4); // Limit integral component
 
    if (abs(error) < 5.0) {
      sumError = 0; // Reset for small errors
    }
 
    float differential = error - previousError;
    float correction = Kp * error + Ki * sumError + Kd * differential;
    previousError = error;
 
    // Adjust motor speeds
    int speedLeft = baseSpeed + (int)correction;
    int speedRight = baseSpeed - (int)correction;
 
    speedLeft = constrain(speedLeft, 20, 255);
    speedRight = constrain(speedRight, 20, 255);
    // dutyCycleLeft = map(speedLeft, 25, 100, 200, 255);
    // dutyCycleRight = map(speedRight, 25, 100, 200, 255);
    ledcWrite(enable1Pin, speedLeft);
    ledcWrite(enable2Pin, speedRight);
  
  }




  delay(50);
}