#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define S1 32 
#define S2 5   
#define S3 18 
#define S4 33 
#define AIN1 27
#define AIN2 26 
#define BIN1 22 
#define BIN2 23 

#define st 4 

BluetoothSerial SerialBT;

float Kp = 150;  
float Ki = 0.01;
float Kd = 5;
int baseSpeed = 140;
float error = 0;
float integral = 0;
float previousError = 0;
const float dt = 0.01;

uint8_t multiP = 0; 
uint8_t multiI = 2;  
uint8_t multiD = 0;
boolean onoff = false;

int val, cnt = 0, v[3];

void setup() {
  pinMode(S1, INPUT_PULLUP);
  pinMode(S2, INPUT_PULLUP);
  pinMode(S3, INPUT_PULLUP);
  pinMode(S4, INPUT_PULLUP);
  
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(st, OUTPUT);

  digitalWrite(st, HIGH);

  analogWrite(AIN1, 0);
  analogWrite(AIN2, 0);
  analogWrite(BIN1, 0);
  analogWrite(BIN2, 0);

  Serial.begin(115200);
  SerialBT.begin("TerminatorNeo");  // Bluetooth device name
  Serial.println("Bluetooth Started! Ready to pair...");
}

void loop() {
  // Check for Bluetooth data (at least 2 bytes for command + value)
  if (SerialBT.available() >= 2) {
    valuesread();
    valuesread();
    processing();
  }
  
  if (onoff == true) {
    int r1 = digitalRead(S1); 
    int r2 = digitalRead(S2); 
    int r3 = digitalRead(S3); 
    int r4 = digitalRead(S4);
    
    int sum = (r1 == LOW) + (r2 == LOW) + (r3 == LOW) + (r4 == LOW);
   
    if (sum > 0) {
      // Symmetric positions for better centering
      int pos = ((r1 == LOW ? -1500 : 0) + (r2 == LOW ? -500 : 0) + 
                 (r3 == LOW ? 500 : 0) + (r4 == LOW ? 1500 : 0)) / sum;
      error = pos;  // Target is 0
      integral += error * dt; 
      integral = constrain(integral, -100, 100);  // Anti-windup
      float derivative = (error - previousError) / dt; 
      previousError = error; 
      
      Serial.printf("Error: %.0f, Integral: %.2f\n", error, integral);
      
      // Use scaled PID gains
      float scaledKp = Kp / pow(10, multiP);
      float scaledKi = Ki / pow(10, multiI);
      float scaledKd = Kd / pow(10, multiD);
      int speedAdjustment = scaledKp * error + scaledKi * integral + scaledKd * derivative;
      
      int leftSpeed = baseSpeed - speedAdjustment;
      int rightSpeed = baseSpeed + speedAdjustment;
      
      leftSpeed = constrain(leftSpeed, 0, 255);
      rightSpeed = constrain(rightSpeed, 0, 255);
      
      analogWrite(AIN1, leftSpeed);
      analogWrite(AIN2, 0);         
      analogWrite(BIN1, rightSpeed); 
      analogWrite(BIN2, 0);        
      
      Serial.printf("Left Speed: %d, Right Speed: %d\n", leftSpeed, rightSpeed);
    } else {
      integral = 0;
      analogWrite(AIN1, 0);
      analogWrite(AIN2, 0);
      analogWrite(BIN1, 0);
      analogWrite(BIN2, 0);
      Serial.println("No line detected, stopping motors");
    }
  } else {
    analogWrite(AIN1, 0);
    analogWrite(AIN2, 0);
    analogWrite(BIN1, 0);
    analogWrite(BIN2, 0);
  }
  
  delay(10);
}

void valuesread() {
  val = SerialBT.read();
  cnt++;
  v[cnt] = val;
  if (cnt == 2) cnt = 0;
}

void processing() {
  int a = v[1];  // Command byte
  if (a == 1) Kp = v[2];
  if (a == 2) multiP = v[2];
  if (a == 3) Ki = v[2];
  if (a == 4) multiI = v[2];
  if (a == 5) Kd = v[2];
  if (a == 6) multiD = v[2];
  if (a == 7) onoff = v[2];
}#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define S1 32 
#define S2 5   
#define S3 18 
#define S4 33 
#define AIN1 27
#define AIN2 26 
#define BIN1 22 
#define BIN2 23 

#define st 4 

BluetoothSerial SerialBT;

float Kp = 150;  
float Ki = 0.01;
float Kd = 5;
int baseSpeed = 140;
float error = 0;
float integral = 0;
float previousError = 0;
const float dt = 0.01;

uint8_t multiP = 0; 
uint8_t multiI = 2;  
uint8_t multiD = 0;
boolean onoff = false;

int val, cnt = 0, v[3];

void setup() {
  pinMode(S1, INPUT_PULLUP);
  pinMode(S2, INPUT_PULLUP);
  pinMode(S3, INPUT_PULLUP);
  pinMode(S4, INPUT_PULLUP);
  
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(st, OUTPUT);

  digitalWrite(st, HIGH);

  analogWrite(AIN1, 0);
  analogWrite(AIN2, 0);
  analogWrite(BIN1, 0);
  analogWrite(BIN2, 0);

  Serial.begin(115200);
  SerialBT.begin("TerminatorNeo");  // Bluetooth device name
  Serial.println("Bluetooth Started! Ready to pair...");
}

void loop() {
  // Check for Bluetooth data (at least 2 bytes for command + value)
  if (SerialBT.available() >= 2) {
    valuesread();
    valuesread();
    processing();
  }
  
  if (onoff == true) {
    int r1 = digitalRead(S1); 
    int r2 = digitalRead(S2); 
    int r3 = digitalRead(S3); 
    int r4 = digitalRead(S4);
    
    int sum = (r1 == LOW) + (r2 == LOW) + (r3 == LOW) + (r4 == LOW);
   
    if (sum > 0) {
      // Symmetric positions for better centering
      int pos = ((r1 == LOW ? -1500 : 0) + (r2 == LOW ? -500 : 0) + 
                 (r3 == LOW ? 500 : 0) + (r4 == LOW ? 1500 : 0)) / sum;
      error = pos;  // Target is 0
      integral += error * dt; 
      integral = constrain(integral, -100, 100);  // Anti-windup
      float derivative = (error - previousError) / dt; 
      previousError = error; 
      
      Serial.printf("Error: %.0f, Integral: %.2f\n", error, integral);
      
      // Use scaled PID gains
      float scaledKp = Kp / pow(10, multiP);
      float scaledKi = Ki / pow(10, multiI);
      float scaledKd = Kd / pow(10, multiD);
      int speedAdjustment = scaledKp * error + scaledKi * integral + scaledKd * derivative;
      
      int leftSpeed = baseSpeed - speedAdjustment;
      int rightSpeed = baseSpeed + speedAdjustment;
      
      leftSpeed = constrain(leftSpeed, 0, 255);
      rightSpeed = constrain(rightSpeed, 0, 255);
      
      analogWrite(AIN1, leftSpeed);
      analogWrite(AIN2, 0);         
      analogWrite(BIN1, rightSpeed); 
      analogWrite(BIN2, 0);        
      
      Serial.printf("Left Speed: %d, Right Speed: %d\n", leftSpeed, rightSpeed);
    } else {
      integral = 0;
      analogWrite(AIN1, 0);
      analogWrite(AIN2, 0);
      analogWrite(BIN1, 0);
      analogWrite(BIN2, 0);
      Serial.println("No line detected, stopping motors");
    }
  } else {
    analogWrite(AIN1, 0);
    analogWrite(AIN2, 0);
    analogWrite(BIN1, 0);
    analogWrite(BIN2, 0);
  }
  
  delay(10);
}

void valuesread() {
  val = SerialBT.read();
  cnt++;
  v[cnt] = val;
  if (cnt == 2) cnt = 0;
}

void processing() {
  int a = v[1];  // Command byte
  if (a == 1) Kp = v[2];
  if (a == 2) multiP = v[2];
  if (a == 3) Ki = v[2];
  if (a == 4) multiI = v[2];
  if (a == 5) Kd = v[2];
  if (a == 6) multiD = v[2];
  if (a == 7) onoff = v[2];
}
