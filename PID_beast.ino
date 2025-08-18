#define S1 32 
#define S2 5 
#define S3 18 
#define S4 33 

#define AIN1 27  
#define AIN2 26 
#define BIN1 22 
#define BIN2 23 

#define st 4

const float Kp = 25; 
const int baseSpeed = 160;

float error = 0;   // shared between tasks
int leftSpeed = 0, rightSpeed = 0;  

// Task handles
TaskHandle_t TaskSensors;
TaskHandle_t TaskMotors;

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

  Serial.begin(115200);

  // Create tasks
  xTaskCreatePinnedToCore(
    readSensorsTask,   
    "TaskSensors",     
    4096,              
    NULL,              
    1,                 
    &TaskSensors,      
    0                  
  );

  xTaskCreatePinnedToCore(
    controlMotorsTask, 
    "TaskMotors", 
    4096, 
    NULL, 
    1, 
    &TaskMotors, 
    1                  
  );
}

void readSensorsTask(void * parameter) {
  for(;;) {
    int r1 = digitalRead(S1); 
    int r2 = digitalRead(S2);
    int r3 = digitalRead(S3); 
    int r4 = digitalRead(S4); 
    
    // same logic as your code
    int sum = (r1 == LOW) + (r2 == LOW) + (r3 == LOW) + (r4 == LOW);
    
    if (sum > 0) {
      int pos = ((r1 == LOW ? 0 : 0) + 
                 (r2 == LOW ? 1000 : 0) + 
                 (r3 == LOW ? 2000 : 0) + 
                 (r4 == LOW ? 3000 : 0)) / sum;
      error = pos - 1500;
    } else {
      // stop case same as your code
      error = 9999;  // special marker for "no line"
    }

    Serial.printf("[Sensors] r1:%d r2:%d r3:%d r4:%d -> Error: %.0f\n", r1, r2, r3, r4, error);
    vTaskDelay(5 / portTICK_PERIOD_MS);  
  }
}

void controlMotorsTask(void * parameter) {
  for(;;) {
    if (error == 9999) {
      // no line -> stop
      analogWrite(AIN1, 0);
      analogWrite(AIN2, 0);
      analogWrite(BIN1, 0);
      analogWrite(BIN2, 0);
      Serial.println("[Motors] No line detected, stopping motors");
    } else {
      int speedAdjustment = Kp * error; 
      leftSpeed = baseSpeed - speedAdjustment;
      rightSpeed = baseSpeed + speedAdjustment;

      leftSpeed = constrain(leftSpeed, 0, 255);
      rightSpeed = constrain(rightSpeed, 0, 255);

      analogWrite(AIN1, leftSpeed); 
      analogWrite(AIN2, 0);  
      analogWrite(BIN1, rightSpeed); 
      analogWrite(BIN2, 0);  

      Serial.printf("[Motors] L:%d R:%d\n", leftSpeed, rightSpeed);
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);  
  }
}

void loop() {
  // Nah id win
}
