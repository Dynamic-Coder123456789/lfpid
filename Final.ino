#define S1 19
#define S2 5
#define S3 18
#define S4 33
#define S5 2

#define AIN1 27
#define AIN2 26
#define BIN1 22
#define BIN2 23

#define st 4

const float Kp = 300;
const int baseSpeed = 175;

float error = 0;
int leftSpeed = 0, rightSpeed = 0;

TaskHandle_t TaskSensors;
TaskHandle_t TaskMotors;

void setup() {
  pinMode(S1, INPUT_PULLUP);
  pinMode(S2, INPUT_PULLUP);
  pinMode(S3, INPUT_PULLUP);
  pinMode(S4, INPUT_PULLUP);
  pinMode(S5, INPUT_PULLUP);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(st, OUTPUT);

  digitalWrite(st, HIGH);
  Serial.begin(115200);

  xTaskCreatePinnedToCore(
    readSensorsTask,   // Task function
    "TaskSensors",     // Task name
    4096,              // Stack size
    NULL,              // Parameters
    1,                 // Priority
    &TaskSensors,      // Task handle
    0                  // Core 0
  );

  xTaskCreatePinnedToCore(
    controlMotorsTask,
    "TaskMotors",
    4096,
    NULL,
    1,
    &TaskMotors,
    1                  // Core 1
  );
}

void readSensorsTask(void *parameter) {
  for (;;) {
    int r1 = digitalRead(S1);
    int r2 = digitalRead(S2);
    int r3 = digitalRead(S3);
    int r4 = digitalRead(S4);
    int r5 = digitalRead(S5);

    // Count active sensors (black detected)
    int sum = (r1 == LOW) + (r2 == LOW) + (r3 == LOW) + (r4 == LOW) + (r5 == LOW);

    if (sum > 0) {
      // Weights → S1 = 0, S2 = 750, S3 = 1500, S4 = 2250, S5 = 3000
      int pos = ((r1 == LOW ? 0    : 0) +
                 (r2 == LOW ? 750  : 0) +
                 (r3 == LOW ? 1500 : 0) +
                 (r4 == LOW ? 2250 : 0) +
                 (r5 == LOW ? 3000 : 0)) / sum;

      // Error = desired center (1500) - current pos
      error = pos - 1500;
    } else {
      // If no sensor sees black, send a special error
      error = 9999;
    }

    Serial.printf("[Sensors] r1:%d r2:%d r3:%d r4:%d r5:%d -> Error: %.0f\n",
                  r1, r2, r3, r4, r5, error);

    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}


void controlMotorsTask(void *parameter) {
  for (;;) {
    if (error == 9999) {
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

      // 🔹 Added this line to print error value for debugging
      Serial.printf("[Motors] Error: %.0f | L:%d | R:%d\n", error, leftSpeed, rightSpeed);
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void loop() {
  // Nothing here, tasks handle everything
}
