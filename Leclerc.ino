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

// PID constants — tune slightly if needed
float Kp = 300;
float Ki = 0.0005;
float Kd = 0.18;

const int baseSpeed = 150;
float error = 0, lastError = 0, integral = 0;

int leftSpeed = 0, rightSpeed = 0;
int lastDirection = 0; // -1 = left, 1 = right

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

  xTaskCreatePinnedToCore(readSensorsTask, "TaskSensors", 4096, NULL, 1, &TaskSensors, 0);
  xTaskCreatePinnedToCore(controlMotorsTask, "TaskMotors", 4096, NULL, 1, &TaskMotors, 1);
}

void readSensorsTask(void *parameter) {
  for (;;) {
    int r1 = digitalRead(S1);
    int r2 = digitalRead(S2);
    int r3 = digitalRead(S3);
    int r4 = digitalRead(S4);
    int r5 = digitalRead(S5);

    int sum = (r1 == LOW) + (r2 == LOW) + (r3 == LOW) + (r4 == LOW) + (r5 == LOW);

    if (sum == 5) {
        // All black → gently drift right
        error = 10000;      
        lastDirection = 1;
    } 
    else if (sum > 0) {
        int pos = ((r1 == LOW ? -20000 : 0) +
                   (r2 == LOW ? -2000  : 0) +
                   (r3 == LOW ? 0      : 0) +
                   (r4 == LOW ? 2000   : 0) +
                   (r5 == LOW ? 20000  : 0)) / sum;

        error = pos;

        if (pos < 0) lastDirection = -1;
        else if (pos > 0) lastDirection = 1;
    } 
    else {
        // All white → stop the motors instead of spinning
        error = 0;
        leftSpeed = 0;
        rightSpeed = 0;
    }

    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

void controlMotorsTask(void *parameter) {
  for (;;) {
    float derivative = error - lastError;
    integral += error;

    float output = Kp * error + Ki * integral + Kd * derivative;
    lastError = error;

    leftSpeed = constrain(baseSpeed - output, 0, 255);
    rightSpeed = constrain(baseSpeed + output, 0, 255);

    analogWrite(AIN1, leftSpeed);
    analogWrite(AIN2, 0);
    analogWrite(BIN1, rightSpeed);
    analogWrite(BIN2, 0);

    Serial.printf("[PID] E: %.0f | L:%d | R:%d\n", error, leftSpeed, rightSpeed);

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void loop() {
  // Empty — tasks handle everything
}