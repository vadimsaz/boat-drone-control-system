#include <Servo.h>

// Пины подключения
#define MOTOR_ESC_PIN 9        // ESC для мотора
#define RUDDER_SERVO_PIN 10    // Сервопривод руля
#define EMERGENCY_STOP_PIN 2   // Кнопка экстренной остановки
#define LED_STATUS_PIN 13      // Светодиод статуса
#define BATTERY_MONITOR_PIN A0 // Мониторинг батареи

// Объекты управления
Servo motorESC;
Servo rudderServo;

// Переменные состояния
int currentMotorSpeed = 0;    // Текущая скорость мотора (0-100)
int currentRudderAngle = 90;  // Текущий угол руля (45-135, 90=прямо)
bool emergencyStop = false;
bool systemArmed = false;

// Буфер для команд
String inputBuffer = "";
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 5000; // 5 секунд таймаут

// Калибровочные значения для ESC
const int ESC_MIN = 1000;  // Минимальный сигнал для ESC
const int ESC_MAX = 2000;  // Максимальный сигнал для ESC
const int ESC_STOP = 1500; // Стоп сигнал для ESC

void setup() {
  Serial.begin(9600);
  
  // Инициализация пинов
  pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);
  pinMode(LED_STATUS_PIN, OUTPUT);
  pinMode(BATTERY_MONITOR_PIN, INPUT);
  
  // Инициализация сервоприводов
  motorESC.attach(MOTOR_ESC_PIN);
  rudderServo.attach(RUDDER_SERVO_PIN);
  
  // Калибровка ESC
  calibrateESC();
  
  // Установка начального положения
  setMotorSpeed(0);
  setRudderAngle(0);
  
  // Прерывание для экстренной остановки
  attachInterrupt(digitalPinToInterrupt(EMERGENCY_STOP_PIN), emergencyStopISR, FALLING);
  
  Serial.println("Arduino Boat Controller v1.0");
  Serial.println("Система готова к работе");
  
  blinkStatusLED(3); // 3 мигания - система готова
}

void loop() {
  // Проверка экстренной остановки
  checkEmergencyStop();
  
  // Проверка таймаута команд
  checkCommandTimeout();
  
  // Чтение команд с Serial
  readSerialCommands();
  
  // Мониторинг системы
  monitorSystem();
  
  // Небольшая задержка
  delay(50);
}

void calibrateESC() {
  Serial.println("Калибровка ESC...");
  
  // Максимальный сигнал
  motorESC.writeMicroseconds(ESC_MAX);
  delay(2000);
  
  // Минимальный сигнал
  motorESC.writeMicroseconds(ESC_MIN);
  delay(2000);
  
  // Нейтральное положение
  motorESC.writeMicroseconds(ESC_STOP);
  delay(1000);
  
  Serial.println("Калибровка ESC завершена");
}

void readSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n') {
      // Обработка полученной команды
      processCommand(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }
}

void processCommand(String command) {
  command.trim();
  lastCommandTime = millis();
  
  if (command.startsWith("MOTOR:")) {
    int speed = command.substring(6).toInt();
    setMotorSpeed(speed);
    Serial.println("OK:MOTOR:" + String(speed));
    
  } else if (command.startsWith("RUDDER:")) {
    int angle = command.substring(7).toInt();
    setRudderAngle(angle);
    Serial.println("OK:RUDDER:" + String(angle));
    
  } else if (command == "ARM") {
    systemArmed = true;
    Serial.println("OK:ARMED");
    blinkStatusLED(1);
    
  } else if (command == "DISARM") {
    systemArmed = false;
    setMotorSpeed(0);
    setRudderAngle(0);
    Serial.println("OK:DISARMED");
    blinkStatusLED(2);
    
  } else if (command == "STATUS") {
    sendStatus();
    
  } else if (command == "PING") {
    Serial.println("PONG");
    
  } else {
    Serial.println("ERROR:UNKNOWN_COMMAND:" + command);
  }
}

void setMotorSpeed(int speed) {
  if (emergencyStop || !systemArmed) {
    speed = 0;
  }
  
  // Ограничение скорости
  speed = constrain(speed, 0, 100);
  
  // Преобразование в микросекунды для ESC
  int microseconds;
  if (speed == 0) {
    microseconds = ESC_STOP; // Стоп
  } else {
    // Преобразование 1-100% в диапазон ESC_STOP - ESC_MAX
    microseconds = map(speed, 1, 100, ESC_STOP + 50, ESC_MAX);
  }
  
  motorESC.writeMicroseconds(microseconds);
  currentMotorSpeed = speed;
  
  Serial.println("MOTOR_SET:" + String(speed));
}

void setRudderAngle(int angle) {
  if (emergencyStop) {
    angle = 0; // Прямо при экстренной остановке
  }
  
  // Ограничение угла руля (-45 до +45 градусов)
  angle = constrain(angle, -45, 45);
  
  // Преобразование в угол сервопривода (90 = прямо)
  int servoAngle = 90 + angle;
  
  rudderServo.write(servoAngle);
  currentRudderAngle = angle;
  
  Serial.println("RUDDER_SET:" + String(angle));
}

void checkEmergencyStop() {
  bool stopPressed = !digitalRead(EMERGENCY_STOP_PIN);
  
  if (stopPressed && !emergencyStop) {
    emergencyStop = true;
    setMotorSpeed(0);
    setRudderAngle(0);
    systemArmed = false;
    
    Serial.println("EMERGENCY_STOP_ACTIVATED");
    
    // Мигание красным светодиодом
    for (int i = 0; i < 10; i++) {
      digitalWrite(LED_STATUS_PIN, HIGH);
      delay(100);
      digitalWrite(LED_STATUS_PIN, LOW);
      delay(100);
    }
  }
  
  if (!stopPressed && emergencyStop) {
    emergencyStop = false;
    Serial.println("EMERGENCY_STOP_DEACTIVATED");
  }
}

void emergencyStopISR() {
  // Немедленная остановка мотора
  motorESC.writeMicroseconds(ESC_STOP);
}

void checkCommandTimeout() {
  if (systemArmed && (millis() - lastCommandTime > COMMAND_TIMEOUT)) {
    // Таймаут команд - аварийная остановка
    setMotorSpeed(0);
    setRudderAngle(0);
    systemArmed = false;
    
    Serial.println("WARNING:COMMAND_TIMEOUT");
    blinkStatusLED(5);
  }
}

void monitorSystem() {
  static unsigned long lastMonitor = 0;
  
  if (millis() - lastMonitor > 1000) { // Каждую секунду
    // Мониторинг напряжения батареи
    int batteryRaw = analogRead(BATTERY_MONITOR_PIN);
    float batteryVoltage = (batteryRaw * 5.0 / 1023.0) * 3.3; // Делитель напряжения
    
    if (batteryVoltage < 10.5) { // Низкий заряд для 3S LiPo
      Serial.println("WARNING:LOW_BATTERY:" + String(batteryVoltage));
      
      // Принудительное снижение мощности
      if (currentMotorSpeed > 50) {
        setMotorSpeed(50);
      }
    }
    
    // Индикация статуса
    if (systemArmed) {
      digitalWrite(LED_STATUS_PIN, HIGH);
    } else {
      // Медленное мигание
      static bool ledState = false;
      ledState = !ledState;
      digitalWrite(LED_STATUS_PIN, ledState);
    }
    
    lastMonitor = millis();
  }
}

void sendStatus() {
  Serial.println("STATUS_REPORT:");
  Serial.println("  Motor: " + String(currentMotorSpeed) + "%");
  Serial.println("  Rudder: " + String(currentRudderAngle) + "°");
  Serial.println("  Armed: " + String(systemArmed ? "YES" : "NO"));
  Serial.println("  Emergency: " + String(emergencyStop ? "YES" : "NO"));
  
  // Напряжение батареи
  int batteryRaw = analogRead(BATTERY_MONITOR_PIN);
  float batteryVoltage = (batteryRaw * 5.0 / 1023.0) * 3.3;
  Serial.println("  Battery: " + String(batteryVoltage) + "V");
  
  Serial.println("STATUS_END");
}

void blinkStatusLED(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_STATUS_PIN, HIGH);
    delay(200);
    digitalWrite(LED_STATUS_PIN, LOW);
    delay(200);
  }
}

// Дополнительные функции безопасности
void performSelfTest() {
  Serial.println("Запуск самодиагностики...");
  
  // Тест сервопривода руля
  Serial.println("Тест руля...");
  setRudderAngle(-30);
  delay(1000);
  setRudderAngle(30);
  delay(1000);
  setRudderAngle(0);
  
  // Тест ESC (без запуска мотора)
  Serial.println("Тест ESC...");
  motorESC.writeMicroseconds(ESC_STOP);
  delay(500);
  
  Serial.println("Самодиагностика завершена");
}