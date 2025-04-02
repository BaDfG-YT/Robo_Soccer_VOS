#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <FastLED.h>
// #include <MPU6050.h>
#include <MPU6050_light.h>
// #include <Gyro.h>
#include <Pixy2I2C.h>
#include <math.h>
#include <Servo.h>

// Настройки OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Настройки энкодера и кнопки
#define ENCODER_PIN_A 15
#define ENCODER_PIN_B 14
#define BUTTON_PIN 16

// Меню
const char *menuItems[] = {
    "Motors", "MoveBall", "BallGyro", "Buzzer", "Ring", "IR_s",
    "Gyroscope", "gyro_norm", "Gates", "gates_norm", "ball_gates",
    "Dribble", "dribble_goal", "kick_test", "dribble_kick", "disharge_kck"};
const int menuLength = sizeof(menuItems) / sizeof(menuItems[0]);
const int itemsPerPage = 6; // Количество пунктов на одной странице
int menuIndex = 0;          // Индекс текущего элемента меню
int pageIndex = 0;          // Индекс текущей страницы
bool selected = false;      // Флаг выбора режима

// Энкодер
int lastEncoded = 0;
long encoderValue = 0;
long lastReportedValue = 0;

// Переменные для отслеживания состояния кнопки
bool lastButtonState = HIGH;
bool buttonPressed = false;

// Настройки для WS2812B
#define LED_PIN 20
#define NUM_LEDS 32
#define BRIGHTNESS 35
CRGB leds[NUM_LEDS];

// Массив цветов
CRGB colors[] = {
    CRGB::Red,
    CRGB::Orange,
    CRGB::Yellow,
    CRGB::Green,
    CRGB::Blue,
    CRGB::Purple,
    CRGB::Pink,
    CRGB::White};

// Переменные для перелива
uint8_t hueOffset = 0;                 // Смещение для цветов
unsigned long lastRingUpdate = 0;      // Время последнего обновления кольца
const unsigned long ringInterval = 50; // Интервал обновления кольца (в мс)

// Настройки для пьезопищалки
#define BUZZER_PIN 17
int melody[] = {262, 294, 330, 349, 392};
int noteDurations[] = {4, 4, 4, 4, 4};

// Структура для хранения данных от IR Locator
struct IRSensorData
{
  int angle;
  int strength;
};
#define smoothing_factor 0.3

int bufferIndex = 0;
// I2C адрес ИК-локатора
const int irLocatorAddress = 0x0E; // 7-битный адрес

// // MPU6050 settings
MPU6050 mpu(Wire);

// Pixy2
Pixy2I2C pixy;
// Определяем сигнатуры
const int SIGNATURE_1 = 1; // Первая сигнатура
const int SIGNATURE_2 = 2; // Вторая сигнатура
// Глобальные переменные для хранения информации о сигнатурах
float avgX1, avgY1, totalArea1, angle1;
float avgX2, avgY2, totalArea2, angle2;

int u = 0;
float k = 0.2;
float k_gates = 10;
bool f_front = 0; // Флаг наличия сигнатуры 1

// Дрибблинг
Servo dribble;
#define MOTOR_PIN 18
// Флаг для управления движением
bool isMoving = false;
// Параметры управления скоростью
int motorSpeed = 1000;     // Начальная скорость (в микросекундах)
const int minSpeed = 1000; // Минимальная скорость
const int maxSpeed = 2000; // Максимальная скорость
const int stepSize = 10;   // Шаг изменения скорости

// Пины управления двигателями
const int pinA1 = 8;  // Пин для управления направлением
const int pinA2 = 9;  // Пин для управления скоростью (PWM)
const int pinB1 = 10; // Пин для управления направлением
const int pinB2 = 11; // Пин для управления скоростью (PWM)
const int pinC1 = 6;  // Пин для управления направлением
const int pinC2 = 7;  // Пин для управления скоростью (PWM)
const int pinD1 = 12; // Пин для управления направлением
const int pinD2 = 13; // Пин для управления скоростью (PWM)

int speed = 95;

void drawMenu()
{
  display.clearDisplay();
  display.setTextSize(1);

  int startIdx = pageIndex * itemsPerPage;               // Первый элемент текущей страницы
  int endIdx = min(startIdx + itemsPerPage, menuLength); // Последний элемент текущей страницы

  for (int i = startIdx; i < endIdx; i++)
  {
    if (i == menuIndex)
    {
      display.setTextColor(BLACK, WHITE); // Активный пункт
    }
    else
    {
      display.setTextColor(WHITE, BLACK);
    }
    display.setCursor(0, (i - startIdx) * 10); // Смещение внутри текущей страницы
    display.print(i + 1);
    display.print(". ");
    display.println(menuItems[i]);
  }

  // Отображение номера страницы
  display.setTextColor(WHITE, BLACK);
  display.setCursor(100, 55);
  display.print("Pg ");
  display.print(pageIndex + 1);
  display.print("/");
  display.print((menuLength + itemsPerPage - 1) / itemsPerPage); // Количество страниц

  display.display();
}

int gyro_angle()
{
  mpu.update();
  int a = mpu.getAngleZ();

  // Приводим угол к диапазону 0-360
  a = a % 360; // Остаток от деления на 360
  if (a < 0)
  {
    a += 360; // Если угол отрицательный, добавляем 360
  }

  return a;
}



void calibrateMPU()
{
  Serial.println("Calibrating MPU6050...");
  for (int i = 0; i < 200; i++)
  { // Считываем 200 раз для усреднения
    mpu.update();
    delay(5);
  }
  mpu.calcGyroOffsets(); // Выполняем калибровку гироскопа
  Serial.println("Calibration complete!");
}

void setup()
{
  // Инициализация дисплея
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    while (1)
      ; // Бесконечный цикл в случае ошибки
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  // Настройка пинов энкодера и кнопки
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  pinMode(19, INPUT_PULLUP); // Настраиваем кнопку на пине 19

  drawMenu(); // Отображение меню

  pinMode(21, 1);

  // Инициализация светодиодного кольца
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  FastLED.show();

  Serial.println("Initializing Pixy2...");
  pixy.init();
  if (pixy.version->hardware == 0)
  {
    Serial.println("Pixy2 initialization failed. Check connections!");
  }
  else
  {
    Serial.println("Pixy2 initialized successfully.");
  }

  // Инициализация мотора
  dribble.attach(MOTOR_PIN);
  dribble.writeMicroseconds(motorSpeed);

  // Настройка пинов моторов
  pinMode(pinA1, 1);
  pinMode(pinB1, 1);
  pinMode(pinC1, 1);
  pinMode(pinD1, 1);

  pinMode(pinA2, 1);
  pinMode(pinB2, 1);
  pinMode(pinC2, 1);
  pinMode(pinD2, 1);

  mpu.begin();
  mpu.calcGyroOffsets();

  calibrateMPU();

  Serial.begin(9600);
}

// Функция отображения выбранного режима
void displayMode(const char *mode)
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextColor(WHITE, BLACK);
  display.println("Selected Mode:");
  display.println(mode);
  display.display();
}

void updatePage()
{
  pageIndex = menuIndex / itemsPerPage; // Рассчитываем, какая страница сейчас активна
  drawMenu();                           // Перерисовываем меню
}

// Функция для установки цвета светодиода
void setLEDColor(int index, uint8_t red, uint8_t green, uint8_t blue)
{
  if (index >= 0 && index < NUM_LEDS)
  {
    leds[index] = CRGB(red, green, blue); // Устанавливаем цвет светодиода
    FastLED.show();                       // Обновляем данные светодиодов
  }
}

// Функция обработки данных с Pixy2
void processPixyData()
{
  pixy.ccc.getBlocks(); // Получаем данные о блоках

  // Очистка светодиодного кольца перед обновлением
  FastLED.clear();

  // Обнуление переменных
  int sumX1 = 0, sumY1 = 0, count1 = 0;
  int sumX2 = 0, sumY2 = 0, count2 = 0;
  totalArea1 = 0;
  totalArea2 = 0;
  angle1 = 0;
  angle2 = 0;
  f_front = false; // Сбрасываем флаг

  if (pixy.ccc.numBlocks)
  {
    Serial.print("Objects detected: ");
    Serial.println(pixy.ccc.numBlocks);

    for (int i = 0; i < pixy.ccc.numBlocks; i++)
    {
      int signature = pixy.ccc.blocks[i].m_signature;
      int x = pixy.ccc.blocks[i].m_x - 160; // Смещение центра в (0,0)
      int y = 120 - pixy.ccc.blocks[i].m_y; // Инверсия оси Y
      int width = pixy.ccc.blocks[i].m_width;
      int height = pixy.ccc.blocks[i].m_height;
      int area = width * height;

      Serial.print("Object ");
      Serial.print(i + 1);
      Serial.print(": X=");
      Serial.print(x);
      Serial.print(", Y=");
      Serial.print(y);
      Serial.print(", W=");
      Serial.print(width);
      Serial.print(", H=");
      Serial.print(height);
      Serial.print(", Signature=");
      Serial.println(signature);

      // Вычисление угла и светодиодного индекса
      float angle = atan2(y, -x) * 180.0 / PI;
      if (angle < 0)
        angle += 360;
      int ledIndex = map((int(angle) + 180) % 360, 0, 360, 0, NUM_LEDS);

      // Обновление информации по сигнатурам
      if (signature == SIGNATURE_1)
      {
        sumX1 += x;
        sumY1 += y;
        totalArea1 += area;
        count1++;
        angle1 = int(angle);
        angle1 = 360 * (angle1 > 180) - angle1;
        f_front = true; // Сигнатура 1 найдена, поднимаем флаг
        setLEDColor(ledIndex, 4, 255, 0);
      }

      if (signature == SIGNATURE_2)
      {
        sumX2 += x;
        sumY2 += y;
        totalArea2 += area;
        count2++;
        angle2 = int(angle);
        angle2 = 360 * (angle2 > 180) - angle2;
        setLEDColor(ledIndex, 255, 255, 255);
      }
    }
  }
}

void move(int speedA, int speedB, int speedC, int speedD)
{
  analogWrite(pinA1, (speedA > 0) * speedA);
  analogWrite(pinB1, (speedB > 0) * speedB);
  analogWrite(pinC1, (speedC > 0) * speedC);
  analogWrite(pinD1, (speedD > 0) * speedD);

  analogWrite(pinA2, (speedA < 0) * abs(speedA));
  analogWrite(pinB2, (speedB < 0) * abs(speedB));
  analogWrite(pinC2, (speedC < 0) * abs(speedC));
  analogWrite(pinD2, (speedD < 0) * abs(speedD));
}

void move_angle(int speed, int angle, int u, float k)
{
  int pA = speed * sin(radians(angle - 45));
  int pB = speed * sin(radians(angle - 135));
  int pC = speed * sin(radians(angle + 135));
  int pD = speed * sin(radians(angle + 45));

  move(pA + int(u * k), pB + int(u * k), pC + int(u * k), pD + int(u * k));
}

// Функция для чтения данных из регистра
int readRegister(byte reg)
{
  Wire.beginTransmission(irLocatorAddress);
  Wire.write(reg);
  if (Wire.endTransmission() != 0)
  {
    return -1; // Ошибка записи
  }

  Wire.requestFrom(irLocatorAddress, 1); // Запрашиваем 1 байт данных
  if (Wire.available())
  {
    return Wire.read(); // Возвращаем значение, прочитанное из регистра
  }
  return -1; // Ошибка чтения
}

// Функция для чтения данных с IR Locator
IRSensorData readIRSensor(byte headingRegister, byte strengthRegister)
{
  IRSensorData sensorData;

  // Чтение угла
  int heading = readRegister(headingRegister);
  if (heading != -1)
  {
    sensorData.angle = (360 - (heading * 5)) % 360; // Угол в градусах
  }
  else
  {
    sensorData.angle = -1; // Ошибка
  }

  // Чтение силы сигнала
  int strength = readRegister(strengthRegister);
  sensorData.strength = (strength != -1) ? strength : -1; // Ошибка, если -1

  return sensorData;
}

void displayAngleOnRing(int angle, CRGB color)
{
  // Очищаем кольцо
  fill_solid(leds, NUM_LEDS, CRGB::Black);

  // Определяем индекс светодиода, который будет гореть указанным цветом
  int ledIndex = map(angle, 0, 360, 0, NUM_LEDS);

  // Устанавливаем указанный цвет для соответствующего светодиода
  leds[ledIndex] = color;

  // Отображаем обновления на кольце
  FastLED.show();
}

void ball_gates()
{
  while (true)
  {
    // Проверка кнопки для выхода
    if (digitalRead(BUTTON_PIN) == LOW)
    {
      delay(50); // Антидребезг
      if (digitalRead(BUTTON_PIN) == LOW)
      {
        selected = false;
        break;
      }
    }
    // Обработка данных с Pixy2
    processPixyData();

    if (f_front)
    {
      if (angle1 > 5)
        u = -speed;
      else if (angle1 < -5)
        u = speed;
      else
        u = 0;
      Serial.println(1);
    }
    else
    {
      if (angle2 > 5)
        u = speed;
      else if (angle2 < -5)
        u = -speed;
      else
        u = 0;
      Serial.println(2);
    }

    // Читаем данные с ИК-датчика
    IRSensorData ballData1200 = readIRSensor(0x04, 0x05);
    IRSensorData ballData600 = readIRSensor(0x06, 0x07);

    int angle, strength;

    // Выбираем угол на основе силы сигнала
    if (ballData1200.strength > 100)
      angle = ballData600.angle;
    else
      angle = ballData1200.angle;

    if (angle != -1)
    {
      int ledIndex = (angle + 180) % 360;
      if (ballData1200.strength >= 100) // Определяем цвет светодиода (красный - если мяч близко, оранжевый - если далеко)
        displayAngleOnRing(ledIndex, CRGB::Orange);
      else
        displayAngleOnRing(ledIndex, CRGB::Red);
      move_angle(speed, angle, u, k);

      delay(5);
    }

    // Очистка дисплея перед обновлением информации
    display.clearDisplay();

    display.print("Angle1=");
    display.println(angle1);

    display.print("Angle2=");
    display.println(angle2);

    // Отображаем статус флага f_front
    display.setCursor(0, 55);
    display.print("Front: ");
    display.println(f_front ? "YES" : "NO");

    display.display(); // Обновление дисплея
    FastLED.show();    // Обновление светодиодов

    Serial.print("f_front: ");
    Serial.println(f_front); // Вывод статуса флага в Serial Monitor

    delay(5); // Задержка обновления
  }
  move(0, 0, 0, 0);
  drawMenu(); // Возврат в меню
}

// Функция отображения скорости на OLED
void drawSpeed()
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.print("Speed:");
  display.setCursor(0, 30);
  display.setTextSize(2);
  display.print(motorSpeed);
  display.display();
}

// Функция обновления скорости мотора
void updateMotorSpeed()
{
  dribble.writeMicroseconds(motorSpeed);
  drawSpeed();
}

void dribbleMode()
{
  for (int i = 0; i < 32; i++)
    setLEDColor(i, 0, 0, 0);
  ;

  while (true)
  {
    // Проверка кнопки для выхода
    if (digitalRead(BUTTON_PIN) == LOW)
    {
      delay(50); // Антидребезг
      if (digitalRead(BUTTON_PIN) == LOW)
      {
        selected = false;
        break;
      }
    }
    // Обработка энкодера
    int MSB = digitalRead(ENCODER_PIN_A); // Most significant bit
    int LSB = digitalRead(ENCODER_PIN_B); // Least significant bit

    int encoded = (MSB << 1) | LSB;         // Конкатенация двух бит
    int sum = (lastEncoded << 2) | encoded; // Конкатенация текущего и предыдущего состояния

    int ledInd = 0;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
      encoderValue++;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
      encoderValue--;

    lastEncoded = encoded;

    // Проверяем, изменилось ли значение encoderValue
    long diff = encoderValue - lastReportedValue;
    if (diff >= 2)
    { // Увеличение скорости
      motorSpeed = constrain(motorSpeed + stepSize, minSpeed, maxSpeed);
      lastReportedValue += 2;
      updateMotorSpeed();
    }
    else if (diff <= -2)
    { // Уменьшение скорости
      motorSpeed = constrain(motorSpeed - stepSize, minSpeed, maxSpeed);
      lastReportedValue -= 2;
      updateMotorSpeed();
    }
    ledInd = map(motorSpeed, 1420, 1900, 0, NUM_LEDS);
    for (int i = 0; i < NUM_LEDS; i++)
      setLEDColor(ledInd, 255 * (diff > 0), 0, 0);
  }
  dribble.writeMicroseconds(1000);
  drawMenu(); // Возврат в меню
}

void dribble_goal()
{
  bool selectingSpeed = true; // Флаг для выбора скорости дрибблера
  bool movePhase = false;     // Флаг для движения после выбора скорости

  pinMode(19, INPUT_PULLUP); // Настраиваем кнопку на пине 19

  Serial.println("Dribble Gates Mode: Selecting Speed...");
  displayMode("Dribble Gates");

  while (true)
  {
    // Проверка кнопки для выхода
    if (digitalRead(BUTTON_PIN) == LOW)
    {
      delay(50);
      if (digitalRead(BUTTON_PIN) == LOW)
      {
        selected = false;
        move(0, 0, 0, 0);
        dribble.writeMicroseconds(1000);
        drawMenu();
        return;
      }
    }

    if (selectingSpeed)
    {
      // Обработка энкодера для выбора скорости дрибблера
      int MSB = digitalRead(ENCODER_PIN_A);
      int LSB = digitalRead(ENCODER_PIN_B);
      int encoded = (MSB << 1) | LSB;
      int sum = (lastEncoded << 2) | encoded;

      if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
        encoderValue++;
      if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
        encoderValue--;

      lastEncoded = encoded;
      long diff = encoderValue - lastReportedValue;

      if (diff >= 2)
      { // Увеличение скорости
        motorSpeed = constrain(motorSpeed + stepSize, minSpeed, maxSpeed);
        lastReportedValue += 2;
        updateMotorSpeed();
      }
      else if (diff <= -2)
      { // Уменьшение скорости
        motorSpeed = constrain(motorSpeed - stepSize, minSpeed, maxSpeed);
        lastReportedValue -= 2;
        updateMotorSpeed();
      }

      // Если нажата кнопка 19, завершаем настройку и начинаем движение
      if (digitalRead(19) == LOW)
      {
        delay(50);
        if (digitalRead(19) == LOW)
        {
          Serial.println("Speed selected. Moving...");
          selectingSpeed = false;
          movePhase = true;
        }
      }
    }
    else if (movePhase)
    {
      Serial.println("Moving for 400ms...");
      move(-200, -200, -200, 0);
      delay(250);
      move(0, 0, 0, 0);
      Serial.println("Movement complete. Exiting.");
      break;
    }

    // Обновление дисплея
    display.clearDisplay();
    if (selectingSpeed)
    {
      display.setCursor(0, 0);
      display.println("Select Speed:");
      display.print("Speed: ");
      display.println(motorSpeed);
    }
    else
    {
      display.setCursor(0, 0);
      display.println("Moving...");
    }
    display.display();
    delay(5);
  }

  move(0, 0, 0, 0);
  dribble.writeMicroseconds(1000);
  drawMenu(); // Возврат в меню
}

void gates_norm()
{
  while (true)
  {
    // Проверка кнопки для выхода
    if (digitalRead(BUTTON_PIN) == LOW)
    {
      delay(50); // Антидребезг
      if (digitalRead(BUTTON_PIN) == LOW)
      {
        selected = false;
        break;
      }
    }
    // Обработка данных с Pixy2
    processPixyData();

    if (f_front)
    {
      if (angle1 > 5)
        u = -75;
      else if (angle1 < -5)
        u = 75;
      else
        u = 0;
      Serial.println(1);
    }
    else
    {
      if (angle2 > 5)
        u = 75;
      else if (angle2 < -5)
        u = -75;
      else
        u = 0;
      Serial.println(2);
    }
    move(u, u, u, u);

    // Очистка дисплея перед обновлением информации
    display.clearDisplay();

    // Вывод информации на OLED
    display.setCursor(0, 0);
    display.println("Signature 1:");
    display.print("X=");
    display.print(avgX1);
    display.print(" Y=");
    display.println(avgY1);
    display.print("Area=");
    display.println(totalArea1);
    display.print("Angle=");
    display.println(angle1);

    display.setCursor(0, 35);
    display.println("Signature 2:");
    display.print("X=");
    display.print(avgX2);
    display.print(" Y=");
    display.println(avgY2);
    display.print("Area=");
    display.println(totalArea2);
    display.print("Angle=");
    display.println(angle2);

    // Отображаем статус флага f_front
    display.setCursor(0, 55);
    display.print("Front: ");
    display.println(f_front ? "YES" : "NO");

    display.display(); // Обновление дисплея
    FastLED.show();    // Обновление светодиодов

    Serial.print("f_front: ");
    Serial.println(f_front); // Вывод статуса флага в Serial Monitor

    delay(5); // Задержка обновления
  }
  move(0, 0, 0, 0);
  drawMenu(); // Возврат в меню
}

void gates()
{
  while (true)
  {
    // Проверка кнопки для выхода
    if (digitalRead(BUTTON_PIN) == LOW)
    {
      delay(50); // Антидребезг
      if (digitalRead(BUTTON_PIN) == LOW)
      {
        selected = false;
        break;
      }
    }
    // Обработка данных с Pixy2
    processPixyData();

    // Очистка дисплея перед обновлением информации
    display.clearDisplay();

    // Вывод информации на OLED
    display.setCursor(0, 0);
    display.println("Signature 1:");
    display.print("X=");
    display.print(avgX1);
    display.print(" Y=");
    display.println(avgY1);
    display.print("Area=");
    display.println(totalArea1);
    display.print("Angle=");
    display.println(angle1);

    display.setCursor(0, 35);
    display.println("Signature 2:");
    display.print("X=");
    display.print(avgX2);
    display.print(" Y=");
    display.println(avgY2);
    display.print("Area=");
    display.println(totalArea2);
    display.print("Angle=");
    display.println(angle2);

    display.display(); // Обновление дисплея
    FastLED.show();    // Обновление светодиодов
  }
  drawMenu(); // Возврат в меню
}

// Реализация режимов
void motorsMode()
{
  while (true)
  {
    // Проверка кнопки для выхода
    if (digitalRead(BUTTON_PIN) == LOW)
    {
      delay(50); // Антидребезг
      if (digitalRead(BUTTON_PIN) == LOW)
      {
        selected = false;
        break;
      }
    }

    fill_solid(leds, NUM_LEDS, colors[((millis() / 1000) % 8)]);
    FastLED.show();
    move_angle(speed, ((millis() / 1000) % 8) * 45, 0, 0);
  }
  move(0, 0, 0, 0);
  drawMenu(); // Возврат в меню
}

void moveBallMode()
{
    // Читаем данные с ИК-датчика
    IRSensorData ballData1200 = readIRSensor(0x04, 0x05);
    IRSensorData ballData600 = readIRSensor(0x06, 0x07);

    int angle, strength;

    // Выбираем угол на основе силы сигнала
    if (ballData1200.strength > 100)
      angle = ballData600.angle;
    else
      angle = ballData1200.angle;

    if (angle != -1)
    {
      int ledIndex = (angle + 180) % 360;

      // Определяем цвет светодиода (красный - если мяч близко, оранжевый - если далеко)
      if (ballData1200.strength >= 100)
        displayAngleOnRing(ledIndex, CRGB::Orange);
      else
        displayAngleOnRing(ledIndex, CRGB::Red);
      Serial.print(angle);
      Serial.print(" ");
      Serial.println(ledIndex);
      move_angle(speed, angle, 0, 0);

      delay(5);
    }
  move(0, 0, 0, 0);
  drawMenu(); // Возврат в меню
}

void ballGyro()
{
  while (true)
  {
    // Проверка кнопки для выхода
    if (digitalRead(BUTTON_PIN) == LOW)
    {
      delay(50); // Антидребезг
      if (digitalRead(BUTTON_PIN) == LOW)
      {
        selected = false;
        break;
      }
    }

    // Читаем данные с ИК-датчика
    IRSensorData ballData1200 = readIRSensor(0x04, 0x05);
    IRSensorData ballData600 = readIRSensor(0x06, 0x07);

    int angle, strength;

    int currentYaw = gyro_angle();

    if (currentYaw > 180)
      currentYaw -= 360;

    // Выбираем угол на основе силы сигнала
    if (ballData600.strength >= 6)
    {
      angle = ballData600.angle;
      strength = ballData600.strength;
    }
    else
    {
      angle = ballData1200.angle;
      strength = ballData1200.strength;
    }
    if (angle != -1)
    {
      // int ledIndex = (angle + 180) % 360;

      // // Определяем цвет светодиода (красный - если мяч близко, оранжевый - если далеко)
      // if (ballData600.strength >= 6)
      //   displayAngleOnRing(ledIndex, CRGB::Orange);
      // else
      //   displayAngleOnRing(ledIndex, CRGB::Red);
      // Serial.println(angle);
      // Serial.print(" ");
      // Serial.println(ledIndex);
      move_angle(speed, angle, currentYaw, 0.5);

      delay(5);
    }
  }
  move(0, 0, 0, 0);
  drawMenu(); // Возврат в меню
}

void buzzerMode()
{
  bool playMelody = false; // Флаг для воспроизведения мелодии

  while (true)
  {
    // Проверка кнопки для выхода или запуска мелодии
    if (digitalRead(BUTTON_PIN) == LOW)
    {
      delay(50); // Антидребезг
      if (digitalRead(BUTTON_PIN) == LOW)
      {
        playMelody = !playMelody; // Переключаем флаг воспроизведения мелодии
        delay(300);               // Небольшая задержка для предотвращения повторного срабатывания
      }
    }

    // Если флаг playMelody установлен, воспроизводим мелодию
    if (playMelody)
    {
      int numNotes = sizeof(melody) / sizeof(melody[0]);
      for (int i = 0; i < numNotes; i++)
      {
        int noteDuration = 1000 / noteDurations[i];
        tone(BUZZER_PIN, melody[i], noteDuration);
        delay(noteDuration * 1.30);
        noTone(BUZZER_PIN);

        // Проверка кнопки для выхода
        if (digitalRead(BUTTON_PIN) == LOW)
        {
          delay(50); // Антидребезг
          if (digitalRead(BUTTON_PIN) == LOW)
          {
            selected = false;
            drawMenu(); // Возврат в меню
            return;     // Выход из функции
          }
        }
      }
      playMelody = false; // Отключаем флаг после воспроизведения мелодии
    }

    // Проверка кнопки для выхода
    if (digitalRead(BUTTON_PIN) == LOW)
    {
      delay(50); // Антидребезг
      if (digitalRead(BUTTON_PIN) == LOW)
      {
        selected = false;
        break;
      }
    }
  }
  drawMenu(); // Возврат в меню
}

void ringMode()
{
  while (true)
  {
    // Проверка кнопки для выхода
    if (digitalRead(BUTTON_PIN) == LOW)
    {
      delay(50); // Антидребезг
      if (digitalRead(BUTTON_PIN) == LOW)
      {
        selected = false;
        break;
      }
    }

    // Основная логика режима
    int i = (millis() / 40) % NUM_LEDS;
    for (int _ = 0; _ < NUM_LEDS; _++)
    {
      setLEDColor(_, 160 * (i == _), 0, 255 * (i == _));
    }
  }
  drawMenu(); // Возврат в меню
}

// Функция обновления эффекта перелива
void updateRingEffect()
{
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CHSV((hueOffset + i * 10) % 255, 255, 255); // Плавный переход оттенков
  }
  FastLED.show();
  hueOffset += 5; // Обновление смещения
}

void irMode()
{
  while (true)
  {
    // Проверка кнопки для выхода
    if (digitalRead(BUTTON_PIN) == LOW)
    {
      delay(50); // Антидребезг
      if (digitalRead(BUTTON_PIN) == LOW)
      {
        selected = false;
        break;
      }
    }

    // Основная логика режима
    IRSensorData data1200Hz = readIRSensor(0x04, 0x05);
    IRSensorData data600Hz = readIRSensor(0x06, 0x07);

    display.clearDisplay();
    display.setCursor(0, 0);

    if (digitalRead(19))
    {
      display.println("1200 Hz:");
      display.print("angle: ");
      display.print(data1200Hz.angle);
      display.println("°");
      display.print("strength: ");
      display.println(data1200Hz.strength);

      display.println("");
      display.println("600 Hz:");
      display.print("angle: ");
      display.print(data600Hz.angle);
      display.println("°");
      display.print("strength: ");
      display.println(data600Hz.strength);
    }
    else
    {
      if (data1200Hz.strength > 100)
      {
        display.println("600 Hz:");
        display.print("angle: ");
        display.print(data600Hz.angle);
        display.println("°");
      }
      else
      {
        display.println("1200 Hz:");
        display.print("angle: ");
        display.print(data1200Hz.angle);
        display.println("°");
      }
      display.print("strength: ");
      display.println(data1200Hz.strength);
    }

    display.display();

    delay(100);
  }
  drawMenu(); // Возврат в меню
}


void gyroscopeMode()
{
  for (int i = 0; i < 32; i++)
    setLEDColor(i, 0, 0, 0);

  while (true)
  {
    // Проверка кнопки для выхода
    if (digitalRead(BUTTON_PIN) == LOW)
    {
      delay(50);
      if (digitalRead(BUTTON_PIN) == LOW)
      {
        selected = false;
        break;
      }
    }
    int currentYaw = gyro_angle();
    // Вывод на дисплей
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Yaw angle:");
    display.setTextSize(2);
    display.setCursor(0, 20);
    display.print(currentYaw);
    display.println("°");
    display.display();

    // int i_ring = (int)round(((currentYaw + 180.0) / 360.0) * NUM_LEDS) % NUM_LEDS;
    // for (int _ = 0; _ < NUM_LEDS; _++)
    // {
    //   setLEDColor(_, 150 * (i_ring == _), 150 * (i_ring == _), 150 * (i_ring == _));
    // }
    move(50, 50, 50, 50);
  }
  drawMenu(); // Возврат в меню
}

#define ALPHA 0.5 // Коэффициент фильтра (чем ближе к 1, тем сильнее фильтрация)
float filteredYaw = 0;

void updateFilteredYaw()
{
  mpu.update();
  int rawYaw = mpu.getAngleZ();
  filteredYaw = ALPHA * (filteredYaw + rawYaw) + (1 - ALPHA) * rawYaw;
}

void gyro_norm(int speed = 60, float k = 0.3)
{
  int u = 0;
  for (int i = 0; i < NUM_LEDS; i++)
    setLEDColor(i, 0, 0, 0);

  while (true)
  {
    if (digitalRead(BUTTON_PIN) == LOW)
    {
      delay(50);
      if (digitalRead(BUTTON_PIN) == LOW)
      {
        selected = false;
        break;
      }
    }

    updateFilteredYaw(); // Обновляем угол с фильтрацией

    int a = filteredYaw; // Берем отфильтрованный угол

    if (a > 5 && a < 180)
      u = 1;
    else if (a >= 180 && a < 355)
      u = -1;
    else
      u = (360 * (a > 180) - a) * -k;

    move(speed * u, speed * u, speed * u, speed * u);
    delay(1);
  }

  move(0, 0, 0, 0);
  drawMenu();
}

#define SMOOTHING_FACTOR 0.3 // Коэффициент сглаживания (0 - инертность, 1 - без сглаживания)

int filterValue(int newValue, int prevValue)
{
  return prevValue + (newValue - prevValue) * SMOOTHING_FACTOR;
}

void beep(int fq = 1000)
{
  tone(17, fq, 100); // Частота 1000 Гц, длительность 100 мс
  delay(100 + 10);   // Задержка для предотвращения наложений
}

void kick_test()
{
  for (int i = 0; i < NUM_LEDS; i++)
    setLEDColor(i, 0, 0, 0);

  while (true)
  {
    // Проверка кнопки для выхода
    if (digitalRead(BUTTON_PIN) == LOW)
    {
      delay(50); // Антидребезг
      if (digitalRead(BUTTON_PIN) == LOW)
      {
        selected = false;
        break;
      }
    }

    if (!digitalRead(19))
    {
      long t = millis();
      while (millis() - t < 1000)
      {
        for (int i = 0; i < NUM_LEDS; i++)
        {
          setLEDColor(i, 255 * ((millis() - t) / 50 % 2), 0, 0);
        }
        // if ((millis() - t) / 100 % 2)
        //   tone(17, 100, 10); // Частота 1000 Гц, длительность 100мс
      }
      for (int i = 0; i < NUM_LEDS; i++)
        setLEDColor(i, 0, 0, 0);
      delay(100);
      digitalWrite(21, 1);
      delay(15);
      digitalWrite(21, 0);
    }
  }
  drawMenu(); // Возврат в меню
}

void discharge_kck()
{
  for (int i = 0; i < NUM_LEDS; i++)
    setLEDColor(i, 0, 0, 0);
  long t;
  while (true)
  {
    // Проверка кнопки для выхода
    if (digitalRead(BUTTON_PIN) == LOW)
    {
      delay(50); // Антидребезг
      if (digitalRead(BUTTON_PIN) == LOW)
      {
        selected = false;
        break;
      }
    }
    if (digitalRead(19))
    {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("turn OFF pwr swithcer!");
      display.display();
    }

    else if (!digitalRead(19))
    {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("discharging...");
      display.display();
      for (int i = 0; i < 5; i++)
      {
        t = millis();
        if (millis() - t < 30)
          digitalWrite(21, 1);
        t = millis();
        if (millis() - t < 2000)
          digitalWrite(21, 0);
      }
      for (int i = 0; i < 3; i++)
      {
        t = millis();
        if (millis() - t < 500)
          digitalWrite(21, 1);
        t = millis();
        if (millis() - t < 800)
          digitalWrite(21, 0);
      }
      t = millis();
      if (millis() - t < 8000)
        digitalWrite(21, 1);
      digitalWrite(21, 0);
      delay(500);
    }
  }
  drawMenu(); // Возврат в меню
}

void dribble_kick()
{
  for (int i = 0; i < 32; i++)
    setLEDColor(i, 0, 0, 0);
  ;

  while (true)
  {
    // Проверка кнопки для выхода
    if (digitalRead(BUTTON_PIN) == LOW)
    {
      delay(50); // Антидребезг
      if (digitalRead(BUTTON_PIN) == LOW)
      {
        selected = false;
        break;
      }
    }
    // Обработка энкодера
    int MSB = digitalRead(ENCODER_PIN_A); // Most significant bit
    int LSB = digitalRead(ENCODER_PIN_B); // Least significant bit

    int encoded = (MSB << 1) | LSB;         // Конкатенация двух бит
    int sum = (lastEncoded << 2) | encoded; // Конкатенация текущего и предыдущего состояния

    int ledInd = 0;

    digitalWrite(21, !digitalRead(19));

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
      encoderValue++;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
      encoderValue--;

    lastEncoded = encoded;

    // Проверяем, изменилось ли значение encoderValue
    long diff = encoderValue - lastReportedValue;
    if (diff >= 2)
    { // Увеличение скорости
      motorSpeed = constrain(motorSpeed + stepSize, minSpeed, maxSpeed);
      lastReportedValue += 2;
      updateMotorSpeed();
    }
    else if (diff <= -2)
    { // Уменьшение скорости
      motorSpeed = constrain(motorSpeed - stepSize, minSpeed, maxSpeed);
      lastReportedValue -= 2;
      updateMotorSpeed();
    }
    ledInd = map(motorSpeed, 1420, 1900, 0, NUM_LEDS);
    for (int i = 0; i < NUM_LEDS; i++)
      setLEDColor(ledInd, 255 * (diff > 0), 0, 0);
  }
  dribble.writeMicroseconds(1000);
  drawMenu(); // Возврат в меню
}

// Запуск режимов
void launchMode(int modeIndex)
{
  switch (modeIndex)
  {
  case 0:
    motorsMode();
    break;
  case 1:
    moveBallMode();
    break;
  case 2:
    ballGyro();
    break;
  case 3:
    buzzerMode();
    break;
  case 4:
    ringMode();
    break;
  case 5:
    irMode();
    break;
  case 6:
    gyroscopeMode();
    break;
  case 7:
    gyro_norm(speed = 60);
    break;
  case 8:
    gates();
    break;
  case 9:
    gates_norm();
    break;
  case 10:
    ball_gates();
    break;
  case 11:
    dribbleMode();
    break;
  case 12:
    dribble_goal();
    break;
  case 13:
    kick_test();
    break;
  case 14:
    dribble_kick();
    break;
  case 15:
    discharge_kck();
    break;
  }
}

void loop()
{
  // Обновление эффекта перелива на кольце
  if (millis() - lastRingUpdate > ringInterval)
  {
    updateRingEffect();
    lastRingUpdate = millis();
  }
  // Обработка энкодера
  int MSB = digitalRead(ENCODER_PIN_A);
  int LSB = digitalRead(ENCODER_PIN_B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderValue++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderValue--;
  lastEncoded = encoded;

  // Проверяем, изменилось ли значение encoderValue
  long diff = encoderValue - lastReportedValue;
  if (!selected)
  {
    if (diff >= 2)
    { // Пролистывание вниз
      menuIndex = (menuIndex + 1) % menuLength;
      lastReportedValue += 2;
      updatePage();
    }
    else if (diff <= -2)
    { // Пролистывание вверх
      menuIndex = (menuIndex - 1 + menuLength) % menuLength;
      lastReportedValue -= 2;
      updatePage();
    }
  }

  // Обработка кнопки
  static bool buttonWasPressed = false;
  bool buttonState = digitalRead(BUTTON_PIN);

  if (buttonState == LOW)
  {
    buttonWasPressed = true;
  }
  else if (buttonWasPressed && buttonState == HIGH)
  {
    buttonPressed = true;
    buttonWasPressed = false;
  }

  if (buttonPressed)
  {
    buttonPressed = false;
    if (!selected)
    {
      selected = true;
      displayMode(menuItems[menuIndex]);
      launchMode(menuIndex);
    }
    else
    {
      selected = false;
      drawMenu();
    }
  }
}
