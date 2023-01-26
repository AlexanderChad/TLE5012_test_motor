#include <Arduino.h>
#include <TLE5012-ino.hpp>

#define pin_en 2
#define pin_dir 3
#define pin_step 4

#define pin_cs 10
#define pin_miso 12
#define pin_mosi 11
#define pin_sck 13

SPIClass3W SPI3W;
Tle5012Ino Tle5012Sensor = Tle5012Ino(SPI3W, pin_cs, pin_miso, pin_mosi, pin_sck, Tle5012Sensor.TLE5012B_S0);
errorTypes checkError = INTERFACE_ACCESS_ERROR;

double d = 0.0; // angle
double r = 0.0; // range
double s = 0.0; // speed
double t = 0;   // Temperature
int16_t b = 0;  // Revolution

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
  };
  Serial.println("Starting");

  // setting pins
  pinMode(pin_en, OUTPUT);
  digitalWrite(pin_en, LOW);
  pinMode(pin_dir, OUTPUT);
  digitalWrite(pin_dir, LOW);
  pinMode(pin_step, OUTPUT);
  digitalWrite(pin_step, LOW);

  while (checkError != NO_ERROR)
  { // пока не инициализируем датчик успешно
    checkError = Tle5012Sensor.begin();
    Serial.print("checkError: ");
    Serial.println(checkError, HEX);
    delay(300);
  };

  Serial.println("Started TLE5012_test_motor");
}

/**
 * Функция вращения вала двигателя
 * @param steps           сколько нужно шагов (без учета делителя драйвера!)
 * @param dir             направление вращения
 * @param step_sleep_mcs  время сна между полушагами, чем неньше,
 *                        тем быстрее будет вращаться (500-16000).
 */
void turn_the_motor(uint16_t steps, uint8_t dir, uint16_t step_sleep_mcs)
{
  digitalWrite(pin_dir, dir); // выбираем направление
  // Serial.println("Started turn_the_motor");
  for (uint16_t i = 0; i < steps; i++)
  { // шагаем сколько нужно шагов
    digitalWrite(pin_step, HIGH);
    delayMicroseconds(step_sleep_mcs);
    digitalWrite(pin_step, LOW);
    delayMicroseconds(step_sleep_mcs);
  }
  // Serial.println("End turn_the_motor");
}

// Функция обновления значений с датчика
void tle5012_upd_values()
{
  Tle5012Sensor.getAngleValue(d);
  Tle5012Sensor.getNumRevolutions(b);
  Tle5012Sensor.getAngleSpeed(s);
  Tle5012Sensor.getAngleRange(r);
  Tle5012Sensor.getTemperature(t);
}

// Функция печати в монитор значений считанных ранее с датчика
void tle5012_print_values()
{
  Serial.print("Temperature:");
  Serial.print(t);
  Serial.print("°C\tangle:");
  Serial.print(d);
  Serial.print("°\trange:");
  Serial.print(r);
  Serial.print("\tspeed:");
  Serial.print(s);
  Serial.print("\t Revolution:");
  Serial.println(b);
}

/** получение значений с датчика и вывод на экран
 * @param disable_motor перед получением будут обесточиваться обмотки мотора
 * может влиять на точность считанных значений!
 */
void print_rez(bool disable_motor)
{
  if (disable_motor)
  {
    digitalWrite(pin_en, HIGH);
    delay(500);
  }
  tle5012_upd_values();
  tle5012_print_values();
  if (disable_motor)
  {
    digitalWrite(pin_en, LOW);
  }
}

#define steps_m 3200 // кол-во шагов для тестового поворота.
                     // 3200/32*1.8=~180°, 32 - делитель,  1.8 - шаг мотора
void loop()
{
  static uint8_t dir_m = 0;

  // if (millis() - Refresh_time_temp >= Refresh_time) {}
  if (Serial.available())
  {
    uint16_t spd = 0;
    char ch_com = Serial.read();
    switch (ch_com)
    {
    case 'q':
      print_rez(true);
      break;
    case 'z':
      spd = 50;
      break;
    case 'x':
      spd = 100;
      break;
    case 'c':
      spd = 300;
      break;
    case 'v':
      spd = 500;
      break;
    case 'b':
      spd = 1000;
      break;
    case 'n':
      spd = 2000;
      break;
    case 'm':
      spd = 3000;
      break;
    default:
      break;
    }
    if (spd)
    {
      turn_the_motor(steps_m, dir_m, spd);
      dir_m = !dir_m;
    }
  }
}
