//#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <OLED_I2C.h>
#include <GyverTimer.h>
#include <EEPROM.h>
// пределы отображения для графиков

Adafruit_BMP085 bmp;

OLED  myOLED(SDA, SCL, 8);

GTimer_ms hour_Timer((long)60*60 * 1000);
GTimer_ms drawPlotTimer(1 * 1000);
GTimer_ms ButtonsTimer(100);

extern uint8_t RusFont[];
extern uint8_t MegaNumbers[];
extern uint8_t BigNumbers[];
unsigned long pressure, aver_pressure,  pressure_array[24];
unsigned long min_mapped_press, max_mapped_press, last_button;
unsigned long sumX, sumY, sumX2, sumXY;
float a;
int aver_temper, Mode_graph, buff_val, buff_val2, buff_val3, buff_val4, delta, temp,  flag, approx, min_mapped_temp, max_mapped_temp,  buff = 0.0, temp_array[24], time_array[24], temp_mapped[24], press_mapped[24];
boolean butt_flag, flag_set, set_on_flag;

void setup() {
  pinMode(A1, INPUT_PULLUP);
  pinMode(A0, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, OUTPUT);
  myOLED.begin();

  //Serial.begin(9600);

  bmp.begin(BMP085_ULTRAHIGHRES);  // включить датчик
  /*if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
    }*/
  int addr = 24;
  for (byte i = 0; i < 24; i++) {
    temp_array[i] = EEPROM[i];
    pressure_array[i] = EEPROM_ulong_read(i * 4 + 24);
    time_array[i] = i;
  }
}

void loop() {
  if (hour_Timer.isReady()) {
    get_approx();
    temp_graph();
    SaveEEPROM();
  }
  if (ButtonsTimer.isReady()) {
    Buttons();
  }



  if (drawPlotTimer.isReady()) {
    if (flag == 0) {

      myOLED.clrScr();
      myOLED.setBrightness(10);
      myOLED.setFont(RusFont);
      myOLED.print("Lfdktybt", CENTER, 4);  //
      myOLED.setFont(MegaNumbers);
      myOLED.print(String(bmp.readSealevelPressure() / 133.3, 1), CENTER, 20);
      myOLED.update();
    }

    if (flag == 1) {
      myOLED.clrScr();
      myOLED.setBrightness(10);
      myOLED.setFont(RusFont);
      myOLED.print("Ntvgthfnehf", CENTER, 4);  //
      myOLED.setFont(MegaNumbers);
      myOLED.print(String(bmp.readTemperature(), 1), CENTER, 20);
      myOLED.update();
    }

    if (flag == 2) {
      myOLED.clrScr();
      myOLED.setBrightness(10);
      myOLED.setFont(RusFont);
      myOLED.print("Ghtlcrfpfybt", CENTER, 4);  //
      myOLED.setFont(BigNumbers);
      myOLED.print(String(approx), CENTER, 20);
      myOLED.update();
    }

    if (flag == 3) {
      if (Mode_graph > -1 && Mode_graph < 24) {
        myOLED.clrScr();
        myOLED.setBrightness(10);
        myOLED.setFont(RusFont);
        myOLED.print("Uhfabr lfdktybz", CENTER, 0);
        myOLED.print(String(max_mapped_press), RIGHT, 9);
        myOLED.print(String(min_mapped_press), RIGHT, 55);
        buff_val4 = set_col_on_press(press_mapped, Mode_graph);
        myOLED.print(String(buff_val4), RIGHT, 32);
        myOLED.update();
      }
      else {
        myOLED.clrScr();
        myOLED.setBrightness(10);
        myOLED.setFont(RusFont);
        myOLED.print("Uhfabr lfdktybz", CENTER, 0);
        myOLED.print(String(max_mapped_press), RIGHT, 9);
        myOLED.print(String(min_mapped_press), RIGHT, 55);
        set_col(press_mapped);
        myOLED.update();
      }
    }

    if (flag == 4) {
      if (Mode_graph > -1 && Mode_graph < 24) {
        myOLED.clrScr();
        myOLED.setBrightness(10);
        myOLED.setFont(RusFont);
        myOLED.print("Uhfabr ntvgthfnehs", CENTER, 0);  //
        myOLED.print(String(max_mapped_temp), RIGHT, 9);
        myOLED.print(String(min_mapped_temp), RIGHT, 55);
        buff_val2 = set_col_on_temp(temp_mapped, Mode_graph);
        myOLED.print(String(buff_val2), RIGHT, 32);
        myOLED.update();
      }
      else {
        myOLED.clrScr();
        myOLED.setBrightness(10);
        myOLED.setFont(RusFont);
        myOLED.print("Uhfabr ntvgthfnehs", CENTER, 0);  //
        myOLED.print(String(max_mapped_temp), RIGHT, 9);
        myOLED.print(String(min_mapped_temp), RIGHT, 55);
        set_col(temp_mapped);
        myOLED.update();
      }
    }
  }
}

void Buttons() {
  boolean button2 = !digitalRead(A1);
  boolean button1 = !digitalRead(A0);
  boolean button3 = !digitalRead(A2);

  if (button2 == 1 && flag_set == 0 && millis() - last_button > 200) {
    flag_set == 1;
    set_on_flag = !set_on_flag;
    Mode_graph += 1;
    digitalWrite(A3, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(500);                       // wait for a second
    digitalWrite(A3, LOW);    // turn the LED off by making the voltage LOW
    //delay(500);
    if (Mode_graph > 24) {
      Mode_graph = -1;
    }
    last_button = millis();
  }

  if (button2 == 0 && flag_set == 1) {
    flag_set == 0;
    set_on_flag = !set_on_flag;
  }

  if (button1 == 1 && butt_flag == 0 && millis() - last_button > 200) {
    butt_flag = 1;
    flag = flag + 1;
    last_button = millis();
    digitalWrite(A3, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(500);                       // wait for a second
    digitalWrite(A3, LOW);    // turn the LED off by making the voltage LOW
    //delay(500);
  }
  if (button1 == 0 && butt_flag == 1) {
    butt_flag = 0;

  }
  if (button3 == 1 && butt_flag == 0 && millis() - last_button > 200) {
    butt_flag = 1;
    flag = flag - 1;
    digitalWrite(A3, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(500);                       // wait for a second
    digitalWrite(A3, LOW);    // turn the LED off by making the voltage LOW
    //delay(1000);
    last_button = millis();
  }
  if (button3 == 0 && butt_flag == 1) {
    butt_flag = 0;
  }
  if (flag > 4) {
    flag = 0;
  }
  if (flag < 0) {
    flag = 4;
  }
}

void temp_graph() {

  temp = aver_temp();                          // найти текущее давление по среднему арифметическому
  for (byte i = 0; i < 23; i++) {                   // счётчик от 0 до 5 (да, до 5. Так как 4 меньше 5)
    temp_array[i] = temp_array[i + 1];     // сдвинуть массив давлений КРОМЕ ПОСЛЕДНЕЙ ЯЧЕЙКИ на шаг назад
  }
  temp_array[23] = temp;

  max_mapped_temp = temp_array[0];
  min_mapped_temp = temp_array[0];

  for (byte i = 0; i < 24; i++) {                   // счётчик от 0 до 5 (да, до 5. Так как 4 меньше 5)
    if (temp_array[i] > max_mapped_temp) {
      max_mapped_temp = temp_array[i];
    }
    if (temp_array[i] < min_mapped_temp) {
      min_mapped_temp = temp_array[i];
    }
  }
  for (byte i = 0; i < 24; i++) {                   // счётчик от 0 до 5 (да, до 5. Так как 4 меньше 5)
    temp_mapped[i] = map(temp_array[i], min_mapped_temp - 10, max_mapped_temp + 10, 7, 63);
  }
}
long aver_temp() {
  temp = 0;
  for (byte i = 0; i < 10; i++) {
    temp += bmp.readTemperature();
  }
  aver_temper = temp / 10;
  return aver_temper;
}

long aver_sens() {
  pressure = 0;
  for (byte i = 0; i < 10; i++) {
    pressure += bmp.readPressure();
  }
  aver_pressure = pressure / 10;
  return aver_pressure;
}

void get_approx() {

  pressure = aver_sens();                          // найти текущее давление по среднему арифметическому
  for (byte i = 0; i < 23; i++) {                   // счётчик от 0 до 5 (да, до 5. Так как 4 меньше 5)
    pressure_array[i] = pressure_array[i + 1];     // сдвинуть массив давлений КРОМЕ ПОСЛЕДНЕЙ ЯЧЕЙКИ на шаг назад
  }
  pressure_array[23] = pressure;

  min_mapped_press = pressure_array[0] / 133.3;
  max_mapped_press = pressure_array[0] / 133.3;

  for (byte i = 0; i < 24; i++) {                   // счётчик от 0 до 5 (да, до 5. Так как 4 меньше 5)
    if (pressure_array[i] / 133.3 > max_mapped_press) {
      max_mapped_press = pressure_array[i] / 133.3 ;
    }
    if (pressure_array[i] / 133.3  < min_mapped_press) {
      min_mapped_press = pressure_array[i] / 133.3 ;
    }
  }
  for (byte i = 0; i < 24; i++) {                   // счётчик от 0 до 5 (да, до 5. Так как 4 меньше 5)
    press_mapped[i] = map(pressure_array[i] / 133.3, min_mapped_press - 10, max_mapped_press + 10, 7, 63); // сдвинуть массив давлений КРОМЕ ПОСЛЕДНЕЙ ЯЧЕЙКИ на шаг назад
  }


  // последний элемент массива теперь - новое давление
  sumX = 0;
  sumY = 0;
  sumX2 = 0;
  sumXY = 0;
  for (int i = 18; i < 24; i++) {                    // для всех элементов массива
    sumX += time_array[i];
    sumY += (long)pressure_array[i];
    sumX2 += time_array[i] * time_array[i];
    sumXY += (long)time_array[i] * pressure_array[i];
  }
  a = 0;
  a = (long)6 * sumXY;             // расчёт коэффициента наклона приямой
  a = a - (long)sumX * sumY;
  a = (float)a / (6 * sumX2 - sumX * sumX);
  delta = a * 6;                   // расчёт изменения давления
  approx = map(delta, -250, 250, 100, -100);  // пересчитать в угол поворота сервы
  //Serial.println(String(a) + " " + String(sumX));
}

void set_col(int arr[]) {
  for (byte i = 0; i < 24; i++) {
    myOLED.drawRect((i + 1) * 4, 63, (i + 1) * 4, 7 + (63 - arr[i]));
  }
}
int set_col_on_temp(int arr[], int num) {
  for (byte i = 0; i < 24; i++) {
    myOLED.drawRect((i + 1) * 4, 63, (i + 1) * 4, 7 + (63 - arr[i]));
    if (num == i) {
      myOLED.drawCircle((i + 1) * 4, 7 + (63 - arr[i]) - 4, 1);
      buff_val = temp_array[i];
    }
  }
  return buff_val;
}
int set_col_on_press(int arr[], int num) {
  for (byte i = 0; i < 24; i++) {
    myOLED.drawRect((i + 1) * 4, 63, (i + 1) * 4, 7 + (63 - arr[i]));
    if (num == i) {
      myOLED.drawCircle((i + 1) * 4, 7 + (63 - arr[i]) - 4, 1);
      buff_val3 = pressure_array[i] / 133.3;
    }
  }
  return buff_val3;
}
/*  myOLED.drawRect(1, 63, 16, 7 + (63 - value1));
  myOLED.drawRect(19, 63, 34, 7 + (63 - value2));
  myOLED.drawRect(37, 63, 52, 7 + (63 - value3));
  myOLED.drawRect(55, 63, 70, 7 + (63 - value4));
  myOLED.drawRect(73, 63, 88, 7 + (63 - value5));
  myOLED.drawRect(91, 63, 106, 7 + (63 - value6));*/


void SaveEEPROM() {
  for (byte i = 0; i < 24; i++) {
    EEPROM[i] = temp_array[i];
    EEPROM_ulong_write(i * 4 + 24, pressure_array[i]);
  }
}

// чтение
unsigned long EEPROM_ulong_read(int addr) {
  byte raw[4];
  for (byte i = 0; i < 4; i++) raw[i] = EEPROM.read(addr + i);
  unsigned long &num = (unsigned long&)raw;
  return num;
}

void EEPROM_ulong_write(int addr, unsigned long num) {
  byte raw[4];
  (unsigned long&)raw = num;
  for (byte i = 0; i < 4; i++) EEPROM.write(addr + i, raw[i]);
}
