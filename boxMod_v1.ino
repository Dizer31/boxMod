//-----setting-----//
#define okB 2
#define upB 4
#define downB 5
#define mosfet 8

#define changeDel 3000
#define sleepDel 10000
#define voltAdr 5
#define eeAdr 14

#define voltCalibr 0
//-----setting-----//

//-----lib & define & init-----//
#include "EEPROMex.h"
#include <TimerOne.h>
#include <OLED_I2C.h>
OLED oled(SDA, SCL);
extern uint8_t MediumFontRus[]; //ширина символа 6, высота 8

#include "lib.h"
Butt ok(okB);
Butt up(upB);
Butt down(downB);

#define debugMode
#ifdef debugMode
#define debug(x) Serial.println(x)
#else
#define debug(x)
#endif
//-----lib & define & init-----//

//-----special variables-----//
byte watts;
bool fireOk;
struct {
  byte watt;
} data;

float voltConst = 1.1;
const byte eeKey = 117;
int batVolt, batVoltF, batVoltOld;
float filterK = 0.04;
uint32_t batTmr, changeTmr, sleepTmr;
bool changeFlag;
float ohms = 0.3;

int PWM, PWM_f, PWM_old = 800;
float PWM_filter_k = 0.1;
int maxW;
//-----special variables-----//
//-----global func-----//
void calibration() {
  voltConst = 1.1; // начальаня константа калибровки
  Serial.print("Real VCC is: ");
  Serial.println(readVcc()); // общаемся с пользователем
  Serial.println("Write your VCC (in millivolts)");
  while (Serial.available() == 0);
  int Vcc = Serial.parseInt();           // напряжение от пользователя
  float real_const = (float)1.1 * Vcc / readVcc(); // расчёт константы
  Serial.print("New voltage constant: ");
  Serial.println(real_const, 3);
  Serial.println("Set vol_calibration 0, flash and enjoy!");
  EEPROM.writeFloat(voltAdr, real_const); // запись в EEPROM
  while (1); // уйти в бесконечный цикл
}
long readVcc() {
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif
  delay(2);      // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC))
    ;        // measuring
  uint8_t low = ADCL;  // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both
  long result = (high << 8) | low;

  result = voltConst * 1023 * 900 / result; // расчёт реального VCC
  return result;                // возвращает VCC
}
int cap(int v) {  //вернет заряд в %
  int capacity;
  if (v > 3870)
    capacity = map(v, 4200, 3870, 100, 77);
  else if ((v <= 3870) && (v > 3750))
    capacity = map(v, 3870, 3750, 77, 54);
  else if ((v <= 3750) && (v > 3680))
    capacity = map(v, 3750, 3680, 54, 31);
  else if ((v <= 3680) && (v > 3400))
    capacity = map(v, 3680, 3400, 31, 8);
  else if (v <= 3400)
    capacity = map(v, 3400, 2600, 8, 0);
  return capacity;
}
//-----global func-----//





void setup() {
  Serial.begin(9600);
  Timer1.initialize(40);
  pinMode(mosfet, OUTPUT);
  Timer1.disablePwm(mosfet);
  digitalWrite(mosfet, LOW);

  if (voltCalibr) calibration();
  voltConst = EEPROM.readFloat(voltAdr);

  if (EEPROM.readByte(eeAdr - 1) != eeKey) {
    data.watt = watts = 20;
    EEPROM.updateBlock(eeAdr, data);
    EEPROM.writeByte(eeAdr - 1, eeKey);
    debug("invalid key");
  }
  EEPROM.readBlock(eeAdr, data);
  watts = data.watt;

  oled.begin();
  oled.setFont(MediumFontRus);

  //batVoltOld = analogRead(A1) * (readVcc() / 1023.0);
  batVoltOld = readVcc();
  batVolt = batVoltOld;

  oled.print("boxmod", CENTER, 24);
  oled.update();
  delay(700);
}






#define sq(x) x*x
bool firePos = false;
void loop() {
  up.tick(); ok.tick(); down.tick();

  if (up.press()) {
    watts++;
    maxW = (sq((float)batVoltF / 1000)) / ohms;
    watts = min(watts, maxW);
    debug(watts);
    changeFlag = true;
    changeTmr = millis();
  }

  if (down.press()) {
    watts--;
    watts = max(0, watts);
    debug(watts);
    changeFlag = true;
    changeTmr = millis();
  }

  if (millis() - batTmr > 20) {
    batTmr = millis();
    //batVolt = analogRead(A1) * (readVcc() / 1023.0);
    batVolt = readVcc();                                // измерить напряжение аккумулятора в миллиВольтах
    batVoltF = filterK * batVolt + (1 - filterK) * batVoltOld;  // фильтруем
    batVoltOld = batVoltF;
    int capacity = cap(batVoltOld);

    if (batVoltF >= 3000) {
      fireOk = true;
      Timer1.disablePwm(mosfet);
      digitalWrite(mosfet, LOW);
    } else fireOk = false;

    if (fireOk) {

      PWM = (float)watts / maxW * 1023;                     // считаем значение для ШИМ сигнала
      if (PWM > 1023) PWM = 1023;                                 // ограничил PWM "по тупому", потому что constrain сука не работает!
      PWM_f = PWM_filter_k * PWM + (1 - PWM_filter_k) * PWM_old;  // фильтруем
      PWM_old = PWM_f;                                            // фильтруем
      debug(PWM_f);
      Timer1.pwm(mosfet, PWM_f);
      digitalWrite(mosfet, firePos);
    }

    /*
    oled.clrScr();
    oled.print((String)batVolt, CENTER, 0);
    oled.print((String)batVoltF, CENTER, 16);
    oled.print((String)capacity, CENTER, 32);
    oled.update();
    */
    oled.clrScr();
    oled.print("pwm: " + (String)PWM_f, CENTER, 0);
    oled.print("watt: " + (String)watts, CENTER, 16);
    oled.print("bat: " + (String)(batVoltF * 1.0 / 1000), CENTER, 32);
    oled.print((String)(digitalRead(mosfet)), CENTER, 48);
    oled.update();
  }

  if (ok.press())firePos = !firePos;
  if (ok.release())firePos = !firePos;

  if (changeFlag && millis() - changeTmr > changeDel) {
    changeTmr = millis();
    changeFlag = false;
    data.watt = watts;
    EEPROM.updateBlock(eeAdr, data);
    debug("changes saved");
  }
}
