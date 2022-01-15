#define debugMode
#ifdef debugMode
#define debug(x) Serial.println(x)
#else
#define debug(x)
#endif
//-----debug-----//

//-----setting-----//
#define fireB 2
#define upB 4
#define downB 5
#define mosfet 9

#define changeDel 3000
#define sleepDel 10000
#define voltAdr 5
#define eeAdr 14
#define batLow 3.2  //нижний порог аккума

#define voltCalibr 0
//-----setting-----//


//-----lib & define & init-----//
#include "EEPROMex.h"
#include "lib_v1.2.h"
#include <TimerOne.h>
#include <OLED_I2C.h>
#include <GyverEncoder.h>
OLED oled(SDA, SCL);
extern uint8_t MediumFontRus[]; //ширина символа 6, высота 8

Button fire(fireB);
Button down(downB);
Button up(upB);
//-----lib & define & init-----//


//-----special variables-----//
struct {
	int8_t watt;
	float ohms;
} data;

bool fireOk;
uint8_t maxW;
float voltConst = 1.1;
const byte eeKey = 107;

uint32_t batTmr, changeTmr;
bool changeFlag;

int batVolt, batVoltF, batVoltOld;
float filterK = 0.04;
int PWM = 100, PWM_f = 500, PWM_old = 500;
float PWM_filter_k = 0.1;
//-----special variables-----//


//-----func-----//
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
uint8_t cap(int v) {  //вернет заряд в %
	uint8_t capacity;
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

bool checkBat(uint16_t x) {
	if (x <= batLow * 1000) {  //если аккум сел
		Timer1.disablePwm(mosfet);
		digitalWrite(mosfet, 0);
		oled.clrScr();
		oled.print("battery", CENTER, 16);
		oled.print("LOW", CENTER, 32);
		oled.update();
		delay(1000);
		oled.clrScr();
		while (1); //спим блять
	}
	return true;
}
//-----func-----//


void setup() {
#ifdef voltCalibr
	Serial.begin(9600);
#endif

	up.setDeb(50);
	up.setHold(500);
	up.setClickTimeOut(160);

	fire.setDeb(50);
	fire.setHold(500);
	fire.setClickTimeOut(160);

	down.setDeb(50);
	down.setHold(500);
	down.setClickTimeOut(160);

	Timer1.initialize(40);
	Timer1.disablePwm(mosfet);

	pinMode(mosfet, OUTPUT);
	digitalWrite(mosfet, LOW);

	if (voltCalibr) calibration();
	voltConst = EEPROM.readFloat(voltAdr);

	if (EEPROM.readByte(eeAdr - 1) != eeKey) {  //запись данных если шо
		data.watt = 20;
		data.ohms = 0.3;
		EEPROM.updateBlock(eeAdr, data);
		EEPROM.writeByte(eeAdr - 1, eeKey);
		debug("invalid key");
	}
	EEPROM.readBlock(eeAdr, data);

	oled.begin();
	oled.setFont(MediumFontRus);

	//batVoltOld = analogRead(A1) * (readVcc() / 1023.0);
	batVolt = batVoltOld = readVcc();

	oled.print("boxmod", CENTER, 16);
	oled.print("by Dizer", CENTER, 32);
	oled.update();
	delay(700);
	oled.clrScr();
}


#define sq(x) x*x
void loop() {
	fire.tick();
	up.tick();
	down.tick();

	//-----battery-----//
	if (millis() - batTmr > 20) {
		batTmr = millis();

		batVolt = readVcc();
		batVoltF = filterK * batVolt + (1 - filterK) * batVoltOld;  // фильтруем
		batVoltOld = batVoltF;
		//batVoltF -> напряжение на аккуме
		maxW = (float)(sq((float)batVoltF / 1000)) / data.ohms;

		if (checkBat(batVoltF)) {
			PWM = (float)data.watt / maxW * 1023;                   // считаем значение для ШИМ сигнала
			if (PWM > 1023) PWM = 1023;                                 // ограничил PWM "по тупому", потому что constrain сука не работает!
			PWM_f = PWM_filter_k * PWM + (1 - PWM_filter_k) * PWM_old;  // фильтруем
			PWM_old = PWM_f;                                            // фильтруем
		}

		if (fireOk) {
			digitalWrite(mosfet, 1);
			Timer1.pwm(mosfet, PWM_f);
		} else {
			digitalWrite(mosfet, 0);
			Timer1.disablePwm(mosfet);
		}

		oled.clrScr();
		//oled.print("ver1.2 ", CENTER, 0);
		oled.print(" watt: " + (String)data.watt, 0, 32);
		oled.print(" pwm: " + (String)PWM_f, 0, 16);
		//oled.print(" bat: " + (String)((float)batVoltF / 1000), 0, 48);
		oled.print(" bat: " + (String)cap(batVoltF) + '%', 0, 48);
		oled.print("mosf:" + (String)digitalRead(mosfet), CENTER, 0);
		//oled.print(">", 0, 16);
		oled.update();
	}
	//-----battery-----//

	if (down.isSingle() || down.isHolded()) {
		debug("down");
		data.watt -= 1;
		data.watt = max(data.watt, 1);
		changeFlag = true;
		changeTmr = millis();
	}

	if (up.isSingle() || up.isHolded()) {
		debug("up");
		data.watt += 1;
		data.watt = min(data.watt, maxW);
		data.watt = min(data.watt, 80);
		changeFlag = true;
		changeTmr = millis();
	}

	if (fire.isPress())fireOk = true;
	if (fire.isRelease())fireOk = false;

	/*
	if (fire.isRelease())firePos = false;
	if (fire.isPress()) firePos = true;

	if (enc.isPress()) {
		if (++mode >= 2)mode = 0;
	}

	if (enc.isLeft()) {
		debug("left");
		switch (mode) {
			case 0:data.watt -= 1; data.watt = max(data.watt, 0);
				break;
			case 1:data.ohms -= 0.1; data.ohms = max(data.ohms, 0.1);
				break;
		}
	}

	if (enc.isRight()) {
		debug("right");
		switch (mode) {
			case 0:data.watt += 1;data.watt = min(maxW, data.watt);
				break;
			case 1:data.ohms += 0.1;data.ohms = min(3, data.ohms);
				break;
		}
	}
	*/

	//-----saving data-----//
	if (changeFlag && (millis() - changeTmr > changeDel)) {
		changeFlag = false;
		changeTmr = millis();
		EEPROM.updateBlock(eeAdr, data);
		debug("changes saved");
	}
	//-----saving data-----//
}