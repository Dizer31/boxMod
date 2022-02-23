//-----setting-----//
#define debugMode 0
#define voltageBoostModule 1
#define IGNORE  0	//ignore battery voltage

#define fireB 2	//не изменять!!
#define downB 4
#define upB 5
#define mosfet 9  //не изменять!!
#define batPin A7 //пин к которому подключен аккум

#define changeDel 7 //в сек
#define sleepDel 14 //в сек
#define voltAdr 5	//адрес ячейки в которую пишется константа напряжения
#define eeAdr 14	//адрес ячейки в которую пишутся данные
#define batLow 3.1	//нижний порог аккума

#define voltCalibr 0 //калибровка напряжения
//-----setting-----//

//-----lib & define & init-----//
#if debugMode == 1
#define debug(x) Serial.println(x)
#else
#define debug(x)
#endif
#define sq(x) x*x

#include <OLED_I2C.h>
#include <TimerOne.h>
#include <LowPower.h>
#include "EEPROMex.h"
#include "buttonLib.h"
#include "timerLight.h"
OLED oled(SDA, SCL);
extern uint8_t MediumFontRus[]; //ширина символа 6, высота 8

Button fire(fireB);
Button down(downB);
Button up(upB);

Timer changeTmr(changeDel * 1000, false);
Timer sleepTmr(sleepDel * 1000);
Timer batTmr(20);
#include "function.h"
//-----lib & define & init-----//

void setup() {
	start();
}

void loop() {
	up.tick();
	down.tick();
	fire.tick();
	globalTick();
	buttonTick();

	batTmr.checkFunc(batTick);
	sleepTmr.checkFunc(sleep);
	changeTmr.checkFunc(dataUpdate);
}