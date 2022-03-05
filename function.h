#pragma once

//-----special variables-----//
struct {
    int8_t watt;
    float ohms;
    uint16_t counter;
} data;

bool fireOk, setingsFlag, changeFlag;
volatile bool globalFlag = false;
uint8_t maxW;
float voltConst = 1.1;
const byte eeKey = 108;

int16_t batVolt, batVoltF, batVoltOld;
float filterK = 0.07;
int16_t PWM = 100, PWM_f = 500, PWM_old = 500;
float PWM_filter_k = 0.6;
//-----special variables-----//

//-----func-----//
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
    delay(2);			 // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA, ADSC));				 // measuring
    uint8_t low = ADCL;	 // must read ADCL first - it then locks ADCH
    uint8_t high = ADCH; // unlocks both
    long result = (high << 8) | low;

    result = voltConst * 1023 * 900 / result; // расчёт реального VCC
    return result;							  // возвращает VCC
}

void calibration() {
    voltConst = 1.1; // начальаня константа калибровки
    debug("Real VCC is: " + (String)readVcc()); // общаемся с пользователем
    debug("Write your VCC (in millivolts)");
    while (Serial.available() == 0);
    int Vcc = Serial.parseInt();					 // напряжение от пользователя
    float real_const = (float)1.1 * Vcc / readVcc(); // расчёт константы
    debug("New voltage constant: " + (String)real_const);
    //Serial.println(real_const, 3);
    debug("Set vol_calibration 0, flash and enjoy!");
    EEPROM.writeFloat(voltAdr, real_const); // запись в EEPROM
    while (1); // уйти в бесконечный цикл
}

uint8_t cap(int v) { //вернет заряд в %
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

void wakeUp() {
    batTmr.reboot();
    sleepTmr.reboot();
    changeTmr.reboot();
}

void sleep() {
    debug("sleep");
    oled.clrScr();
    oled.print("Bye", CENTER, 32);
    oled.update();

    delay(500);
    oled.clrScr();
    oled.update();

    digitalWrite(mosfet, 0);
    Timer1.disablePwm(mosfet);

    delay(50);
    attachInterrupt(0, wakeUp, LOW);
    delay(50);

    globalFlag = true;
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}

bool checkBat(uint16_t x) {
#if IGNORE == 1
    return true;
#endif

    if (x <= batLow * 1000) {	//если аккум сел
        Timer1.disablePwm(mosfet);
        digitalWrite(mosfet, 0);
        oled.clrScr();
        oled.print("battery", CENTER, 16);
        oled.print("LOW", CENTER, 32);
        oled.update();
        delay(700);
        sleep();
    }
    return true;
}

void settings() {
    oled.clrScr();
    oled.print("mosfet:" + (String)digitalRead(mosfet), CENTER, 0);
    oled.print(">Ohms:" + (String)data.ohms, CENTER, 16);
    oled.print(" pwm: " + (String)PWM_f, 0, 32);
    oled.print(" bat: " + (String)(batVoltF / 1000.0), 0, 48);
    oled.update();
}

void draw() {
    oled.clrScr();
    oled.print(" bat: " + (String)cap(batVoltF) + '%', CENTER, 16);
    oled.print(">watt: " + (String)data.watt, CENTER, 32);
    oled.print(" puffs:" + (String)data.counter, CENTER, 48);
    oled.update();
}

void batTick() {
    //-----battery-----//
#if voltageBoostModule == 1
    batVolt = analogRead(batPin) * (readVcc() / 1023.0);
#else
    batVolt = readVcc();
#endif

    batVoltF = filterK * batVolt + (1 - filterK) * batVoltOld; // фильтруем
    batVoltOld = batVoltF;
    //batVoltF -> напряжение на аккуме
    maxW = (float)(sq((float)batVoltF / 1000)) / data.ohms;
    data.watt = constr(data.watt, 1, maxW);

    if (checkBat(batVoltF)) {
        PWM = (float)data.watt / maxW * 1023; // считаем значение для ШИМ сигнала
        if (PWM > 1023)PWM = 1023; // ограничил PWM "по тупому", потому что constrain сука не работает!
        PWM_f = PWM;	//закомментить если PWM фильтр включен
        //PWM_f = PWM_filter_k * PWM + (1 - PWM_filter_k) * PWM_old;  // фильтруем
        //PWM_old = PWM_f;                                            // фильтруем
    }
    if (setingsFlag)settings(); else draw();

    static bool dflag = true;
    static uint8_t d = 0;
    if (fireOk) {
        digitalWrite(mosfet, 1);
        Timer1.pwm(mosfet, PWM_f);

        if (dflag && ++d >= 10) {
            d = 0;
            data.counter++;
            dflag = false;
            changeTmr.flag = true;
            changeTmr.reboot();
        }
    } else {
        digitalWrite(mosfet, 0);
        Timer1.disablePwm(mosfet);
        dflag = true;
        d = 0;
    }
    //-----battery-----//
}

void dataInit() {
    data.watt = 30;
    data.ohms = 0.3;
    data.counter = 0;
}

void dataUpdate() {
    changeTmr.flag = false;
    oled.print("*", 0, 0);
    oled.update();
    EEPROM.updateBlock(eeAdr, data);
    debug("changes saved");
}

void globalTick() {
    if (globalFlag) {
        detachInterrupt(0);
        delay(50);
        batTmr.setPeriod(5000);
        oled.clrScr();
        oled.print("unlock?", CENTER, 32);
        oled.update();

        bool sleepFlag = false;
        while (1) {
            fire.tick();
            if (fire.isPress())batTmr.reboot();
            if (fire.isMultiple(5) || fire.isMultiple(4)) { sleepFlag = false; break; }
            if (batTmr.check()) { sleepFlag = true; break; }
        }

        if (sleepFlag) {
            batTmr.setPeriod(20);
            sleep();
        } else {
            globalFlag = false;
            oled.clrScr();
            oled.print("wake up", CENTER, 32);
            oled.update();
            delay(300);

            batTmr.reboot();
            sleepTmr.reboot();
            changeTmr.reboot();
        }
    }
}

void buttonTick() {
    //-----button-----//
    if (down.isSingle() || down.isHolded()) {
        debug("down");
        if (setingsFlag) {
            data.ohms -= 0.05;
            data.ohms = min(data.ohms, 0.005);
        } else {
            data.watt -= 1;
            data.watt = max(data.watt, 1);
        }
        changeTmr.flag = true;
        changeTmr.reboot();
        sleepTmr.reboot();
    }

    if (up.isSingle() || up.isHolded()) {
        debug("up");
        if (setingsFlag) {
            data.ohms += 0.05;
            data.ohms = min(data.ohms, 3.0);
        } else {
            data.watt += 1;
            data.watt = min(data.watt, maxW);
        }
        changeTmr.flag = true;
        changeTmr.reboot();
        sleepTmr.reboot();
    }

    /*
    if (fire.isMultiple(6) && setingsFlag) {
        oled.clrScr();
        oled.print("reset", CENTER, 0);
        oled.print("to", CENTER, 16);
        oled.print("factory", CENTER, 32);
        oled.print("settings", CENTER, 48);
        oled.update();
        dataInit();
        EEPROM.updateBlock(eeAdr, data);
        delay(700);
    }
    */
    if (fire.isPress()) {
        changeTmr.flag = true;
        changeTmr.reboot();
        sleepTmr.reboot();
        fireOk = true;
    }
    if (fire.isRelease()) fireOk = false;
    if (fire.isMultiple(3))setingsFlag = !setingsFlag;
    if (fire.isMultiple(5))sleep();
    //-----button-----//
}

void start() {
#if debugMode == 1 && 0
    Serial.begin(9600);
#endif

    Timer1.initialize(40);
    Timer1.disablePwm(mosfet);
    pinMode(mosfet, OUTPUT);
    digitalWrite(mosfet, LOW);

#if voltCalibr == 1
    calibration();
#endif
    voltConst = EEPROM.readFloat(voltAdr);

    if (EEPROM.readByte(eeAdr - 1) != eeKey) { //запись данных(если неверный ключ)
        dataInit();
        EEPROM.updateBlock(eeAdr, data);
        EEPROM.writeByte(eeAdr - 1, eeKey);
        debug("invalid key");
    } else EEPROM.readBlock(eeAdr, data);

    oled.begin();
    oled.setFont(MediumFontRus);

#if voltageBoostModule == 1
    batVolt = batVoltOld = analogRead(batPin) * (readVcc() / 1023.0);
#else
    batVolt = batVoltOld = readVcc();
#endif

    oled.print("boxmod", CENTER, 16);
    oled.print("by Dizer", CENTER, 32);
    oled.update();
    delay(700);
    oled.clrScr();
}
//-----func-----//
