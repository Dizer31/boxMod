#pragma once

class Butt {
public:

    Butt(int pin, int del = 20) {
        _pin = pin;
        _del = del;
        pinMode(pin, INPUT_PULLUP);
    }

    void tick() {
        _state = !digitalRead(_pin);
        if (_state && !_flag && (millis() - _tmr > _del)) {
            _tmr = millis();
            _flag = true;
            _flagPress = true;
            //debug(_pin);
        }

        if (!_state && _flag) {
            _flag = false;
            _flagRelease = true;
        }
    }

    bool press() {
        if (_flagPress) {
            _flagPress = false;
            return true;
        } else return false;
    }

    bool release() {
        if (_flagRelease) {
            _flagRelease = false;
            return true;
        } else return false;
    }

    int pin() {
        return _pin;
    }

private:
    bool _flagPress;
    bool _flagRelease;
    bool _flag;
    bool _state;
    int _pin;
    int _del;
    uint32_t _tmr;
};

