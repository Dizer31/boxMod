#pragma once

struct Timer {
    Timer(uint32_t x, bool f = true) { _period = x; flag = f; }
    void setPeriod(uint32_t x) { _period = x; }
    void reboot() { _tmr = millis(); }
    bool _check() { return millis() - _tmr >= _period; }

    bool check() {
        if (flag && _check()) {
            reboot();
            return true;
        } return false;
    }

    bool checkFunc(void(*func)() = 0) {
        if (check() && func != 0) func();
    }

    uint32_t _period, _tmr;
    bool flag = true;
};
