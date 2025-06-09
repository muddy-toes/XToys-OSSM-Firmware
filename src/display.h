#pragma once
#include <U8g2lib.h>
#include <Arduino.h>

enum IconState {
    ICON_OFF = 0,
    ICON_AVAILABLE,
    ICON_CONNECTED
};

class DisplayManager {
public:
    DisplayManager();
    void begin();
    void showConnecting();
    void showSpinner(uint8_t spinnerIndex);
    void showConnected(IPAddress ip);
    void showConnectFailed();
    void clear();
    void showRunning();

    // Icon state setters
    void setWiFiIcon(IconState state);
    void setBluetoothIcon(IconState state);
    void setSerialIcon(IconState state);

private:
    U8G2_SSD1306_128X64_NONAME_F_HW_I2C display;
    IconState wifiState = ICON_OFF;
    IconState btState = ICON_OFF;
    IconState serialState = ICON_AVAILABLE;
    IPAddress myip = IPAddress(0, 0, 0, 0);

    void drawIcons();
    void drawWiFiIcon(int x, int y, IconState state);
    void drawBluetoothIcon(int x, int y, IconState state);
    void drawSerialIcon(int x, int y, IconState state);
};
