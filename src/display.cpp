#include "display.h"
#include "config.h"

DisplayManager::DisplayManager()
    : display(U8G2_R0, -1, displayClock, displayData)
{}

void DisplayManager::drawWiFiIcon(int x, int y, IconState state) {
    // Simple WiFi icon: 3 arcs, filled or not depending on state
    if (state == ICON_OFF) return;

    if (state == ICON_CONNECTED) {
        display.setDrawColor(1);
        display.drawArc(x+6, y+6, 6, 225, 59);
        display.drawArc(x+6, y+6, 4, 225, 59);
        display.drawArc(x+6, y+6, 2, 225, 59);
        display.drawDisc(x+6, y+10, 4); // filled dot for connected
    } else if (state == ICON_AVAILABLE) {
        display.setDrawColor(1);
        display.drawArc(x+6, y+6, 6, 225, 59);
        display.drawArc(x+6, y+6, 4, 225, 59);
        // Only draw the smallest arc as a dashed line for available
        for (int angle = 225; angle < 59+360; angle += 12) {
            int a1 = angle;
            int a2 = angle + 6;
            if (a2 > 360) a2 -= 360;
            display.drawArc(x+6, y+6, 2, a1, a2);
        }
        display.drawCircle(x+6, y+10, 4); // hollow dot for available
    }
}

void DisplayManager::drawBluetoothIcon(int x, int y, IconState state) {
    if (state == ICON_OFF) return;

    display.setDrawColor(1);
    display.drawLine(x+4, y+2, x+8, y+10);
    display.drawLine(x+8, y+2, x+4, y+10);
    display.drawLine(x+6, y+0, x+6, y+12);

    if (state == ICON_CONNECTED) {
        display.drawDisc(x+6, y+13, 2); // larger filled dot for connected
    } else if (state == ICON_AVAILABLE) {
        display.drawCircle(x+6, y+13, 2); // larger hollow dot for available
        // Add a dashed underline for available
        for (int dx = -2; dx <= 2; dx += 2) {
            display.drawPixel(x+6+dx, y+15);
        }
    }
}

void DisplayManager::drawSerialIcon(int x, int y, IconState state) {
    if (state == ICON_OFF) return;

    display.setDrawColor(1);
    display.drawFrame(x+2, y+2, 10, 8);
    display.drawDisc(x+4, y+6, 1);
    display.drawDisc(x+10, y+6, 1);

    if (state == ICON_CONNECTED) {
        display.drawBox(x+5, y+11, 4, 3); // filled rectangle for connected
    } else if (state == ICON_AVAILABLE) {
        display.drawFrame(x+5, y+11, 4, 3); // hollow rectangle for available
        // Add a dashed line below for available
        display.drawPixel(x+6, y+15);
        display.drawPixel(x+8, y+15);
    }
}

void DisplayManager::drawIcons() {
    // Draw icons at the top right, spaced horizontally
    int x = 128 - 14;
    drawWiFiIcon(x, 0, wifiState);
    x -= 14;
    drawBluetoothIcon(x, 0, btState);
    x -= 14;
    drawSerialIcon(x, 0, serialState);
}

void DisplayManager::setWiFiIcon(IconState state) {
    wifiState = state;
}
void DisplayManager::setBluetoothIcon(IconState state) {
    btState = state;
}
void DisplayManager::setSerialIcon(IconState state) {
    serialState = state;
}

void DisplayManager::begin() {
    display.begin();
    display.setFont(u8g2_font_cu12_tr);
    display.setFontRefHeightExtendedText();
    display.setDrawColor(1);
    display.setFontPosTop();
    display.setFontDirection(0);
    display.setFontMode(1);
    display.clearBuffer();
    display.sendBuffer();
}

void DisplayManager::showConnecting() {
    display.clearBuffer();
    drawIcons();
    display.drawStr(0, 20, "Connecting to WiFi");
    display.sendBuffer();
}

void DisplayManager::showSpinner(uint8_t spinnerIndex) {
    static const char spinner[] = {'|', '/', '-', '\\'};
    display.setDrawColor(0);
    display.drawBox(64, 40, 17, 17);
    display.setDrawColor(1);
    drawIcons();
    char s[2] = {spinner[spinnerIndex % 4], '\0'};
    display.drawStr(64, 40, s);
    display.sendBuffer();
}

void DisplayManager::showConnected(IPAddress ip) {
    myip = ip;
    display.clearBuffer();
    drawIcons();
    display.print("Connected!");
    display.print(ip.toString());
    display.sendBuffer();
}

void DisplayManager::showRunning() {
    display.clearBuffer();
    drawIcons();
    display.setFont(u8g2_font_cu12_tr);
    display.drawStr(0, 24, "Running");
    if (myip == IPAddress(0, 0, 0, 0)) {
        display.drawStr(0, 46, "No IP Address");
    } else {
        display.drawStr(0, 46, myip.toString().c_str());
    }

    display.sendBuffer();
}

void DisplayManager::showConnectFailed() {
    display.clearBuffer();
    drawIcons();
    display.drawStr(0, 20, "Unable to connect");
    display.drawStr(0, 40, "to WiFi");
    display.sendBuffer();
}

void DisplayManager::clear() {
    display.clearBuffer();
    drawIcons();
    display.sendBuffer();
}
