#include <Arduino.h>
#include "MotorController.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <Preferences.h>
#include "AiEsp32RotaryEncoder.h"
#include "utils/analog.h"
#include <U8g2lib.h>
#include "display.h"
#include "StreamingController.h"

#include "config.h"
#if COMPILE_BLUETOOTH
  #include "BLEManager.h"
#endif
#if COMPILE_WEBSOCKET
  #include "WebsocketManager.h"
#endif
#if COMPILE_SERIAL
  #include "SerialManager.h"
#endif

// prefs configured during firmware flash from XToys website
Preferences preferences;
boolean prefUseWebsocket;
boolean prefUseBluetooth;

long rebootInMillis = 0; // if > 0, reboot in this many ms

// Current stroke/depth as percentages (0-100), updated by setStroke/setDepth
float currentStrokePercent = 100;  // stroke length (0=none, 100=full range back from depth)
float currentDepthPercent = 100;   // max insertion position (0=retracted, 100=fully inserted)
float currentSpeedPercent = 0;    // speed mode speed (0-100)

//StaticJsonDocument<1024> doc;

MotorController Motor;
StreamingController streamingController;

// Compute stroke limits in steps from current stroke/depth percentages
// depth = max insertion position (0-100%), stroke = length of movement back from depth (0-100%)
// Example: depth=80%, stroke=50% -> moves between 30% and 80% of travel
void getStrokeLimits(int32_t* outMin, int32_t* outMax) {
    int32_t range = Motor.getMaxStep() - Motor.getMinStep();
    *outMax = Motor.getMinStep() + static_cast<int32_t>(currentDepthPercent / 100.0 * range);
    *outMin = *outMax - static_cast<int32_t>(currentStrokePercent / 100.0 * range);
    if (*outMax > Motor.getMaxStep()) *outMax = Motor.getMaxStep();
    if (*outMin < Motor.getMinStep()) *outMin = Motor.getMinStep();
}

// Convert 0-100 speed percentage to strokes per minute, scaled by stroke length
float speedPercentToSPM(float speedPercent) {
    float spm = speedPercent * (SPEED_UPPER_LIMIT - SPEED_LOWER_LIMIT) / 100.0f + SPEED_LOWER_LIMIT;
    // Scale speed up when stroke is shorter (less distance to cover)
    int32_t strokeMin, strokeMax;
    getStrokeLimits(&strokeMin, &strokeMax);
    int32_t range = Motor.getMaxStep() - Motor.getMinStep();
    int32_t strokeRange = strokeMax - strokeMin;
    if (range > 0 && strokeRange > 0) {
        float percentOfFullStroke = (float)strokeRange / range;
        spm /= (percentOfFullStroke * 1.25f);
    }
    return spm;
}

static DisplayManager displayManager;

void homingNotification(bool isHomed) {
  if (isHomed) {
    // Update StreamingController with new physical limits from homing
    streamingController.updatePhysicalLimits(Motor.getMinStep(), Motor.getMaxStep());
    int32_t strokeMin, strokeMax;
    getStrokeLimits(&strokeMin, &strokeMax);
    streamingController.setStrokeLimits(strokeMin, strokeMax);

    // Move to max position after successful homing (matches main branch behavior)
    Motor.moveToMax(HOMING_SPEED, true);
  }
  DynamicJsonDocument doc(200);
  JsonArray root = doc.to<JsonArray>();
  JsonObject versionInfo = root.createNestedObject();
  versionInfo["action"] = "home";
  versionInfo["success"] = isHomed;
  String jsonString;
  serializeJson(doc, jsonString);

  #if COMPILE_WEBSOCKET
    if (prefUseWebsocket || AUTO_START_BLUETOOTH_OR_WEBSOCKET) {
      WebsocketManager::sendMessage(jsonString);
    }
  #endif
  #if COMPILE_BLUETOOTH
    if (prefUseBluetooth) {
      BLEManager::sendNotification(jsonString);
    }
  #endif
  //Serial.println(jsonString);
}

// Encoder stuff
static AiEsp32RotaryEncoder encoder = AiEsp32RotaryEncoder(
    encoderA, encoderB, encoderSwitch,
    encoderPower, encoderStepsPerNotch);

static void IRAM_ATTR readEncoderISR() { encoder.readEncoder_ISR(); }

static void initEncoder() {
    // we must initialize rotary encoder
    encoder.begin();
    encoder.setup(readEncoderISR);
    // set boundaries and if values should cycle or not
    encoder.setBoundaries(
        0, 100, false);  // minValue, maxValue, circleValues true|false (when max
                        // go to min and vice versa)
    encoder.setAcceleration(0);

    // really disabled acceleration
    encoder.disableAcceleration();
}

void processCommand(DynamicJsonDocument doc) {
  //serializeJsonPretty(doc, Serial); Serial.println();

  JsonArray commands = doc.as<JsonArray>();
  for(JsonObject command : commands) {
    String action = command["action"];

    if (action.equals("move")) {
      // position = 0-100
      // time = ms (duration to reach position)
      int position = command["position"];
      int time = command["time"];
      boolean replace = (command["replace"] == true);
      // Use new look-ahead StreamingController instead of StrokeEngine
      streamingController.addTarget(position, time, replace);

    } else if (action.equals("startStreaming")) {
      // Stop both modes before starting streaming
      Motor.stopPattern();
      streamingController.stop();
      if (Motor.getState() == MOTOR_READY) {
        streamingController.updatePhysicalLimits(Motor.getMinStep(), Motor.getMaxStep());
        int32_t strokeMin, strokeMax;
        getStrokeLimits(&strokeMin, &strokeMax);
        streamingController.setStrokeLimits(strokeMin, strokeMax);
        streamingController.start();
      }

    // homing
    } else if (action.equals("home")) {
      streamingController.stop();
      Motor.disable();
      String type = command["type"];

      if (type.equals("sensor")) {
        // Endstop homing - direction based on which side
        // Original logic: side="front" means homeToBack=true, else false
        String side = command["side"];
        bool homeToBack = side.equals("front");
        Motor.homeEndstop(SERVO_ENDSTOP, true, homeToBack, HOMING_SPEED, homingNotification);

      } else if (type.equals("manual")) {
        float rodLength = command["length"];
        Motor.homeManual(rodLength);
        homingNotification(true);

      } else if (type.equals("sensorless")) {
        float maxTravel = command["length"] | 0.0f;  // 0 = no limit (backward compat)
        Motor.homeSensorless(SERVO_SENSORLESS, 1.5, SENSORLESS_HOMING_SPEED, maxTravel, homingNotification);
      }
    
    } else if (action.equals("configureWebsocket")) {
      String ssid =  command["ssid"];
      String password = command["password"];

      preferences.putString("ssid", ssid);
      preferences.putString("password", password);
      preferences.putBool("useWebsocket", true);
      preferences.end();

      rebootInMillis = millis() + 3000; // reboot in 3 seconds


    } else if (action.equals("configureBluetooth")) {
      String bleName =  command["name"];

      preferences.putString("bleName", bleName);
      preferences.putBool("useBluetooth", true);
      preferences.end();

      rebootInMillis = millis() + 3000; // reboot in 3 seconds

    } else if (action.equals("stop")) {
      streamingController.stop();
      Motor.stopPattern();

    } else if (action.equals("setStroke")) {
      currentStrokePercent = command["stroke"];
      int32_t strokeMin, strokeMax;
      getStrokeLimits(&strokeMin, &strokeMax);
      streamingController.setStrokeLimits(strokeMin, strokeMax);
      Motor.setPatternLimits(strokeMin, strokeMax);
      // Update pattern speed since it's scaled by stroke length
      Motor.setPatternSpeed(speedPercentToSPM(currentSpeedPercent));

    } else if (action.equals("setDepth")) {
      currentDepthPercent = command["depth"];
      int32_t strokeMin, strokeMax;
      getStrokeLimits(&strokeMin, &strokeMax);
      streamingController.setStrokeLimits(strokeMin, strokeMax);
      Motor.setPatternLimits(strokeMin, strokeMax);
      Motor.setPatternSpeed(speedPercentToSPM(currentSpeedPercent));

    } else if (action.equals("setPhysicalTravel")) {
      float travel = command["travel"];
      Motor.setPhysicalTravel(travel);

    } else if (action.equals("connected")) {
      streamingController.stop();
      Motor.disable();

    } else if (action.equals("disable")) {
      streamingController.stop();
      Motor.disable();

    // print version JSON to any active connections
    } else if (action.equals("version")) {
      DynamicJsonDocument doc(200);
      JsonArray root = doc.to<JsonArray>();
      JsonObject versionInfo = root.createNestedObject();
      versionInfo["action"] = "version";
      versionInfo["api"] = API_VERSION;
      versionInfo["firmware"] = FIRMWARE_VERSION;
      String jsonString;
      serializeJson(doc, jsonString);

      #if COMPILE_WEBSOCKET
        if (prefUseWebsocket || AUTO_START_BLUETOOTH_OR_WEBSOCKET) {
          WebsocketManager::sendMessage(jsonString);
        }
      #endif
      #if COMPILE_BLUETOOTH
        if (prefUseBluetooth) {
          BLEManager::sendNotification(jsonString);
        }
      #endif
      Serial.println(jsonString);

    } else if (action.equals("getPatternList")) {
      // not implemented

    // Speed mode (oscillation pattern)
    } else if (action.equals("setPattern")) {
      // XToys sends setPattern when speed mode is selected
      // Stop both modes before starting pattern
      Motor.stopPattern();
      streamingController.stop();
      if (Motor.getState() == MOTOR_READY) {
        int32_t strokeMin, strokeMax;
        getStrokeLimits(&strokeMin, &strokeMax);
        float spm = speedPercentToSPM(currentSpeedPercent);
        Motor.startPattern(strokeMin, strokeMax, spm);
      }

    } else if (action.equals("setSpeed")) {
      currentSpeedPercent = command["speed"];
      float spm = speedPercentToSPM(currentSpeedPercent);
      Motor.setPatternSpeed(spm);
    }
  }
};

void onToyMessage (String msg) {
  StaticJsonDocument<512> localDoc;
  if( deserializeJson(localDoc, msg) == DeserializationError::Ok ) {
    processCommand(localDoc);
  }
};



void setup() {
  Serial.begin(115200);

  // load prefs previously set via XToys website
  preferences.begin("ossm", false);
  prefUseWebsocket = preferences.getBool("useWebsocket", AUTO_START_BLUETOOTH_OR_WEBSOCKET);
  prefUseBluetooth = preferences.getBool("useBluetooth", AUTO_START_BLUETOOTH_OR_WEBSOCKET);

  displayManager.begin();

  #if COMPILE_WEBSOCKET
    if (prefUseWebsocket || AUTO_START_BLUETOOTH_OR_WEBSOCKET) {
      
      String ssid = preferences.getString("ssid", WIFI_SSID);
      String password = preferences.getString("password", WIFI_PSK);
      displayManager.setWiFiIcon(ICON_AVAILABLE);

      // Connect to WiFi
      displayManager.showConnecting();
      WiFi.begin(ssid.c_str(), password.c_str());
      int wifiTimeout = 0;
      // Wait for up to 15 seconds for WiFi to connect
      while (WiFi.status() != WL_CONNECTED && wifiTimeout < 30) {
          displayManager.showSpinner(wifiTimeout % 4);
          delay(500);
          wifiTimeout++;
      }

      if (WiFi.status() == WL_CONNECTED) {
        displayManager.setWiFiIcon(ICON_CONNECTED);
        displayManager.showConnected(WiFi.localIP());
        // Start websocket server
        WebsocketManager::setup(&onToyMessage);
      } else {
        displayManager.showConnectFailed();
        delay(2000);
      }
      
    }
  #endif

  #if COMPILE_BLUETOOTH
    if (prefUseBluetooth) {
      displayManager.setBluetoothIcon(ICON_AVAILABLE);
      String bleName = preferences.getString("bleName", BLE_NAME);
      BLEManager::setup(bleName, &onToyMessage);
    }
  #endif

  #if COMPILE_SERIAL
    displayManager.setSerialIcon(ICON_AVAILABLE);
    SerialManager::setup(&onToyMessage);
  #endif

  // Set PWM output with 8bit resolution and 5kHz
  ledcSetup(0, 5000, 8);
  ledcAttachPin(PWM, 0);
  ledcWrite(0, 0);

  // Setup Encoder
  initEncoder();

  // Setup Motor Controller
  Motor.begin(SERVO_PULSE, SERVO_DIR, SERVO_ENABLE, STEP_PER_MM, true, true);
  Motor.setMaxSpeed(MAX_SPEED);
  Motor.setMaxAcceleration(100000);

  // Setup StreamingController with safety limits from Motor
  // Note: Full initialization happens when startStreaming is called,
  // after homing establishes valid position limits
  streamingController.begin(
    Motor.getServo(),
    Motor.getMinStep(),
    Motor.getMaxStep(),
    Motor.getMaxStepPerSecond(),
    Motor.getMaxStepAcceleration(),
    Motor.getStepsPerMillimeter()
  );

};

void loop() {
  static uint64_t nextUpdate=0;
  static bool lastBLEconnected = false;

  // Note: StreamingController tick() now runs in its own dedicated FreeRTOS task
  // created by start() and deleted by stop()

  if (rebootInMillis > 0 && millis() > rebootInMillis) {
    ESP.restart();
  }

  #if COMPILE_WEBSOCKET
    if (AUTO_START_BLUETOOTH_OR_WEBSOCKET || prefUseWebsocket) {
      WebsocketManager::loop();
    }
  #endif

  #if COMPILE_BLUETOOTH
    if (prefUseBluetooth) {
      BLEManager::processQueue();

      if (BLEManager::isConnected() != lastBLEconnected) {
        lastBLEconnected = BLEManager::isConnected();
        if (lastBLEconnected) {
          displayManager.setBluetoothIcon(ICON_CONNECTED);
        } else {
          streamingController.stop();
          Motor.disable();
          displayManager.setBluetoothIcon(ICON_AVAILABLE);
        }
      }
    }
  #endif

  #if COMPILE_SERIAL
    SerialManager::loop();
    SerialManager::processQueue();

    static bool lastSerialConnected = false;
    if (SerialManager::isConnected() != lastSerialConnected) {
      lastSerialConnected = SerialManager::isConnected();
      displayManager.setSerialIcon(lastSerialConnected ? ICON_CONNECTED : ICON_AVAILABLE);
    }
  #endif

   // Output remote and depth values to serial
  if (nextUpdate < millis()) {
    char msg[200];
    int used=1;
    nextUpdate = millis() + 200;
    //Serial.printf("Free heap: %i\n", ESP.getFreeHeap());
    msg[0] = '{';
    float analogValue = getAnalogAveragePercent(speedPotPin, 10);
    int encoderValue = encoder.readEncoder();
    used += snprintf(msg+used, sizeof(msg)-used, "\"analog\": %.0f,", analogValue);
    used += snprintf(msg+used, sizeof(msg)-used, "\"encoder\": %i,", encoderValue);
    used += snprintf(msg+used, sizeof(msg)-used, "\"depth\": %i", Motor.getDepthPercent());
    snprintf(msg+used, sizeof(msg)-used, "}");
    // send to serial
    Serial.println(msg);
    if (prefUseWebsocket || AUTO_START_BLUETOOTH_OR_WEBSOCKET) {
      if (WiFi.status() == WL_CONNECTED) {
        // Send to websocket if connected
        WebsocketManager::sendMessage(msg);
        }
    }

    // Show running screen on display
    displayManager.showRunning();
  }
};