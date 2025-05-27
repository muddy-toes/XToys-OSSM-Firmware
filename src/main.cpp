#include <Arduino.h>
#include <StrokeEngine.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <Preferences.h>
#include "AiEsp32RotaryEncoder.h"
#include "utils/analog.h"
#include <U8g2lib.h>
#include "display.h"

#include "config.h"
#if COMPILE_BLUETOOTH
  #include "BLEManager.h"
#endif
#if COMPILE_WEBSOCKET
  #include "WebsocketManager.h"
#endif

// prefs configured during firmware flash from XToys website
Preferences preferences;
boolean prefUseWebsocket;
boolean prefUseBluetooth;

float currentSpeedPercentage = 0; // speed as a value from 0-100

//StaticJsonDocument<1024> doc;

StrokeEngine Stroker;

static motorProperties servoMotor {  
  .maxSpeed = MAX_SPEED,                
  .maxAcceleration = 100000,      
  .stepsPerMillimeter = STEP_PER_MM,   
  .invertDirection = true,      
  .enableActiveLow = true,      
  .stepPin = SERVO_PULSE,              
  .directionPin = SERVO_DIR,          
  .enablePin = SERVO_ENABLE              
};

static machineGeometry strokingMachine = {
  .physicalTravel = PHYSICAL_TRAVEL,       
  .keepoutBoundary = KEEPOUT_TRAVEL      
};

static endstopProperties endstop = {
  .homeToBack = true,    
  .activeLow = true,          
  .endstopPin = SERVO_ENDSTOP,
  .pinMode = INPUT_PULLUP
};

static sensorlessHomeProperties sensorlessHome = {
  .currentPin = SERVO_SENSORLESS,
  .currentLimit = 1
};

static DisplayManager displayManager;

void homingNotification(bool isHomed) {
  if (isHomed) {
    Stroker.moveToMax(HOMING_SPEED, true);
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
    // in this example we will set possible values between 0 and 1000;
    encoder.setBoundaries(
        0, 100, false);  // minValue, maxValue, circleValues true|false (when max
                        // go to min and vice versa)
    encoder.setAcceleration(0);

    // really disabled acceleration
    encoder.disableAcceleration();
}

void updateSpeed () {
  // convert from 0-100% to min-max in mm, taking into account current stroke length
  float newSpeed = currentSpeedPercentage * (SPEED_UPPER_LIMIT - SPEED_LOWER_LIMIT) / 100 + SPEED_LOWER_LIMIT;
  float percentOfStroke = Stroker.getStroke() / Stroker.getMaxDepth(); // scale speed faster if stroke is smaller
  newSpeed /= (percentOfStroke * 1.25);
  Stroker.setSpeed(newSpeed, true);
}

void processCommand(DynamicJsonDocument doc) {
  JsonArray commands = doc.as<JsonArray>();
  for(JsonObject command : commands) {
    String action = command["action"];
    //Serial.println("Action:");
    //Serial.println(action);
    
    if (action.equals("move")) {
      // position = 0-100
      // time = ms ?
      int position = command["position"];
      int time = command["time"];
      boolean replace = (command["replace"] == true);
      Stroker.appendToStreaming(position, time, replace);
    
    } else if (action.equals("startStreaming")) {
      Stroker.startStreaming();

    // homing
    } else if (action.equals("home")) {
      Stroker.disable();
      String type = command["type"];

      if (type.equals("sensor")) {
        String side = command["side"];
        if (side == "front") {
          endstop.homeToBack = true;
        } else {
          endstop.homeToBack = false;
        }
        Stroker.enableAndHome(&endstop, homingNotification, HOMING_SPEED);

      } else if (type.equals("manual")) {
        float rodLength = command["length"];
        Stroker.setPhysicalTravel(rodLength);
        Stroker.thisIsHome(HOMING_SPEED);
        homingNotification(true);

      } else if (type.equals("sensorless")) {
        float rodLength = command["length"];
        Stroker.setPhysicalTravel(rodLength);
        Stroker.enableAndSensorlessHome(&sensorlessHome, homingNotification, HOMING_SPEED);
      }
    
    } else if (action.equals("configureWebsocket")) {
      
      String ssid =  command["ssid"];
      String password = command["password"];

      preferences.putString("ssid", ssid);
      preferences.putString("password", password);
      preferences.putBool("useWebsocket", true);
      preferences.putBool("useBluetooth", false);
      preferences.end();

      delay(1000);
      ESP.restart();

    } else if (action.equals("configureBluetooth")) {
      
      String bleName =  command["name"];

      preferences.putString("bleName", bleName);
      preferences.putBool("useWebsocket", false);
      preferences.putBool("useBluetooth", true);
      preferences.end();

      delay(1000);
      ESP.restart();

    } else if (action.equals("stop")) {
      Stroker.stopMotion();
      
    } else if (action.equals("setPattern")) {
      int pattern = command["pattern"]; // XToys currently only sets pattern 0 (which it expects to be a basic steady in/out stroke)
      Stroker.setPattern(pattern, true);
      Stroker.startPattern();

    } else if (action.equals("setSpeed")) {
      currentSpeedPercentage = command["speed"];
      updateSpeed();
    
    } else if (action.equals("setStroke")) {
      float stroke = command["stroke"];
      Stroker.setStroke(stroke / 100.0 * Stroker.getMaxDepth(), true);
      updateSpeed();
      
    } else if (action.equals("setDepth")) {
      float depth = command["depth"];
      Stroker.setDepth(depth / 100.0 * Stroker.getMaxDepth(), true);

    } else if (action.equals("setPhysicalTravel")) {
      float travel = command["travel"];
      Stroker.setPhysicalTravel(travel);

    // not currently sent by XToys
    } else if (action.equals("setSensation")) {
      float sensation = command["sensation"];
      Stroker.setSensation(sensation, true);

    // not currently sent by XToys
    } else if (action.equals("setup")) {
      float speed = command["speed"] != NULL ? command["speed"] : 10.0;
      Stroker.setupDepth(speed, true);

    // not currently sent by XToys
    } else if (action.equals("retract")) {
      float speed = command["speed"] != NULL ? command["speed"] : 10.0;
      Stroker.moveToMin(speed);

    // not currently sent by XToys
    } else if (action.equals("extend")) {
      float speed = command["speed"] != NULL ? command["speed"] : 10.0;
      Stroker.moveToMax(speed);

    } else if (action.equals("connected")) {
      Stroker.disable();

    } else if (action.equals("disable")) {
      Stroker.disable();

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
      Serial.println(jsonString);

    } else if (action.equals("getPatternList")) {
      // not implemented
    }
  }
};

void onToyMessage (String msg) {
  StaticJsonDocument<512> localDoc;
  if( deserializeJson(localDoc, msg) == DeserializationError::Ok ) {
    processCommand(localDoc);
  }
};

void onBLEToyMessage (String msg) {
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

  Serial.println("Websocket Enabled: " + String(prefUseWebsocket));
  Serial.println("Bluetooth Enabled: " + String(prefUseBluetooth));

  displayManager.begin();

  #if COMPILE_WEBSOCKET
    if (prefUseWebsocket || AUTO_START_BLUETOOTH_OR_WEBSOCKET) {
      
      String ssid = preferences.getString("ssid", WIFI_SSID);
      String password = preferences.getString("password", WIFI_PSK);
      displayManager.setWiFiIcon(ICON_AVAILABLE);

      // Connect to WiFi
      displayManager.showConnecting();
      Serial.println("Setting up WiFi");
      WiFi.begin(ssid.c_str(), password.c_str());
      int wifiTimeout = 0;
      // Wait for up to 15 seconds for WiFi to connect
      while (WiFi.status() != WL_CONNECTED && wifiTimeout < 30) {
          Serial.print(".");
          displayManager.showSpinner(wifiTimeout % 4);
          delay(500);
          wifiTimeout++;
      }

      if (WiFi.status() == WL_CONNECTED) {
        displayManager.setWiFiIcon(ICON_CONNECTED);
        displayManager.showConnected(WiFi.localIP());
        Serial.print("IP=");
        Serial.println(WiFi.localIP());
        // Start websocket server
        WebsocketManager::setup(&onToyMessage);
      } else {
        Serial.print("Unable to connect to WiFi, disabling Websocket");
        displayManager.showConnectFailed();
        delay(2000);
      }
      
    }
  #endif

  #if COMPILE_BLUETOOTH
    if (prefUseBluetooth) {
      displayManager.setBluetoothIcon(ICON_AVAILABLE);
      String bleName = preferences.getString("bleName", BLE_NAME);
      BLEManager::setup(bleName, &onBLEToyMessage);
      Serial.print("BLE=");
      Serial.println(bleName);
    }
  #endif

  // Set PWM output with 8bit resolution and 5kHz
  ledcSetup(0, 5000, 8);
  ledcAttachPin(PWM, 0);
  ledcWrite(0, 0);

  // Setup Encoder
  initEncoder();

  // Setup Stroke Engine
  Stroker.begin(&strokingMachine, &servoMotor);

};

void loop() {
  static uint64_t nextUpdate=0;
  static float lastanalogValue=0;
  static float lastEncoderValue=0;
  bool lastBLEconnected = false;

  #if COMPILE_WEBSOCKET
    if (AUTO_START_BLUETOOTH_OR_WEBSOCKET || prefUseWebsocket) {
      WebsocketManager::loop();
    }
  #endif

  #if COMPILE_BLUETOOTH
    if (prefUseBluetooth) {
      if (BLEManager::isConnected() != lastBLEconnected) {
        lastBLEconnected = BLEManager::isConnected();
        if (lastBLEconnected) {
          displayManager.setBluetoothIcon(ICON_CONNECTED);
        } else {
          displayManager.setBluetoothIcon(ICON_AVAILABLE);
        }
      } 
    }
  #endif

  #if COMPILE_SERIAL
    // read new commands from serial
    if( Serial.available() ) {
      displayManager.setBluetoothIcon(ICON_CONNECTED);
      StaticJsonDocument<512> localDoc;
      if( deserializeJson(localDoc, Serial) == DeserializationError::Ok ) {
        processCommand(localDoc);
      }
    }
  #endif

   // Output remote and depth values to serial
  if (nextUpdate < millis()) {
    char msg[200];
    int used=1;
    nextUpdate = millis() + 100;
    //Serial.printf("Free heap: %i\n", ESP.getFreeHeap());
    msg[0] = '{';
    float analogValue = getAnalogAveragePercent(speedPotPin, 10);
    int encoderValue = encoder.readEncoder();
    used += snprintf(msg+used, sizeof(msg)-used, "\"analog\": %.0f,", analogValue);
    used += snprintf(msg+used, sizeof(msg)-used, "\"encoder\": %i,", encoderValue);
    used += snprintf(msg+used, sizeof(msg)-used, "\"depth\": %i", Stroker.getDepthPercent());
    snprintf(msg+used, sizeof(msg)-used, "}");
    Serial.println(msg);

    // Show running screen on display
    displayManager.showRunning();
  }
};