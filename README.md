
Muddy's Notes:

The modifications on this fork are for my own benefit and may include adjustments that won't be appropriate for your machine.  Particularly note
that I have changed the steps-per-revolution from 2000 to 800 to match the official OSSM firmware.

Review the differences before flashing this to your OSSM.

Uses my fork of StrokeEngine.  Check it out into lib/StrokeEngine under your checkout of this repo.

This firmware does NOT support the wired controller.

Serial connectivity still drops after some time and I don't know why, but Bluetooth has been quite stable for me.

Sensorless homing is still hit-or-miss.  I recommend using manual homing.  When connecting the OSSM in XToys, push the effector all the way in so the business end is touching the motor.  Measure the amount of rail you can see sticking out the back.  Only measure the silver rail, from where it comes out of the housing to where it goes into the end stop piece.  Enter the distance in mm into the manual homing distance and hit the homing button.  It will move all the way out and stop.  If it skips the belt at the end, you entered too high of a number.

Important safety notes that apply here as well as to the main XToys firmware:

  1. In positition streaming mode, depending on the pattern you run, the machine can move very erratically and suddenly.  That is correct behaviour, because it needs to be able to make any move the pattern presents it with.  You should preview any patterns or funscripts you intend to use for play BEFORE using them during play.
  2. Once you connect your OSSM to XToys, you should try all the functionality.  Try all the controls to get a feel for how the machine reacts to various things before it's in contact with a human.
  3. If you're going to host a public session on XToys, I recommend disabling position mode.  You can do that by clicking "OSSM" under the bluetooth button in your OSSM block in XToys, then click Edit.  Set "Enabled Modes" to Speed.  

I've added a firmware_merged.bin file that's ready to flash if you aren't into compiling it.  It's got Bluetooth enabled, not WiFi.  It's the same file I run on my OSSM where it's worked well, but of course use at your own risk.

---------------------------

Firmware for controlling the [OSSM (Open Source Sex Machine)](https://github.com/KinkyMakers/OSSM-hardware) via the [XToys.app](https://xtoys.app) website.

The firmware enables the OSSM to be controlled via serial, Bluetooth or websocket commands.

Uses a [slightly modified version](https://github.com/denialtek/StrokeEngine) of the [StrokeEngine](https://github.com/theelims/StrokeEngine) library to interact with the OSSM.

# Automatic Setup

The firmware can be installed directly from the XToys website via these steps:
1. Connect your OSSM via USB cable.
2. Load the OSSM block in XToys and click the grey connect button.
3. Select the download icon beside the v2.0 label and follow the install steps.

# Manual Setup

1. On your OSSM motor set the DIP switches to:  
![S1=Off, S2=On, S3=On, S4=Off, S6=Off](ossm-dip.png)  
**Note:** If the dildo is mounted on the left side of the OSSM (the side with the USB port) then flip S6 to ON instead
**Muddy's Note:** If running Muddy-toes' fork, do not change your switches from the official firmware settings.  Leave S4 On for 800 Steps/Rotation.
2. Connect your OSSM via USB cable.
3. Download and extract the repository.
4. Install [Visual Studio Code](https://code.visualstudio.com) and the [PlatformIO](https://platformio.org/platformio-ide) extension.
5. In Visual Studio go to File > Open and select the XToys-OSSM firmware folder.
6. Open the config.h file (OSSM > src > config.h).

If you want to use Bluetooth fill out these fields:  
#define COMPILE_BLUETOOTH true  
#define COMPILE_WEBSOCKET false  
#define BLE_NAME "OSSM"

If you want to use Websockets fill out these fields:  
#define COMPILE_BLUETOOTH false  
#define COMPILE_WEBSOCKET true  
#define WIFI_SSID "your network ssid"  
#define WIFI_PSK "your network password"

If you use non-standard pins on the ESP32 this can also be adjusted here.

6. Click the PlatformIO icon in the left side bar and then click the Upload button (Project Tasks > esp32dev > Upload).

**NOTE: The firmware will not work if both COMPILE_BLUETOOTH and COMPILE_WEBSOCKET are set to true. Only use one or the other.**

# Command Structure

XToys sends JSON messages over Bluetooth, serial or websocket depending on what connection method the user chose in XToys.

The JSON is always an array of objects where each object contains an "action" key as well as additional values as needed.

ex.  

    [
        { "action": "startStreaming" },
        { "action": "move", "position": 100, "time": 200 },
        { "action": "move", "position": 0, "time": 0 }
    ]

# Supported Commands

| XToys Event                                               | Action Name        | Sample JSON                                                                                                                                                                                    |
|-----------------------------------------------------------|--------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| On Connect                                                | connected          | {<br>"action": "connected"<br>}                                                                                                                                                                |
| On homing selected                                        | home               | {<br>"action": "home",<br>"type": "sensorless", // or "sensor", or "manual"<br>"length": 100, // rod length in mm (ignored for sensorless home)<br>"side": "front" // or "back" sensor side, used for sensor home only<br>}                                                                                                                                   |
| On Speed mode selected                                    | setPattern         | {<br>"action": "setPattern",<br>"pattern": 0 // XToys only ever sends pattern 0<br>}                                                                                                           |
| On speed set to 0                                         | stop               | {<br>"action": "stop"<br>}                                                                                                                                                                     |
| On speed set to >0                                        | setSpeed           | {<br>"action": "setSpeed",<br>"speed": 50 // 0-100<br>}                                                                                                                                        |
| On upper stroke length changed                            | setDepth           | {<br>"action": "setDepth",<br>"depth": 50 // 0-100<br>}                                                                                                                                        |
| On lower stroke length changed                            | setStroke          | {<br>"action": "setStroke",<br>"stroke": 50 // 0-100<br>}                                                                                                                                      |
| On Position mode selected                                 | startStreaming     | {<br>"action": "startStreaming"<br>}                                                                                                                                                           |
| On position changed (manually or via pattern)             | move               | {<br>"action": "move",<br>"position: 50, // 0-100<br>"time": 500, // ms<br>"replace": true // whether to wipe the existing pending position commands (ex. when user has changed patterns)<br>} |
| On manual disconnect or homing cancelled                  | disable            | {<br>"action": "disable"<br>}                                                                                                                                                                  |
| On firmware flash complete and Bluetooth being configured | configureBluetooth | {<br>	"action": "configureBluetooth",<br>	"name": "OSSM" // desired Bluetooth name<br>}                                                                                                          |
| On firmware flash complete and Websocket being configured | configureWebsocket | {<br>	"action": "configureWebsocket",<br>	"ssid": "MyNetwork",<br>	"password": "mysecretpassword"<br>}                                                                                            |
| On connect and version being checked                      | version            | {<br>	"action": "version"<br>}                                                                                                                                                                  |


Actions that are in the OSSM firmware code but are not sent via XToys currently:
- "getPatternList"
- "setup"
- "retract"
- "extend"
- "setSensation"
