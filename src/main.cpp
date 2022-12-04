#include <Arduino.h>
#include <WiFiNINA.h>
#include <SPI.h>
#include <ArduinoHttpClient.h>
#include "secrets.h"

#define NBR_OF_BUTTONS 5
#define SERIAL_TIMEOUT 10000

#define pending true
#define switched true

#define TRIGGER_TYPE RISING
#define debounce 100

String FS_ID = "arduino-test";

/////// Wifi Settings ///////
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

char serverAddress[] = SERVER_IP;    // server address
int port = SERVER_PORT;

WiFiClient wifi;
HttpClient client = HttpClient(wifi, serverAddress, port);

int status = WL_IDLE_STATUS;
unsigned long lastMillis = 0;

const int ledPin = LED_BUILTIN;

bool buttonValues[NBR_OF_BUTTONS] = {};
bool initComplete = false;

volatile bool interruptStatus[NBR_OF_BUTTONS] = {!pending, !pending, !pending, !pending, !pending};
bool prevBtnValue[NBR_OF_BUTTONS] = {0, 0, 0, 0, 0};

/**
 * Interrupt enabled pins on the nano 33 iot
 *  2, 3, 9, 10, 11, 13, A1, A5, A7
 */
const int btnPin[] = { 10, 9, 11, 3, 2 };


bool switching_pending[NBR_OF_BUTTONS] = {false, false, false, false, false};
long int elapse_timer[NBR_OF_BUTTONS];

void printLine() {
    Serial.println("*************************************************");
}

void interruptHandler(int buttonId) {
    if(initComplete) {
        if (interruptStatus[buttonId] == !pending) {
            interruptStatus[buttonId] = pending;
        }
    }
}


void btn1InterruptHandler() {
    interruptHandler(0);
}
void btn2InterruptHandler() {
    interruptHandler(1);
}
void btn3InterruptHandler() {
    interruptHandler(2);
}
void btn4InterruptHandler() {
    interruptHandler(3);
}
void btn5InterruptHandler() {
    interruptHandler(4);
}


bool readButton(int id, int btnReading) {

    if (interruptStatus[id] == pending)
    {
        if (switching_pending[id] == false) {
            Serial.println("Handling pending interrupt");
            // Interrupt was raised, waiting to be handled

            // switch is pressed, so start/restart wait for button relealse, plus end of debounce process
            switching_pending[id] = true;
            
            // start elapse timing for debounce checking
            elapse_timer[id] = millis();
            return true;
        }
        else if (btnReading == LOW && (millis() - elapse_timer[id] >= debounce)) {
            Serial.println("Resetting interrupt");
            // Button has been released, and debounce period over,
            // we can now enable interrupts again
            interruptStatus[id] = !pending;
            switching_pending[id] = false;
            return false;
        }
    }

    return false; // either no press request or debounce period not elapsed
}

void printWifiVersion()
{
    // Print firmware version on the module
    String fv = WiFi.firmwareVersion();
    Serial.print("Firmware version installed: ");
    Serial.println(fv);

    // Print required firmware version
    Serial.print("Latest firmware version available : ");
    Serial.println(WIFI_FIRMWARE_LATEST_VERSION);

    // Check if the latest version is installed
    Serial.println();
    if (fv >= WIFI_FIRMWARE_LATEST_VERSION)
    {
        Serial.println("Check result: PASSED");
    }
    else
    {
        Serial.println("Check result: NOT PASSED");
        Serial.println(" - The firmware version on the module does not match the");
        Serial.println("   version required by the library, you may experience");
        Serial.println("   issues or failures.");
    }
    printLine();
}

void printWiFiStatus()
{
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);
    Serial.println("Signal strenght: " + String(WiFi.RSSI()));
    printLine();
}

void connectWiFi()
{
    if (WiFi.status() == WL_NO_MODULE)
    {
        Serial.println("ERROR: WiFi module failed");
        while (true)
            ;
    }
    WiFi.noLowPowerMode();
    WiFi.setTimeout(10000);
    Serial.print("Connecting to ");
    Serial.println(ssid); // print the network name (SSID);
    while (status != WL_CONNECTED)
    {
        // Connect to WPA/WPA2 network:
        status = WiFi.begin(ssid, pass);
    }
    printWiFiStatus();
}

void setup()
{
    pinMode(ledPin, OUTPUT);

    // Turn on the led to indicate startup
    digitalWrite(ledPin, HIGH);

    Serial.begin(38400);
    unsigned long start = millis();

    while (!Serial)
    {
        // Wait for serial port to connect

        if ((millis() - start) > SERIAL_TIMEOUT) {
            break;
        }
    }
    printLine();
    Serial.println("** Starting setup...");

    digitalWrite(ledPin, LOW);

    for (int i = 0; i < NBR_OF_BUTTONS; i++)
    {
        pinMode(btnPin[i], INPUT_PULLUP);
    }

    attachInterrupt(digitalPinToInterrupt(btnPin[0]), btn1InterruptHandler, TRIGGER_TYPE);
    attachInterrupt(digitalPinToInterrupt(btnPin[1]), btn2InterruptHandler, TRIGGER_TYPE);
    attachInterrupt(digitalPinToInterrupt(btnPin[2]), btn3InterruptHandler, TRIGGER_TYPE);
    attachInterrupt(digitalPinToInterrupt(btnPin[3]), btn4InterruptHandler, TRIGGER_TYPE);
    attachInterrupt(digitalPinToInterrupt(btnPin[4]), btn5InterruptHandler, TRIGGER_TYPE);

    printWifiVersion();
    connectWiFi();

    initComplete = true;
    printLine();

    // Turn off led to indicate setup complete
    digitalWrite(ledPin, LOW);

    Serial.println("Setup complete");
}


void postBtnPress(int buttonId, int btnValue ) {
    unsigned long start = millis();

    digitalWrite(ledPin, HIGH);

    String body = "{\"fs_id\":\"" + FS_ID + "\",\"btn_id\":" + String(buttonId) + ",\"state\":" + String(btnValue) + "}";
    Serial.println(body);

    String contentType = "application/json";
    client.post("/footswitch/btn-change", contentType, body);
    int statusCode = client.responseStatusCode();
    client.skipResponseHeaders();

    Serial.println("Process time: " + String(millis() - start) + " ms, status-code: " + String(statusCode));
    
    start = millis();

    String response = client.responseBody();
    Serial.println("Body process time: " + String(millis() - start) + "ms");
    digitalWrite(ledPin, LOW);
}


void loop()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("Network disconnected!");
        // Attempt to reconnect
        while (status != WL_CONNECTED) {
            connectWiFi();
            delay(5000);
        }
    }

    for (int id = 0; id < NBR_OF_BUTTONS; id++) {
        bool buttonValue = digitalRead(btnPin[id]);

        if (readButton(id, buttonValue)) {
            printLine();
            Serial.println("Button " + String(id) + " value: " + String(buttonValue));
            postBtnPress(id, 1);
        }
    }
}