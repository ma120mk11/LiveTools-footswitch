
#define _WEBSOCKETS_LOGLEVEL_ 5

#include <Arduino.h>
#include <WiFiNINA.h>
#include <SPI.h>
#include <ArduinoHttpClient.h>
// #include <WebSocketsClient.h>

#include <WebSocketClient.h>
#include "secrets.h"

#define NBR_OF_BUTTONS 2

// Modes
#define WEBSOCKET 1
#define API 2

#define MODE API

class Button
{
public:
    Button()
    {
    }
    String id = "test";
};

/////// Wifi Settings ///////
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

// char serverAddress[] = "192.168.43.249:8000";  // server address
char serverAddress[] = "192.168.43.249"; // server address
int port = 8000;

WiFiClient wifi;
WebSocketClient socket = WebSocketClient(wifi, serverAddress, port);
HttpClient client = HttpClient(wifi, serverAddress, port);

int status = WL_IDLE_STATUS;
unsigned long lastMillis = 0;

const int buttonPin = 4;
const int ledPin = LED_BUILTIN;

bool prevBtnValue = 0;

bool buttonValues[NBR_OF_BUTTONS] = {};

String FS_ID = "arduino-test";

void printWifiVersion()
{
    // Print firmware version on the module
    String fv = WiFi.firmwareVersion();
    String latestFv;
    Serial.print("Firmware version installed: ");
    Serial.println(fv);

    latestFv = WIFI_FIRMWARE_LATEST_VERSION;

    // Print required firmware version
    Serial.print("Latest firmware version available : ");
    Serial.println(latestFv);

    // Check if the latest version is installed
    Serial.println();
    if (fv >= latestFv)
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
}

void printWiFiStatus()
{
    // Prints WiFI status to serial monitor

    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // IP Address
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);
    Serial.println("Signal strenght: " + String(WiFi.RSSI()));
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
    // WiFi.beginAP
    // WiFi.config():
    Serial.print("Connecting to ");
    Serial.println(ssid); // print the network name (SSID);
    while (status != WL_CONNECTED)
    {
        // Connect to WPA/WPA2 network:
        // status = WiFi.begin(ssid, pass);
        status = WiFi.begin(ssid, pass);
    }
}

void setup()
{
    Serial.begin(38400);
    // Serial.begin(9600);
    while (!Serial)
    {
        ; // Wait for serial port to connect
    }

    Serial.println("Starting setup... Mode: " + MODE);

    pinMode(ledPin, OUTPUT);          // use the LED as an output
    pinMode(buttonPin, INPUT_PULLUP); // use button pin as an input

    printWifiVersion();
    connectWiFi();
    printWiFiStatus();
}

void connectWebsocket()
{
    Serial.println("Starting WebSocket client...");
    socket.connectionKeepAlive();
    socket.begin("/footswitch/arduino-test");

    if (socket.connected())
    {
        Serial.println("Connected to LiveTools Server (Websocket Mode)");

        socket.beginMessage(TYPE_TEXT);
        socket.print("{\"type\":\"config\",\"data\":{\"fs_id\":\"" + FS_ID + "\"}}");
        socket.endMessage();
    }
}

void loop()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("Network disconnected!");
        connectWiFi();
    }
    if (wifi.available()) {
        Serial.println("Wifi: " + wifi.available());
    }

    if (millis() - lastMillis > 5000) {
        Serial.println("Signal strenght: " + String(WiFi.RSSI()));
        lastMillis = millis();
    }

    if (MODE == WEBSOCKET) {

        connectWebsocket();

        while (socket.connected())
        {
            // Serial.println("WiFi status: " + String(WiFi.status()));
            bool buttonValue = !digitalRead(buttonPin);
            // Serial.print("Btn value: ");
            // Serial.println(buttonValue);

            if (buttonValue != prevBtnValue)
            {
                // Serial.print("Btn value: ");
                // Serial.println(buttonValue);

                String msg = "{\"type\":\"btn-change\",\"data\":{\"fs_id\":\"" + FS_ID + "\",\"btn_id\":0,\"state\":" + String(buttonValue) + "}}";
                Serial.println(msg);

                socket.beginMessage(TYPE_TEXT);
                socket.print(msg);
                socket.endMessage();

                // check if a message is available to be received
                // int messageSize = client.parseMessage();

                // if (messageSize > 0) {
                //   Serial.println("Received a message:");
                //   Serial.println(client.readString());
                // }
            }
            prevBtnValue = buttonValue;
            // delay(100);
        }
        Serial.println("disconnected");
    }

    else if (MODE == API) {
        bool buttonValue = !digitalRead(buttonPin);
        if (buttonValue != prevBtnValue)
        {
            String body = "{\"fs_id\":\"" + FS_ID + "\",\"btn_id\":0,\"state\":" + String(buttonValue) + "}";
            Serial.println(body);

            String contentType = "application/json";
            client.post("/footswitch/btn-change", contentType, body);
            int statusCode = client.responseStatusCode();
            String response = client.responseBody();
            Serial.println("Status code: " + String(statusCode));
            Serial.println("Response: " + response);
        }
        prevBtnValue = buttonValue;
    }

}