/**
 * ============================================================================
 *   ESP8266 3D Flight Simulator Telemetry Server + Live Debug Terminal
 * ============================================================================
 * 
 * Hardware Connections (Soldered on Perfboard):
 * - ESP8266 D5 (GPIO14, RX) <--- Raspberry Pi Pico GP16 (Pin 21, UART0 TX)
 * - ESP8266 D6 (GPIO12, TX) ---> Raspberry Pi Pico GP17 (Pin 22, UART0 RX)
 * - ESP8266 GND             <--- Raspberry Pi Pico GND (Common ground)
 * - Baud Rate: 38400 baud via SoftwareSerial (D5/D6, with auto-probe & dynamic swap)
 * - USB Serial: 115200 baud for PC monitoring & flash upload
 * 
 * Features:
 * - Dual Wi-Fi: AP ("PicoFlight-Sim", IP 192.168.4.1) & Station ("Gautam", IP 192.168.0.x)
 * - Embedded 3D airplane flight deck served from Flash PROGMEM
 * - WebSocket server (port 81) streaming 50Hz AHRS telemetry + Live Serial Console
 * - Decodes 9-DOF AHRS binary packets (0xAA 0x55), legacy packets (0xAA 0xBB), and ASCII JSON/logs
 * - Forward commands (TARE, CALIB, SWAP) to Pico over D6/D5
 * - Auto-probes RX pin between GPIO14 (D5) and GPIO12 (D6) until packets arrive
 * - Periodic diagnostics heartbeat (packet rate, pin status, link health)
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <SoftwareSerial.h>
#include "web_assets.h"

// --- Pin Definitions for Soldered Perfboard (D6=RX, D5=TX @ 38400 baud) ---
#define PICO_RX_PIN D5 // D5 (GPIO14, RX) <--- Pico GP16 (Pin 21, UART0 TX)
#define PICO_TX_PIN D6 // D6 (GPIO12, TX) ---> Pico GP17 (Pin 22, UART0 RX)
#define PICO_BAUD   38400

SoftwareSerial picoSerial(PICO_RX_PIN, PICO_TX_PIN);
bool isPinSwapped = false;
bool pinLocked = true;

// --- Wi-Fi Access Point Configuration ---
const char* AP_SSID = "PicoFlight-Sim";
const char* AP_PASS = "flightattitude";
IPAddress apIP(192, 168, 4, 1);
IPAddress netMsk(255, 255, 255, 0);

// --- Wi-Fi Station Configuration (Connects to user's home router) ---
const char* STA_SSID = "Gautam";
const char* STA_PASS = "980135635";

// --- Servers ---
ESP8266WebServer server(80);
WebSocketsServer webSocket(81);

// Status indicator LED (Active LOW on NodeMCU/ESP-12)
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// --- Telemetry Binary Packet Structure ---
#pragma pack(push, 1)
// Standard 23-byte Binary Telemetry Packet (matches perfboard & FC_UI_esp8266)
struct TelemetryPacket {
    uint8_t header1; // 0xAA
    uint8_t header2; // 0xBB
    int16_t roll;    // deg * 100
    int16_t pitch;   // deg * 100
    int16_t yaw;     // deg * 100
    int16_t dt;      // loop microseconds
    int16_t alt_cm;  // cm AGL
    int16_t vsi_cms; // cm/s vertical speed
    int16_t prs_hpa10;// hPa * 10
    int16_t roll_pid;
    int16_t pitch_pid;
    int16_t yaw_pid;
    uint8_t checksum;// XOR of 20 payload bytes (roll through yaw_pid)
};
#pragma pack(pop)

uint8_t rxBuffer[64];
size_t rxIdx = 0;

// Telemetry & Diagnostic state
String latestTelemetry = "{\"r\":0,\"p\":0,\"y\":0,\"status\":\"waiting for Pico\"}";
uint32_t rxByteCount = 0;
uint32_t rxPacketCount = 0;
uint32_t lastSerialRxMs = 0;
uint32_t lastDiagBroadcastMs = 0;

void setupPicoSerial() {
    picoSerial.begin(PICO_BAUD);
    Serial.printf("[INIT] Pico UART listening on RX=D5(GPIO14), TX=D6(GPIO12) @ %d baud\n", PICO_BAUD);
}

void swapPins(bool manual = false) {
    isPinSwapped = !isPinSwapped;
    picoSerial.end();
    if (isPinSwapped) {
        picoSerial.begin(PICO_BAUD, SWSERIAL_8N1, D5, D6);
    } else {
        picoSerial.begin(PICO_BAUD, SWSERIAL_8N1, D6, D5);
    }
    String msg = "[ESP8266] Swapped pins: RX=" + String(isPinSwapped ? "D5(GPIO14)" : "D6(GPIO12)") +
                 ", TX=" + String(isPinSwapped ? "D6(GPIO12)" : "D5(GPIO14)");
    Serial.println(msg);
    String json = "{\"sys\":\"" + msg + "\"}";
    webSocket.broadcastTXT(json);
}

// Helper: Escape string for JSON
String escapeJson(const String& s) {
    String out = "";
    out.reserve(s.length() + 16);
    for (unsigned int i = 0; i < s.length(); i++) {
        char c = s[i];
        if (c == '"') out += "\\\"";
        else if (c == '\\') out += "\\\\";
        else if (c == '\n') out += "\\n";
        else if (c == '\r') out += "\\r";
        else if (c == '\t') out += "\\t";
        else if (c >= 32 && c <= 126) out += c;
    }
    return out;
}

// --- HTTP Request Handlers ---
void handleRoot() {
    server.send_P(200, "text/html", INDEX_HTML);
}

void handleMobile() {
    server.send_P(200, "text/html", MOBILE_HTML);
}

void handleTelemetry() {
    server.send(200, "application/json", latestTelemetry);
}

void handleTare() {
    picoSerial.println("TARE");
    Serial.println("TARE");
    server.send(200, "application/json", "{\"status\":\"Tare sent to Pico\"}");
}

void handleCalib() {
    picoSerial.println("CALIB");
    Serial.println("CALIB");
    server.send(200, "application/json", "{\"status\":\"Calib sent to Pico\"}");
}

void handleSwap() {
    swapPins(true);
    pinLocked = true;
    String msg = isPinSwapped ? "RX=D6(GPIO12), TX=D5(GPIO14)" : "RX=D5(GPIO14), TX=D6(GPIO12)";
    server.send(200, "application/json", "{\"status\":\"" + msg + "\"}");
}

void handleNotFound() {
    server.send(404, "text/plain", "404: Not Found");
}

// --- WebSocket Event Handler ---
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            break;

        case WStype_CONNECTED: {
            IPAddress ip = webSocket.remoteIP(num);
            String staIpStr = WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "Not Connected";
            String currentPinStr = isPinSwapped ? "D6(GPIO12)" : "D5(GPIO14)";
            String welcome = "{\"sys\":\"[ESP8266] Web client #" + String(num) + " connected (" + ip.toString() + "). " +
                             "Pico UART: RX=" + currentPinStr + " @ 38400 baud. STA IP: " + staIpStr + "\"}";
            webSocket.sendTXT(num, welcome);

            if (latestTelemetry.length() > 0) {
                webSocket.sendTXT(num, latestTelemetry);
            }
            break;
        }

        case WStype_TEXT:
            if (length > 0) {
                String cmd = String((const char*)payload);
                cmd.trim();

                if (cmd.equalsIgnoreCase("SWAP")) {
                    swapPins(true);
                    pinLocked = true;
                } else {
                    // Forward command to Raspberry Pi Pico over SoftwareSerial & Hardware UART
                    picoSerial.println(cmd);
                    Serial.println(cmd);
                    String echoMsg = "{\"cmd\":\"Sent to Pico: " + escapeJson(cmd) + "\"}";
                    webSocket.broadcastTXT(echoMsg);
                }
            }
            break;

        default:
            break;
    }
}

// Process incoming byte through 23-byte binary packet decoder
void processIncomingByte(uint8_t c) {
    rxByteCount++;
    lastSerialRxMs = millis();

    if (rxPacketCount == 0 && (rxByteCount <= 50 || rxByteCount % 50 == 0)) {
        Serial.printf("[RAW #%u: 0x%02X]\n", rxByteCount, c);
    }

    // 1. Sync byte 1: 0xAA
    if (rxIdx == 0) {
        if (c == 0xAA) {
            rxBuffer[0] = 0xAA;
            rxIdx = 1;
        }
        return;
    }

    // 2. Sync byte 2: 0xBB
    if (rxIdx == 1) {
        if (c == 0xBB) {
            rxBuffer[1] = 0xBB;
            rxIdx = 2;
        } else if (c == 0xAA) {
            rxIdx = 1; // Double 0xAA
        } else {
            rxIdx = 0; // Reset
        }
        return;
    }

    // 3. Accumulate payload & checksum (bytes 2 through 22)
    rxBuffer[rxIdx++] = c;

    if (rxIdx >= sizeof(TelemetryPacket)) {
        rxIdx = 0;
        TelemetryPacket* pkt = (TelemetryPacket*)rxBuffer;

        uint8_t calc_chk = 0;
        uint8_t* payload = (uint8_t*)&pkt->roll;
        for (int i = 0; i < 20; i++) {
            calc_chk ^= payload[i];
        }

        if (calc_chk == pkt->checksum) {
            rxPacketCount++;
            pinLocked = true; // Pin confirmed!

            float r   = pkt->roll / 100.0f;
            float p_a = pkt->pitch / 100.0f;
            float y   = pkt->yaw / 100.0f;
            float dt  = pkt->dt / 1000000.0f;
            float alt = pkt->alt_cm / 100.0f;
            float vsi = pkt->vsi_cms / 100.0f;
            float prs = pkt->prs_hpa10 / 10.0f;

            char json[260];
            snprintf(json, sizeof(json),
                "{\"r\":%.2f,\"p\":%.2f,\"y\":%.2f,\"alt\":%.2f,\"vsi\":%.2f,\"prs\":%.1f,\"dt\":%.5f,"
                "\"dr\":%.2f,\"dp\":%.2f,\"dy\":%.2f,\"rp\":%d,\"pp\":%d,\"yp\":%d}",
                r, p_a, y, alt, vsi, prs, dt, alt, vsi, prs,
                pkt->roll_pid, pkt->pitch_pid, pkt->yaw_pid);

            latestTelemetry = String(json);
            webSocket.broadcastTXT(json);

            if (rxPacketCount <= 10 || (rxPacketCount % 50 == 0)) {
                Serial.printf("[PKT #%u] R=%.1f P=%.1f Y=%.1f Alt=%.2fm Prs=%.1f\n",
                              rxPacketCount, r, p_a, y, alt, prs);
            }
        } else {
            if (rxPacketCount == 0 || rxPacketCount % 50 == 0) {
                Serial.printf("[BAD CHK] Calc=0x%02X != Pkt=0x%02X\n", calc_chk, pkt->checksum);
            }
        }
    }
}

void setup() {
    // Hardware USB Serial for flashing and PC debug console
    Serial.begin(115200);

    // SoftwareSerial connection to Raspberry Pi Pico (RX=D5, TX=D6 @ 38400 baud)
    setupPicoSerial();

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // LED off

    // Dual Wi-Fi mode: SoftAP + Station
    WiFi.mode(WIFI_AP_STA);

    // 1. Setup Access Point ("PicoFlight-Sim")
    WiFi.softAPConfig(apIP, apIP, netMsk);
    if (strlen(AP_PASS) > 0) {
        WiFi.softAP(AP_SSID, AP_PASS);
    } else {
        WiFi.softAP(AP_SSID);
    }

    // 2. Connect to user's home Wi-Fi station ("Gautam")
    if (strlen(STA_SSID) > 0) {
        WiFi.begin(STA_SSID, STA_PASS);
        Serial.printf("\n[WIFI] Connecting to Station SSID '%s'...\n", STA_SSID);
    }

    // Configure HTTP Webserver routes
    server.on("/", HTTP_GET, handleRoot);
    server.on("/m", HTTP_GET, handleMobile);
    server.on("/mobile", HTTP_GET, handleMobile);
    server.on("/mobile/settings", HTTP_GET, handleMobile);
    server.on("/hud", HTTP_GET, handleMobile);
    server.on("/telemetry", HTTP_GET, handleTelemetry);
    server.on("/tare", HTTP_GET, handleTare);
    server.on("/calib", HTTP_GET, handleCalib);
    server.on("/swap", HTTP_GET, handleSwap);
    server.onNotFound(handleNotFound);
    server.begin();

    // Configure WebSocket server
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);

    digitalWrite(LED_BUILTIN, LOW); // LED ON when server is ready
    Serial.printf("\n[INIT] ESP8266 Flight Server Ready @ 80 MHz!\n");
    Serial.printf(" - SoftAP IP: %s (SSID: %s)\n", WiFi.softAPIP().toString().c_str(), AP_SSID);
    Serial.printf(" - Pico UART: RX=D5(GPIO14), TX=D6(GPIO12) @ %d baud\n", PICO_BAUD);
    Serial.printf(" - Desktop HUD: http://%s/\n", WiFi.localIP().toString().c_str());
    Serial.printf(" - Mobile HUD:  http://%s/mobile\n", WiFi.localIP().toString().c_str());
}

void loop() {
    webSocket.loop();
    server.handleClient();

    // 1. Read incoming stream from Raspberry Pi Pico over SoftwareSerial (D6=RX, D5=TX)
    while (picoSerial.available()) {
        uint8_t c = (uint8_t)picoSerial.read();
        processIncomingByte(c);
    }

    // 2. Also accept data/commands from PC USB Serial (if connected)
    while (Serial.available()) {
        uint8_t c = (uint8_t)Serial.read();
        processIncomingByte(c);
    }

    // 3. Diagnostics heartbeat every 1.0 second
    if (millis() - lastDiagBroadcastMs > 1000) {
        lastDiagBroadcastMs = millis();
        bool hasRecentRx = (millis() - lastSerialRxMs < 2000);
        String staIp = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "Connecting...";
        String currentPinStr = isPinSwapped ? "D5(GPIO14)" : "D6(GPIO12)";

        Serial.printf("[STATUS] Pkts=%u, Bytes=%u | RX=%s%s | IP: %s\n",
                      rxPacketCount, rxByteCount, currentPinStr.c_str(),
                      pinLocked ? " [LOCKED]" : " [SEARCHING]", staIp.c_str());

        String diagJson = "{\"diag\":{"
            "\"bytes\":" + String(rxByteCount) + ","
            "\"pkts\":" + String(rxPacketCount) + ","
            "\"pin\":\"RX: " + currentPinStr + " @ 38400" + (pinLocked ? " [LOCKED]" : " [SEARCHING]") + "\","
            "\"sta_ip\":\"" + staIp + "\","
            "\"active\":" + (hasRecentRx ? "true" : "false") +
        "}}";
        webSocket.broadcastTXT(diagJson);
    }
}
