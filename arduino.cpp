#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <HX711_ADC.h>
#include <cstring> // For C-style string functions

// BLE UUIDs
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// HX711 pins and configuration
#define LEAD_DOUT 26
#define LEAD_CLK 27
#define TRAIL_DOUT 32
#define TRAIL_CLK 33
const int CALIBRATION_FACTOR = -7050; // Adjust based on your scale calibration
const float WEIGHT_THRESHOLD = 1000.0; // Minimum weight to detect presence (grams, adjust as needed)
const float SAMPLE_INTERVAL = 0.0125; // 12.5 ms for 80 SPS

HX711_ADC leadScale(LEAD_DOUT, LEAD_CLK); // Lead side scale
HX711_ADC trailScale(TRAIL_DOUT, TRAIL_CLK); // Trail side scale

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool recording = false;
String tempo = ""; // Keeping String for tempo to match web app expectations
int backFrames = 0;
int downFrames = 0;
const float frameTime = 0.033; // 33 ms per frame
unsigned long countdownStart = 0;
std::vector<float> times;
std::vector<float> leadWeights;
std::vector<float> trailWeights;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("Device connected");
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("Device disconnected");
        pServer->startAdvertising();
        recording = false;
        times.clear();
        leadWeights.clear();
        trailWeights.clear();
    }
};

class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        // Use const char* directly from the characteristic value
        const char* rxData = pCharacteristic->getValue().c_str();
        if (rxData && *rxData) { // Check if data exists and is not empty
            Serial.print("Received Value: ");
            Serial.println(rxData);

            // Parse the tempo using C-style string functions
            char* slashPos = strchr(rxData, '/');
            if (slashPos) {
                *slashPos = '\0'; // Temporarily null-terminate at '/'
                backFrames = atoi(rxData);
                downFrames = atoi(slashPos + 1);
                *slashPos = '/'; // Restore the original string

                // Update tempo as String for compatibility
                tempo = String(backFrames) + "/" + String(downFrames);

                Serial.print("Tempo set: Back ");
                Serial.print(backFrames);
                Serial.print(", Down ");
                Serial.println(downFrames);
            }
        }
    }
};

void setup() {
    Serial.begin(115200);

    // Initialize HX711 scales with 80 SPS (RATE pin must be HIGH)
    leadScale.begin();
    leadScale.start(2000, true); // Stabilize and tare, 2000ms stabilization for 80 SPS
    leadScale.setCalFactor(CALIBRATION_FACTOR); // Set calibration factor
    if (leadScale.getTareTimeoutFlag() || leadScale.getSignalTimeoutFlag()) {
        Serial.println("Lead HX711 timeout, check wiring");
        while (1);
    }

    trailScale.begin();
    trailScale.start(2000, true); // Stabilize and tare
    trailScale.setCalFactor(CALIBRATION_FACTOR); // Set calibration factor
    if (trailScale.getTareTimeoutFlag() || trailScale.getSignalTimeoutFlag()) {
        Serial.println("Trail HX711 timeout, check wiring");
        while (1);
    }

    // Initialize BLE
    BLEDevice::init("ESP32_PRESSURE");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService* pService = pServer->createService(SERVICE_UUID);

    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
    );

    pCharacteristic->addDescriptor(new BLE2902());
    pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

    pService->start();

    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);
    BLEDevice::startAdvertising();
    Serial.println("Waiting for BLE connection...");
}

void loop() {
    if (deviceConnected) {
        if (leadScale.update() && trailScale.update()) {
            float leadWeight = leadScale.getData();
            float trailWeight = trailScale.getData();

            // Debug print to verify 80 SPS (approx. 12.5 ms intervals)
            static unsigned long lastTime = 0;
            unsigned long currentTime = millis();
            if (currentTime - lastTime >= 10) {
                Serial.print("Sample Time: ");
                Serial.println(currentTime - lastTime);
                lastTime = currentTime;
            }

            if (leadWeight > WEIGHT_THRESHOLD && trailWeight > WEIGHT_THRESHOLD) {
                if (countdownStart == 0) {
                    countdownStart = millis();
                    Serial.println("Weight detected, starting countdown");
                    pCharacteristic->setValue("WEIGHT_DETECTED");
                    pCharacteristic->notify();
                }

                unsigned long elapsed = millis() - countdownStart;
                if (elapsed >= 5000) {
                    // 5 seconds reached, start swing and send first beep
                    pCharacteristic->setValue("START_SWING");
                    pCharacteristic->notify();
                    Serial.println("5 seconds reached, start swing");

                    // Record for backswing + downswing + 1s
                    float backDelay = backFrames * frameTime * 1000; // ms
                    float downDelay = downFrames * frameTime * 1000; // ms
                    float totalRecordTime = backDelay + downDelay + 1000; // ms after start swing

                    recording = true;
                    unsigned long recordEnd = millis() + totalRecordTime;
                    while (millis() < recordEnd) {
                        recordData();
                        delay(SAMPLE_INTERVAL * 1000); // 12.5 ms delay for 80 SPS
                    }

                    // Send top beep after backswing delay
                    delay(backDelay);
                    pCharacteristic->setValue("TOP_BEEP");
                    pCharacteristic->notify();
                    Serial.println("Top beep");

                    // Send impact beep after downswing delay
                    delay(downDelay);
                    pCharacteristic->setValue("IMPACT_BEEP");
                    pCharacteristic->notify();
                    Serial.println("Impact beep");

                    sendData();
                    resetRecording();
                } else if (elapsed >= 4000 && !recording) {
                    // Start recording at 4 seconds
                    recording = true;
                    Serial.println("Started recording at 4 seconds");
                }
            } else {
                if (countdownStart != 0) {
                    // Stepped off early
                    pCharacteristic->setValue("STEPPED_OFF");
                    pCharacteristic->notify();
                    Serial.println("Stepped off early, restarting");
                    resetRecording();
                }
            }
        }
    }
    delay(10); // Small delay to prevent overwhelming the loop
}

void recordData() {
    if (recording) {
        float elapsed = (millis() - countdownStart - 4000) / 1000.0; // Time since recording start (4s mark)
        times.push_back(elapsed);
        leadWeights.push_back(leadScale.getData());
        trailWeights.push_back(trailScale.getData());
    }
}

void sendData() {
    String dataStr = "(";
    for (float t : times) {
        dataStr += String(t, 4) + ",";
    }
    dataStr.remove(dataStr.length() - 1); // Remove last comma
    dataStr += ");(";
    for (float w : leadWeights) {
        dataStr += String(w, 1) + ",";
    }
    dataStr.remove(dataStr.length() - 1);
    dataStr += ");(";
    for (float t : times) {
        dataStr += String(t, 4) + ",";
    }
    dataStr.remove(dataStr.length() - 1);
    dataStr += ");(";
    for (float w : trailWeights) {
        dataStr += String(w, 1) + ",";
    }
    dataStr.remove(dataStr.length() - 1);
    dataStr += ")";

    pCharacteristic->setValue(dataStr.c_str());
    pCharacteristic->notify();
    Serial.println("Data sent");
}

void resetRecording() {
    countdownStart = 0;
    recording = false;
    times.clear();
    leadWeights.clear();
    trailWeights.clear();
}