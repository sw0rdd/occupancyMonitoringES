#include "arduino_secrets.h"
#include <ArduinoBLE.h>
#include <EEPROM.h>
#include <WiFiS3.h>
#include <ArduinoHttpClient.h>

const char* ssid = "Seif";
const char* password = "12345678";
const char* serverAddress = "192.168.59.211";
int serverPort = 3000;
const char* resource = "/api/occupancy/update";

WiFiClient wifiClient;
HttpClient client = HttpClient(wifiClient, serverAddress, serverPort);

// Motion count variables corrisponding to the 3 zones
int motionCount1 = 0;
int motionCount2 = 0;
int motionCount3 = 0;

// Bluetooth UUIDs (Universal Unique Identifiers) for the UART service and TX characteristic
const char* uartServiceUUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
const char* txCharUUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

unsigned long previousMillis = 0; // Track the time since the last switch
const long switchInterval = 10000; // Interval to switch between Picos, 10 seconds
const long interval = 1000; // Interval to send data to cloud, 1 second

String picoDeviceAddresses[] = {"one-uart", "two-uart", "tre-uart"};
int currentPicoIndex = 0;
BLEDevice currentPicoAdvertiser; // Track the currently connected peripheral(Pico device)

bool isBLEConnected = false; // Track if we're currently connected to a device

// EEPROM addresses for motionCount1, motionCount2 & motionCount3
const int address1 = 0; // Use address 0 for motionCount1
const int address2 = sizeof(motionCount1); // Use the next address block for motionCount2
const int address3 = address2 + sizeof(motionCount2); // New address for motionCount3

void saveCounts() {
  EEPROM.put(address1, motionCount1);
  EEPROM.put(address2, motionCount2);
  EEPROM.put(address3, motionCount3);
}

void loadCounts() {
  EEPROM.get(address1, motionCount1);
  EEPROM.get(address2, motionCount2);
  EEPROM.get(address3, motionCount3);
}

void resetCounts() {
  motionCount1 = 0;
  motionCount2 = 0;
  motionCount3 = 0;
  saveCounts();
}

void setup() {
  Serial.begin(9600);
  while (!Serial);
  delay(5000);


  loadCounts(); // Load the counts from EEPROM
  // resetCounts(); // uncomment when you want to reset counters stored in EEPROM

  // Initialize WiFi
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
 
  if (!BLE.begin()) { // forum 'solution' to known issue
    Serial.println("Reseting...");
    static const char RESET[] = "AT+RESET\n";
    Serial2.write(RESET, sizeof(RESET) - 1);
    delay(2000);
    NVIC_SystemReset();
  }

  BLE.scanForUuid(uartServiceUUID); // Start scanning for the UART service
}

void loop() {

  unsigned long currentMillis = millis();

  // Check if it's time to switch Pico devices
  if (currentMillis - previousMillis >= switchInterval) {
    previousMillis = currentMillis;

    // If connected, disconnect before switching
    if (isBLEConnected && currentPicoAdvertiser) {
      currentPicoAdvertiser.disconnect();
      isBLEConnected = false;
      Serial.println("Disconnected from current device, switching...");
      delay(1000); // Delay before rescanning
    }

    // Move to the next device
    currentPicoIndex = (currentPicoIndex + 1) % 3; // Now cycles through 0, 1, 2
    Serial.print("Switching to Pico ");
    Serial.println(currentPicoIndex + 1);

    // Restart scanning for the next device
    BLE.stopScan();
    BLE.scanForUuid(uartServiceUUID);
  }

  // If not connected, try to connect to the next Pico
  if (!isBLEConnected) {
    BLEDevice picoAdvertiser = BLE.available(); // Check if a device is available
    if (picoAdvertiser) {

      // Check if the found device is the current Pico we want to connect to
      if (picoAdvertiser.localName() == picoDeviceAddresses[currentPicoIndex]) {
        Serial.println("Device found, trying to connect...");
        if (connectToPico(picoAdvertiser, currentPicoIndex)) {
          Serial.println("Connected, data received, and switched.");
          isBLEConnected = true; // Mark as connected
          currentPicoAdvertiser = picoAdvertiser; // Track the current peripheral
        } else {
          Serial.println("////////////////////////////////////\n\n\n///////////    OBS!!!!Failed to connect.          \n\n\n ////////////////////////////////////");
          isBLEConnected = false; // Ensure isConnected is explicitly set to false on failure
          currentPicoAdvertiser = BLEDevice(); // Reset currentPeripheral to a default, unconnected state
        }
      }
    }
  }
}

bool connectToPico(BLEDevice& picoAdvertiser, int picoIndex) {
  if (picoAdvertiser.connect()) {
    Serial.println("Connected to device");
    delay(500);
    if (exploreUartService(picoAdvertiser, picoIndex)) {
      picoAdvertiser.disconnect();
      Serial.println("Disconnected");
      return true;
    }
  }
  return false;
}

// Function to explore the UART(Universal Asynchronous Receiver-Transmitter) service and subscribe to the TX characteristic
bool exploreUartService(BLEDevice& picoAdvertiser, int picoIndex) {
  if (!picoAdvertiser.discoverAttributes()) return false;

  delay(500); // Delay after discovering services

  BLEService uartService = picoAdvertiser.service(uartServiceUUID); // Find the UART service
  if (!uartService) return false;

  BLECharacteristic txCharacteristic = uartService.characteristic(txCharUUID);
  if (!txCharacteristic) return false;

  if (txCharacteristic.canSubscribe()) {
    delay(300); // Delay before subscribing
    txCharacteristic.subscribe();
    Serial.println("Subscribed to TX Characteristic");

    delay(1000); // Delay after subscribing and before reading data

    unsigned long startTime = millis(); // millis gives us the current time in milliseconds since the board started running
    const unsigned long timeout = 10000; // 5 seconds timeout for receiving data

    while (picoAdvertiser.connected() && (millis() - startTime < timeout)) { // basically, stay in this loop for 5 seconds
      // Check if the value of the characteristic has been updated, if it has been, read the value,
      // otherwise, continue waiting for the timeout period to expire before moving on to the next Pico
      // This is so we only read the value of the characteristic once per connection
      if (txCharacteristic.valueUpdated()) {
        uint8_t buffer[4]; // Keep this as is for raw byte reading
        if (txCharacteristic.readValue(buffer, sizeof(buffer)) == sizeof(buffer)) {
          // Change to interpret as signed int
          int detections_increment = *((int*)buffer); // Cast the buffer to signed int
          // Logic to safely adjust motionCount variables
          if (picoIndex == 0) {
            if (detections_increment < 0 && abs(detections_increment) > motionCount1) {
              motionCount1 = 0;
            } else {
              motionCount1 += detections_increment;
            }
          } else if (picoIndex == 1) {
            if (detections_increment < 0 && abs(detections_increment) > motionCount2) {
              motionCount2 = 0;
            } else {
              motionCount2 += detections_increment;
            }
          } else if (picoIndex == 2) {
            if (detections_increment < 0 && abs(detections_increment) > motionCount3) {
              motionCount3 = 0;
            } else {
              motionCount3 += detections_increment;
            }
          }
          Serial.println("Recieved incremental update from Pico " + String(picoIndex + 1) + ". Increment: " + String(detections_increment));
         
          static unsigned long lastSaveTime = 0;
          if (millis() - lastSaveTime > 60000) { // Every 60 seconds
            saveCounts();
            Serial.println("Saving counters to EEPROM");
            lastSaveTime = millis();
          }          
          startTime = millis(); // Reset the timer on successful data reception
        }
        break;
      }
    }
    txCharacteristic.unsubscribe();
    delay(500); // Delay after unsubscribing

    if (WiFi.status() == WL_CONNECTED) {
      String postData = "{"
                        "\"entry\":" + String(motionCount1) + 
                        ", \"secondFloor\":" + String(motionCount2) + 
                        ", \"groundFloor\":" + String(motionCount3) +
                        "}"; 
                        
      Serial.print("Postdata to send: ");
      Serial.println(postData);

      client.beginRequest();
      client.post(resource);
      client.sendHeader("Content-Type", "application/json");
      client.sendHeader("Content-Length", postData.length());
      client.sendHeader("x-api-key", "684RZydFtpG1997");
      client.beginBody();
      client.print(postData);
      client.endRequest();

      int statusCode = client.responseStatusCode();
      String response = client.responseBody();

      Serial.print("HTTP Response code: ");
      Serial.println(statusCode);
      Serial.print("Response: ");
      Serial.println(response);
    } else {
      Serial.println("WiFi Not Connected");
    }

    return true;
  } else {
    Serial.println("Cannot subscribe to TX Characteristic.");
    return false;
  }
}
