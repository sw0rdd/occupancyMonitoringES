#include <ArduinoBLE.h>
#include <Arduino_FreeRTOS.h>
#include <WiFiS3.h>
#include <ArduinoHttpClient.h>
#include "arduino_secrets.h"

// Bluetooth UUIDs for the UART service and TX characteristic
const char *uartServiceUUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
const char *txCharUUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

String picoDeviceAddress1 = "one-uart";
String picoDeviceAddress2 = "tre-uart";

// Track the connection status of the two Pico devices
bool isBLEConnected1 = false;
bool isBLEConnected2 = false; 

BLEDevice targetPicoAdvertiser; // Track the currently connected peripheral (Pico device)

volatile int motionCount1 = 0; // Data received from the first Pico by BLETask1
volatile int motionCount2 = 0; // Data received from the second Pico by BLETask2

SemaphoreHandle_t bleSemaphore; // Semaphore for managing BLE task access

// WiFi credentials
const char *ssid = SECRET_SSID;
const char *password = SECRET_OPTIONAL_PASS;

// Function prototypes for the FreeRTOS tasks
static void BLETask1(void *pvParameters);
static void BLETask2(void *pvParameters);
static void WiFiTask(void *pvParameters);

void setup()
{
  delay(4000);

  Serial.begin(9600);
  while (!Serial)
    ; // Wait for serial port to connect

  delay(1000);
  //   Serial.println(xPortGetFreeHeapSize());

  // Initialize the semaphore
  bleSemaphore = xSemaphoreCreateBinary();
  // Immediately give the semaphore so the first task can take it
  xSemaphoreGive(bleSemaphore);

  //   Serial.println(xPortGetFreeHeapSize());

  // Create two BLE tasks for two different Picos
  xTaskCreate(BLETask1, "BLETask1", 512, (void *)1, 1, NULL);

  //   Serial.println(xPortGetFreeHeapSize());

  xTaskCreate(BLETask2, "BLETask2", 512, (void *)2, 1, NULL);

  //   Serial.println(xPortGetFreeHeapSize());

  delay(5000);

  BaseType_t wifiTaskStatus;

  wifiTaskStatus = xTaskCreate(
      WiFiTask,   
      "WiFiTask", 
      512,        
      NULL,       
      1,          
      NULL        
  );

  if (wifiTaskStatus != pdPASS)
  {
    Serial.println("     Failed to start WiFi task!");
  }
  else
  {
    Serial.println("     WiFi task started successfully.");
  }

  Serial.println(xPortGetFreeHeapSize());

  Serial.println("Starting scheduler... V3");
  vTaskStartScheduler();
}

static void WiFiTask(void *pvParameters)
{
  (void)pvParameters;
  Serial.println("     WiFiTask started, connecting to WiFi...");

  // Initiate WiFi connection
  WiFi.begin(ssid, password);

  while (true)
  {
    // Check WiFi connection status every 20 seconds
    vTaskDelay(pdMS_TO_TICKS(30000));

    if (WiFi.status() == WL_CONNECTED)
    {
      WiFiClient wifiClient;
      HttpClient client(wifiClient, "192.168.59.211", 3000);

      String postData = "{\"entry\":" + String(motionCount1) + ",\"secondFloor\":" + String(motionCount2) + "}";
      Serial.print("     Trying to POST:  ");
      Serial.print(postData);

      client.beginRequest();
      client.post("/api/occupancy/update");
      client.sendHeader("Content-Type", "application/json");
      client.sendHeader("Content-Length", postData.length());
      client.beginBody();
      client.print(postData);
      client.endRequest();

      // Reading the response
      int statusCode = client.responseStatusCode();
      String response = client.responseBody();

      // Output the HTTP response data
      Serial.print("     HTTP Response code: ");
      Serial.println(statusCode);
      Serial.print("     Response: ");
      Serial.println(response);
    }
    else
    {
      Serial.println("     WiFi Not Connected");
      WiFi.begin(ssid, password);
    }
  }
}

static void BLETask1(void *pvParameters)
{
  while (1)
  {
    if (!isBLEConnected1)
    {
      if (xSemaphoreTake(bleSemaphore, portMAX_DELAY))
      {
        isBLEConnected1 = connectAndReadFromPico(picoDeviceAddress1, 1);
        if (isBLEConnected1)
        {
          Serial.println(" ");
        }
        xSemaphoreGive(bleSemaphore); // Release access
        isBLEConnected1 = false;      // Reset connection flag
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20000)); // Wait some time before trying again
  }
}

static void BLETask2(void *pvParameters)
{
  while (1)
  {
    if (!isBLEConnected2)
    {
      if (xSemaphoreTake(bleSemaphore, portMAX_DELAY))
      {
        isBLEConnected2 = connectAndReadFromPico(picoDeviceAddress2, 2);
        if (isBLEConnected2)
        {
          Serial.println(" ");
        }
        xSemaphoreGive(bleSemaphore);
        isBLEConnected2 = false; // Reset connection flag
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20000));
  }
}

bool connectAndReadFromPico(String picoDeviceAddress, int picoNumber)
{
  if (!BLE.begin())
  {
    Serial.println("Reseting...");
    static const char RESET[] = "AT+RESET\n";
    Serial2.write(RESET, sizeof(RESET) - 1);
    delay(2000);
    NVIC_SystemReset();
  }

  Serial.println("Scanning for " + picoDeviceAddress + "...");
  BLE.scanForUuid(uartServiceUUID); // Start scanning for devices with the specified UUID

  bool deviceFound = false;
  BLEDevice device;

  // Try to find the device
  while (!deviceFound)
  {
    device = BLE.available();
    if (device && device.localName() == picoDeviceAddress)
    {
      deviceFound = true;
      BLE.stopScan(); // Stop scanning as we've found the device
      Serial.println(picoDeviceAddress + " found, attempting to connect...");
    }
  }

  if (device.connect())
  {
    Serial.println("Connected to " + picoDeviceAddress);

    if (exploreUartService(device, picoNumber))
    {
      Serial.println("Data read from " + picoDeviceAddress);
      device.disconnect();
      Serial.println("Disconnected from " + picoDeviceAddress);
      return true; 
    }
    else
    {
      Serial.println("Failed to read data from " + picoDeviceAddress);
      device.disconnect();
    }
  }
  else
  {
    Serial.println("Failed to connect to " + picoDeviceAddress);
  }

  return false;
}

bool exploreUartService(BLEDevice &picoAdvertiser, int picoNumber)
{
  if (!picoAdvertiser.discoverAttributes())
    return false;

  BLEService uartService = picoAdvertiser.service(uartServiceUUID);
  if (!uartService)
    return false;

  BLECharacteristic txCharacteristic = uartService.characteristic(txCharUUID);
  if (!txCharacteristic)
    return false;

  if (txCharacteristic.canSubscribe())
  {
    txCharacteristic.subscribe();
    Serial.println("Subscribed to TX Characteristic");

    unsigned long startTime = millis();
    const unsigned long timeout = 5000; // 5 seconds timeout for receiving data

    while (picoAdvertiser.connected() && (millis() - startTime < timeout))
    {
      if (txCharacteristic.valueUpdated())
      {
        uint8_t buffer[4];
        if (txCharacteristic.readValue(buffer, sizeof(buffer)) == sizeof(buffer))
        {
          int detections_increment = *((int *)buffer);
          if (picoNumber == 1)
          {
            motionCount1 += detections_increment; // Update motionCount1
          }
          else if (picoNumber == 2)
          {
            motionCount2 += detections_increment; // Update motionCount2
          }
          Serial.println(" Pico " + String(picoNumber) + " increment: " + String(detections_increment));
          startTime = millis(); // Reset the timer on successful data reception
        }
      }
    }
    txCharacteristic.unsubscribe();
    return true;
  }
  else
  {
    Serial.println("Cannot subscribe to TX Characteristic.");
    return false;
  }
}

// ignore this since we are using FreeRTOS
void loop()
{
}
