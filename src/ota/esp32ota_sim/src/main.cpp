/**
 * Project:     Control template for the Pololu 3Pi+  with OTA
 * Author/Modified by:      Jonathan Pu
 * Created:     october 2023
 */

// ================================================================================
// Libraries
// ================================================================================
#include <Arduino.h>
#include <WiFi.h>
#include "BasicOTA.hpp"
#include <tinycbor.h>
#include "codegen.h"
#include <ArduinoJson.h>

// ================================================================================
// Variable definitions
// ================================================================================
#define SSID "Robotat"
#define PASSWORD "iemtbmcit116"

BasicOTA OTA; // Object for Over-The-Air (OTA) Updates

// ================================================================================
// Definitions
// ================================================================================
uint8_t uart_send_buffer[32] = {0};              // buffer CBOR
static const unsigned int control_time_ms = 100; // período de muestreo del control

// Constants for WiFi connection and communication with Robotat
const unsigned robot_id = 1; // TAG number here
const char *ssid = "Robotat";
const char *password = "iemtbmcit116";
const char *host = "192.168.50.200";
const int port = 1883;
char msg2robotat[] = "{\"dst\":1,\"cmd\":1,\"pld\":100}";
WiFiClient client;
StaticJsonDocument<512> doc;
String msg = "";
char c;
int t1 = 0;
int ti = 60;
int t2 = 180;

// control parameters and variables
double temp[2];
double q[4];
volatile float phi_ell = 0;
volatile float phi_r = 0;
double WHEEL_RADIUS = (0.32 / 2);
double DISTANCE_FROM_CENTER = (0.96 / 2);

volatile float x, y, z, n, ex, ey, ez, roll, pitch, yaw, v, w;
float goal_x = 1.5; // PLACEHOLDER FOR THE GOAL THAT WILL BE MODIFIED FROM THE GUI
float goal_y = -1.5;

// ================================================================================
// Task for encoding and sending wheel speeds
// ================================================================================
void encode_send_wheel_speeds_task(void *p_params)
{
  // Control frequency settings
  TickType_t last_control_time;
  const TickType_t control_freq_ticks = pdMS_TO_TICKS(control_time_ms);

  // Initialize last control time
  last_control_time = xTaskGetTickCount();

  while (1)
  {
    // Wait for the control time period
    vTaskDelayUntil(&last_control_time, control_freq_ticks);

    // Encode wheel speeds using CBOR and send them over Serial2
    TinyCBOR.Encoder.init(uart_send_buffer, sizeof(uart_send_buffer));
    TinyCBOR.Encoder.create_array(2);
    TinyCBOR.Encoder.encode_float(phi_ell);
    TinyCBOR.Encoder.encode_float(phi_r);
    TinyCBOR.Encoder.close_container();
    Serial2.write(TinyCBOR.Encoder.get_buffer(), TinyCBOR.Encoder.get_buffer_size());
  }
}
// ================================================================================
// Task for connecting to Robotat and receiving data
// ================================================================================

void connect2robotat_task(void *p_params)
{

  while (1)
  {
    // Check for available data from Robotat
    if (client.available())
    {
      // Read and process data from Robotat
      client.write(msg2robotat);
      while (t1 < t2)
      {
        c = client.read();
        msg = msg + c;
        t1 = t1 + 1;
        if (c == '}')
          t1 = t2;
      }

      // Deserialize received JSON data
      DeserializationError err = deserializeJson(doc, msg);
      if (err == DeserializationError::Ok)
      {
        // Extract data from JSON
        x = doc["data"][0].as<double>();
        y = doc["data"][1].as<double>();
        z = doc["data"][2].as<double>();
        n = doc["data"][3].as<double>();
        ex = doc["data"][4].as<double>();
        ey = doc["data"][5].as<double>();
        ez = doc["data"][6].as<double>();

        // Serial.print("x = ");
        // Serial.print(x);
      }
      else
      {
        Serial.print("deserializeJson() returned ");
        Serial.println(err.c_str());
      }

      // Reset variables for the next iteration
      msg = "";
      t1 = 0;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Delay to control the loop frequency
  }
}

// ================================================================================
// Control
// ================================================================================

void control_algorithm_task(void *p_params)
{
  while (1)
  {

    q[0] = n;
    q[1] = ex;
    q[2] = ey;
    q[3] = ez;

    yaw = atan2(2 * (q[1] * q[2] + q[0] * q[3]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));

    // MARKER OFFSET DEPENDING ON ROBOT ID
    if (robot_id == 8)
    {
      yaw = yaw + 3.05;
    } 
    if (robot_id == 7)
    {
      yaw = yaw - 1.5708;
    }
    if (robot_id == 1)
    {
      yaw = yaw;
    }
   
    control(goal_x, goal_y, x, y, yaw, temp);

    v = temp[0];
    w = temp[1];
    phi_ell = (v - w * DISTANCE_FROM_CENTER) / WHEEL_RADIUS; // rad/s;
    phi_r = (v + w * DISTANCE_FROM_CENTER) / WHEEL_RADIUS;   // rad/s;

    float limit = 300.0;
    if (phi_ell > limit)
    {
      phi_ell = 0.0;
    }
    if (phi_ell < -limit)
    {
      phi_ell = 0.0;
    }
    if (phi_r > limit)
    {
      phi_r = 0.0;
    }
    if (phi_r < -limit)
    {
      phi_r = 0.0;
    }

    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// ================================================================================
// Setup function
// ================================================================================

void setup()
{

  Serial.begin(115200);  // ***NO MODIFICAR***
  Serial2.begin(115200); // ***NO MODIFICAR***
  TinyCBOR.init();       // ***NO MODIFICAR***
  Serial.println("Startup");
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  OTA.begin(); // Setup settings

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // WiFi+Robotat
  if (robot_id < 10)
  {
    msg2robotat[25] = 48 + robot_id;
  }

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println("Connected to the WiFi network");
  Serial.println(WiFi.localIP());

  Serial.println("Connecting to Robotat");
  if (client.connect(host, port))
  {
    Serial.println("Connected");
    while (client.available())
    {
      client.read();
    }
    client.write(msg2robotat);
  }
  else
    Serial.println("Connection failed");

  // Create tasks for parallel execution
  xTaskCreate(encode_send_wheel_speeds_task, "encode_send_wheel_speeds_task", 1024 * 2, NULL, configMAX_PRIORITIES - 3, NULL);
  xTaskCreate(connect2robotat_task, "connect2robotat_task", 1024 * 4, NULL, configMAX_PRIORITIES - 1, NULL);
  xTaskCreate(control_algorithm_task, "control_algorithm_task", 1024 * 2, NULL, configMAX_PRIORITIES - 2, NULL);
}

// ================================================================================
// Loop function (for handling OTA updates)
// ================================================================================

void loop()
{
  OTA.handle();
}
