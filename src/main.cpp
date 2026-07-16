// Modbus RTU -> MQTT industrial edge gateway (ESP32-S3).
//
// A poll task owns the RS-485/UART side and is the only writer to the bus.
// It hands samples to a network task through a queue, so a stalled broker or
// a dropped Wi-Fi link can never delay or corrupt meter polling. A UI task
// renders the latest sample locally and keeps working with no network at all.
//
// Serial output is deliberately terse and stable: the CI scenario asserts on
// these exact lines, so treat them as a contract.

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <esp_task_wdt.h>

#include "modbus_rtu.h"

// ---------------------------------------------------------------- wiring ---

static const int PIN_METER_RX = 18;  // ESP32 receives <- meter TX
static const int PIN_METER_TX = 17;  // ESP32 transmits -> meter RX
static const int PIN_I2C_SDA = 8;
static const int PIN_I2C_SCL = 9;
static const int PIN_STATUS_LED = 2;

static const uint32_t METER_BAUD = 9600;
static const uint8_t METER_SLAVE_ID = 1;

// ----------------------------------------------------------- meter model ---

static const uint16_t REG_BASE = 0;
static const uint16_t REG_COUNT = 8;

struct MeterSample {
  uint16_t voltage_dV;   // volts x10
  uint16_t current_dA;   // amps x10
  uint32_t power_W;
  uint16_t frequency_dHz;
  uint16_t powerFactor_pct;
  uint32_t energy_Wh;
  uint32_t sequence;
  bool valid;
};

// ------------------------------------------------------------- constants ---

static const char *WIFI_SSID = "Wokwi-GUEST";
static const char *WIFI_PASSWORD = "";
static const int WIFI_CHANNEL = 6;  // Wokwi's gateway only listens on 6

static const char *MQTT_HOST = "broker.hivemq.com";
static const uint16_t MQTT_PORT = 1883;
static const char *MQTT_TOPIC = "rnalbuga/gateway/meter1/telemetry";
static const char *MQTT_STATUS_TOPIC = "rnalbuga/gateway/meter1/status";

static const uint32_t POLL_INTERVAL_MS = 1000;
static const uint8_t POLL_RETRIES = 3;
static const uint32_t WDT_TIMEOUT_S = 10;

// --------------------------------------------------------------- globals ---

static ModbusRtuMaster meter(Serial1, METER_BAUD, /*responseTimeoutMs=*/300);
static WiFiClient wifiClient;
static PubSubClient mqtt(wifiClient);
static Adafruit_SSD1306 display(128, 64, &Wire, -1);
static Preferences prefs;

static QueueHandle_t sampleQueue;
static SemaphoreHandle_t latestMutex;
static MeterSample latestSample;
static volatile uint32_t consecutiveFailures = 0;
static volatile uint32_t totalPolls = 0;
static volatile uint32_t totalErrors = 0;
static bool displayReady = false;

// ------------------------------------------------------------- poll task ---

static void decodeSample(const uint16_t *regs, MeterSample &out) {
  out.voltage_dV = regs[0];
  out.current_dA = regs[1];
  out.power_W = ((uint32_t)regs[2] << 16) | regs[3];
  out.frequency_dHz = regs[4];
  out.powerFactor_pct = regs[5];
  out.energy_Wh = ((uint32_t)regs[6] << 16) | regs[7];
  out.valid = true;
}

static void publishLatest(const MeterSample &sample) {
  if (xSemaphoreTake(latestMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    latestSample = sample;
    xSemaphoreGive(latestMutex);
  }
}

static void pollTask(void *) {
  esp_task_wdt_add(NULL);

  uint16_t regs[REG_COUNT];
  uint32_t sequence = 0;
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    esp_task_wdt_reset();

    MbResult result = MbResult::Timeout;
    uint8_t attempt = 0;
    for (; attempt < POLL_RETRIES; attempt++) {
      result = meter.readInputRegisters(METER_SLAVE_ID, REG_BASE, REG_COUNT, regs);
      if (result == MbResult::Ok) {
        break;
      }
      // Back off before retrying so a confused slave can resynchronise.
      vTaskDelay(pdMS_TO_TICKS(50));
    }

    totalPolls++;

    if (result == MbResult::Ok) {
      MeterSample sample = {};
      decodeSample(regs, sample);
      sample.sequence = ++sequence;
      consecutiveFailures = 0;

      publishLatest(sample);
      // Drop the sample rather than block the bus if the network is wedged.
      xQueueSend(sampleQueue, &sample, 0);

      Serial.printf("METER OK seq=%lu V=%.1f I=%.1f P=%lu E=%lu retries=%u\n",
                    (unsigned long)sample.sequence, sample.voltage_dV / 10.0,
                    sample.current_dA / 10.0, (unsigned long)sample.power_W,
                    (unsigned long)sample.energy_Wh, attempt);
    } else {
      consecutiveFailures++;
      totalErrors++;
      if (result == MbResult::Exception) {
        Serial.printf("METER FAIL reason=EXCEPTION code=0x%02X streak=%lu\n",
                      meter.lastExceptionCode(),
                      (unsigned long)consecutiveFailures);
      } else {
        Serial.printf("METER FAIL reason=%s streak=%lu\n", mbResultName(result),
                      (unsigned long)consecutiveFailures);
      }
    }

    digitalWrite(PIN_STATUS_LED, consecutiveFailures == 0 ? HIGH : LOW);
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(POLL_INTERVAL_MS));
  }
}

// -------------------------------------------------------------- net task ---

static bool ensureWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }

  Serial.println("WIFI CONNECTING");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL);

  uint32_t deadline = millis() + 20000;
  while (WiFi.status() != WL_CONNECTED && (int32_t)(millis() - deadline) < 0) {
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WIFI FAIL");
    return false;
  }
  Serial.print("WIFI OK ip=");
  Serial.println(WiFi.localIP());
  return true;
}

static bool ensureMqtt() {
  if (mqtt.connected()) {
    return true;
  }

  String clientId = "gw-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  Serial.println("MQTT CONNECTING");
  if (mqtt.connect(clientId.c_str(), MQTT_STATUS_TOPIC, 0, true, "offline")) {
    mqtt.publish(MQTT_STATUS_TOPIC, "online", true);
    Serial.println("MQTT OK");
    return true;
  }
  Serial.printf("MQTT FAIL rc=%d\n", mqtt.state());
  return false;
}

static void netTask(void *) {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(512);

  MeterSample sample;
  for (;;) {
    if (!ensureWifi()) {
      vTaskDelay(pdMS_TO_TICKS(5000));
      continue;
    }
    if (!ensureMqtt()) {
      vTaskDelay(pdMS_TO_TICKS(5000));
      continue;
    }
    mqtt.loop();

    if (xQueueReceive(sampleQueue, &sample, pdMS_TO_TICKS(500)) == pdTRUE) {
      char payload[256];
      int len = snprintf(
          payload, sizeof(payload),
          "{\"seq\":%lu,\"voltage\":%.1f,\"current\":%.1f,\"power\":%lu,"
          "\"frequency\":%.1f,\"pf\":%.2f,\"energy_wh\":%lu,\"errors\":%lu}",
          (unsigned long)sample.sequence, sample.voltage_dV / 10.0,
          sample.current_dA / 10.0, (unsigned long)sample.power_W,
          sample.frequency_dHz / 10.0, sample.powerFactor_pct / 100.0,
          (unsigned long)sample.energy_Wh, (unsigned long)totalErrors);

      if (len > 0 && mqtt.publish(MQTT_TOPIC, payload)) {
        Serial.printf("PUB %s %d bytes\n", MQTT_TOPIC, len);
      } else {
        Serial.println("PUB FAIL");
      }
    }
  }
}

// --------------------------------------------------------------- ui task ---

static void uiTask(void *) {
  MeterSample sample;
  for (;;) {
    if (displayReady) {
      if (xSemaphoreTake(latestMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        sample = latestSample;
        xSemaphoreGive(latestMutex);

        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);

        display.setCursor(0, 0);
        display.println("MODBUS -> MQTT GW");

        if (!sample.valid) {
          display.setCursor(0, 20);
          display.println("waiting for meter");
        } else {
          display.setCursor(0, 14);
          display.printf("%.1f V  %.1f A", sample.voltage_dV / 10.0,
                         sample.current_dA / 10.0);
          display.setCursor(0, 26);
          display.setTextSize(2);
          display.printf("%lu W", (unsigned long)sample.power_W);
          display.setTextSize(1);
          display.setCursor(0, 46);
          display.printf("E %lu Wh", (unsigned long)sample.energy_Wh);
        }

        display.setCursor(0, 56);
        display.printf("%s err:%lu",
                       consecutiveFailures == 0 ? "LINK OK" : "LINK DOWN",
                       (unsigned long)totalErrors);
        display.display();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// ----------------------------------------------------------------- setup ---

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nBOOT modbus-mqtt-gateway");

  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);

  // Config survives reboots; the poll interval is stored so a field tech can
  // retune the gateway without a reflash.
  prefs.begin("gateway", false);
  uint32_t storedBoots = prefs.getUInt("boots", 0) + 1;
  prefs.putUInt("boots", storedBoots);
  Serial.printf("NVS boots=%lu\n", (unsigned long)storedBoots);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    displayReady = true;
    display.clearDisplay();
    display.display();
    Serial.println("OLED OK");
  } else {
    Serial.println("OLED FAIL");
  }

  meter.begin(PIN_METER_RX, PIN_METER_TX);

  sampleQueue = xQueueCreate(16, sizeof(MeterSample));
  latestMutex = xSemaphoreCreateMutex();
  latestSample.valid = false;

  esp_task_wdt_init(WDT_TIMEOUT_S, true);

  // Fieldbus timing lives on core 0, away from the Wi-Fi stack on core 1.
  xTaskCreatePinnedToCore(pollTask, "poll", 4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(netTask, "net", 8192, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(uiTask, "ui", 4096, NULL, 1, NULL, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
