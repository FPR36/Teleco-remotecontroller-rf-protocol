#pragma once
// RFM95B (SX1276) als FSK-zender in continuous mode, voor ESPHome op een Wemos D1 mini.
// Vervangt de Si4432-code uit het B4R-project (reprogregs / transmitdata / formatstream).
#include "esphome.h"
#include <string>

namespace sx {

// ---- Pinnen (GPIO-nummers van de D1 mini, zoals in je B4R-code) ----
static const uint8_t PIN_MISO  = 5;   // D1
static const uint8_t PIN_SCK   = 4;   // D2
static const uint8_t PIN_MOSI  = 12;  // D6
static const uint8_t PIN_NSS   = 2;   // D4  (idle hoog, ok bij boot)
static const uint8_t PIN_RESET = 13;  // D7  (RFM95 NRESET is actief laag)
static const uint8_t PIN_DATA  = 15;  // D8  -> DIO2 (data-ingang in continuous TX)

static const char *TAG = "sx1276";

// ---- SPI bit-bang, mode 0, MSB eerst ----
inline uint8_t spi_byte(uint8_t out) {
  uint8_t in = 0;
  for (int i = 0; i < 8; i++) {
    digitalWrite(PIN_MOSI, (out & 0x80) ? HIGH : LOW);
    out <<= 1;
    digitalWrite(PIN_SCK, HIGH);
    in = (in << 1) | (digitalRead(PIN_MISO) ? 1 : 0);
    digitalWrite(PIN_SCK, LOW);
  }
  return in;
}

inline void write_reg(uint8_t reg, uint8_t val) {
  digitalWrite(PIN_NSS, LOW);
  spi_byte(reg | 0x80);
  spi_byte(val);
  digitalWrite(PIN_NSS, HIGH);
}

inline uint8_t read_reg(uint8_t reg) {
  digitalWrite(PIN_NSS, LOW);
  spi_byte(reg & 0x7F);
  uint8_t v = spi_byte(0x00);
  digitalWrite(PIN_NSS, HIGH);
  return v;
}

// ---- Initialisatie: 868,3 MHz, FSK, +-70 kHz, continuous mode ----
inline bool init() {
  pinMode(PIN_NSS, OUTPUT);   digitalWrite(PIN_NSS, HIGH);
  pinMode(PIN_SCK, OUTPUT);   digitalWrite(PIN_SCK, LOW);
  pinMode(PIN_MOSI, OUTPUT);  digitalWrite(PIN_MOSI, LOW);
  pinMode(PIN_MISO, INPUT);
  pinMode(PIN_DATA, INPUT);   // pas OUTPUT als de chip in TX zit

  // Hardware reset: >100 us laag, daarna loslaten en 5 ms wachten
  pinMode(PIN_RESET, OUTPUT);
  digitalWrite(PIN_RESET, LOW);
  delay(1);
  pinMode(PIN_RESET, INPUT);
  delay(10);

  uint8_t ver = read_reg(0x42);
  ESP_LOGI(TAG, "RegVersion = 0x%02X (verwacht 0x12)", ver);
  if (ver != 0x12) return false;

  write_reg(0x01, 0x00);  // sleep, FSK/OOK-modus (alleen in sleep te wisselen)
  delay(2);
  write_reg(0x01, 0x01);  // standby
  write_reg(0x31, 0x00);  // RegPacketConfig2: continuous mode
  write_reg(0x14, 0x08);  // RegOokPeak: BitSyncOn = 0 (asynchrone data op DIO2)
  write_reg(0x04, 0x04);  // Fdev MSB \_ 1147 * 61,035 Hz = ~70 kHz
  write_reg(0x05, 0x7B);  // Fdev LSB /
  write_reg(0x06, 0xD9);  // Frf = 868,3 MHz
  write_reg(0x07, 0x13);
  write_reg(0x08, 0x33);
  write_reg(0x0A, 0x09);  // PaRamp 40 us, geen Gaussian shaping
  write_reg(0x09, 0x8F);  // PA_BOOST, +17 dBm
  return true;
}

inline bool wait_ready(uint8_t mask) {
  uint32_t t0 = millis();
  while ((read_reg(0x3E) & mask) != mask) {   // RegIrqFlags1
    if (millis() - t0 > 20) return false;
    yield();
  }
  return true;
}

inline bool tx_on() {
  write_reg(0x01, 0x03);                      // TX
  bool ok = wait_ready(0xA0);                 // ModeReady + TxReady
  digitalWrite(PIN_DATA, LOW);
  pinMode(PIN_DATA, OUTPUT);                  // nu pas aansturen
  return ok;
}

inline void tx_off() {
  pinMode(PIN_DATA, INPUT);                   // eerst loslaten
  write_reg(0x01, 0x01);                      // standby
}

// ---- Frame-opbouw (1-op-1 uit formatstream) ----
// button: 1=dicht 2=kwart 3=midden 4=vol open 5=rechts 7=stop 8=links
inline bool build_frame(uint8_t button, uint8_t *s) {
  switch (button) {
    case 1: s[0]=0x0B; s[1]=0x34; s[2]=0x95; s[3]=0x03; s[4]=0x6B; break;
    case 2: s[0]=0x24; s[1]=0x7C; s[2]=0xB1; s[3]=0x12; s[4]=0x6D; break;
    case 3: s[0]=0x0F; s[1]=0x94; s[2]=0x84; s[3]=0x12; s[4]=0x2D; break;
    case 4: s[0]=0x50; s[1]=0xC0; s[2]=0xA4; s[3]=0xA0; s[4]=0x2F; break;
    case 5: s[0]=0xB8; s[1]=0x78; s[2]=0x00; s[3]=0x02; s[4]=0x84; break;
    case 7: s[0]=0xDA; s[1]=0x9C; s[2]=0x30; s[3]=0x10; s[4]=0x44; break;
    case 8: s[0]=0x58; s[1]=0x6C; s[2]=0x8C; s[3]=0x50; s[4]=0x18; break;
    default: return false;
  }
  const uint8_t c1 = 0x00, c2 = 0x08, c3 = 0x00, c4 = 0x00;
  s[1] = (s[1] & 0xFC) | c1;
  s[2] = (s[2] & 0x3F) | ((c2 << 4) & 0xFF);
  s[3] = (s[3] & 0xF3) | ((c3 << 2) & 0xFF);
  s[5] = c4;
  s[6] = button;
  uint16_t sum = 0;
  for (int i = 0; i < 7; i++) sum += s[i];
  s[7] = (0x100 - (sum & 0xFF)) & 0xFF;
  return true;
}

// ---- Eén frame: 2 ms hoog, daarna per bit een flank en 500 us (0) of 1000 us (1) ----
inline void send_frame(const uint8_t *s) {
  uint32_t t = micros();
  auto wait = [&t](uint32_t us) {
    t += us;
    while ((int32_t)(micros() - t) < 0) { }
  };
  bool level = true;
  digitalWrite(PIN_DATA, HIGH);
  wait(2000);
  for (int i = 0; i < 8; i++) {
    uint8_t ch = s[i];
    for (int b = 0; b < 8; b++) {
      level = !level;
      digitalWrite(PIN_DATA, level ? HIGH : LOW);
      wait(500);
      if (ch & 0x80) wait(500);
      ch <<= 1;
    }
  }
  digitalWrite(PIN_DATA, LOW);
}

// ---- Volledig commando verzenden (4 herhalingen, ~250 ms) ----
inline void send(uint8_t button) {
  uint8_t s[8];
  if (!build_frame(button, s)) {
    ESP_LOGW(TAG, "onbekende knop %d", button);
    return;
  }
  ESP_LOGI(TAG, "TX knop %d: %02X %02X %02X %02X %02X %02X %02X %02X",
           button, s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7]);
  if (!tx_on()) ESP_LOGW(TAG, "TX niet klaar binnen 20 ms");
  for (int rep = 0; rep < 4; rep++) {
    send_frame(s);
    delayMicroseconds(7500);
    yield();
  }
  delay(1);
  tx_off();
}

// ---- Commandonaam (MQTT / HA) naar knopnummer ----
inline uint8_t button_from_name(const std::string &x) {
  if (x == "Closed")      return 1;
  if (x == "Quarter")     return 2;
  if (x == "Midposition") return 3;
  if (x == "FullOpen")    return 4;
  if (x == "MoveRight")   return 5;
  if (x == "Stop")        return 7;
  if (x == "MoveLeft")    return 8;
  return 0;
}

inline void send_name(const std::string &x) {
  uint8_t b = button_from_name(x);
  if (b) send(b);
  else ESP_LOGW(TAG, "onbekend commando: %s", x.c_str());
}

// ---- Meetfunctie: 1 s draaggolf op +dev, 1 s op -dev (geen data) ----
// Met een RTL-SDR zie je twee pieken, ~140 kHz uit elkaar, gecentreerd op 868,3 MHz.
inline void carrier_test() {
  if (!tx_on()) ESP_LOGW(TAG, "TX niet klaar binnen 20 ms");
  digitalWrite(PIN_DATA, HIGH);
  delay(1000);
  digitalWrite(PIN_DATA, LOW);
  delay(1000);
  tx_off();
  ESP_LOGI(TAG, "carrier test klaar, RegOpMode=0x%02X", read_reg(0x01));
}

}  // namespace sx
