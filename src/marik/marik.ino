#include <Arduino.h>
#include <ESP_I2S.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "startup_greeting.h"

constexpr uint8_t OLED_SCL_PIN = 1;
constexpr uint8_t OLED_SDA_PIN = 2;
constexpr uint32_t OLED_BUS_HZ = 100000;
constexpr uint32_t OLED_RETRY_MS = 3000;
constexpr uint8_t MPU_SCL_PIN = 5;
constexpr uint8_t MPU_SDA_PIN = 6;
constexpr uint8_t TOUCH_PIN = 4;
constexpr int TOUCH_ACTIVE_LEVEL = HIGH;
constexpr int8_t AUDIO_BCLK_PIN = 8;
constexpr int8_t AUDIO_LRC_PIN = 9;
constexpr int8_t AUDIO_DATA_PIN = 7;
constexpr uint32_t AUDIO_SAMPLE_RATE_HZ = 16000;

constexpr uint32_t FRAME_INTERVAL_MS = 40;
constexpr uint32_t STARTUP_DURATION_MS = 1200;
constexpr uint32_t ANGRY_DURATION_MS = 2000;
constexpr uint32_t HAPPY_DURATION_MS = 1850;
constexpr uint32_t TOUCH_DEBOUNCE_MS = 35;
constexpr uint32_t MOTION_SAMPLE_MS = 20;
constexpr uint32_t MPU_RETRY_MS = 2000;
constexpr uint32_t SHAKE_WINDOW_MS = 240;
constexpr float SHAKE_THRESHOLD_G = 1.0f;
constexpr float STRONG_SHAKE_G = 1.50f;
constexpr float SHAKE_THRESHOLD_DPS = 300.0f;
constexpr float STRONG_SHAKE_DPS = 450.0f;

constexpr uint8_t MPU_CONFIG = 0x1A;
constexpr uint8_t MPU_GYRO_CONFIG = 0x1B;
constexpr uint8_t MPU_ACCEL_CONFIG = 0x1C;
constexpr uint8_t MPU6500_ACCEL_CONFIG_2 = 0x1D;
constexpr uint8_t MPU_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t MPU_PWR_MGMT_1 = 0x6B;
constexpr uint8_t MPU_WHO_AM_I = 0x75;
constexpr uint8_t MPU6050_ID = 0x68;
constexpr uint8_t MPU6500_ID = 0x70;
constexpr float MPU_ACCEL_COUNTS_PER_G = 8192.0f;
constexpr float MPU_GYRO_COUNTS_PER_DPS = 65.5f;

#ifndef MARIK_OLED_SH1106
#define MARIK_OLED_SH1106 0  // Set to 1 for a module with an SH1106 controller.
#endif

#if MARIK_OLED_SH1106
using OledDisplay = U8G2_SH1106_128X64_NONAME_F_HW_I2C;
#else
using OledDisplay = U8G2_SSD1306_128X64_NONAME_F_HW_I2C;
#endif

OledDisplay display(
    U8G2_R0, U8X8_PIN_NONE, OLED_SCL_PIN, OLED_SDA_PIN);
bool oledReady = false;
uint32_t lastOledAttemptAt = 0;

// A timed expression returns to Idle when its animation ends.
enum class Face { Startup, Idle, Angry, Happy };

// A three-axis measurement in g for acceleration or degrees/s for angular rate.
struct Axes {
  float x;
  float y;
  float z;
};

// Acceleration and angular rate from one MPU-6050 register read.
struct MotionSample {
  Axes acceleration;
  Axes angularRate;
};

Face face = Face::Startup;
uint32_t faceStartedAt = 0;
uint32_t lastFrameAt = 0;

bool touchRaw = false;
bool touchStable = false;
uint32_t touchChangedAt = 0;

uint8_t mpuAddress = 0;
uint8_t mpuReadFailures = 0;
uint32_t lastMpuAttemptAt = 0;
uint32_t lastMotionSampleAt = 0;
Axes baseline{};
bool baselineReady = false;
bool impulsePending = false;
uint32_t firstImpulseAt = 0;

// Keep declarations with custom types visible to Arduino's sketch preprocessor.
bool readMotion(MotionSample &sample);
float magnitudeSquared(const Axes &axes);
void setFace(Face next, uint32_t now);

bool beginDisplay() {
  if (!Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN, OLED_BUS_HZ)) {
    Serial.println("OLED: could not start I2C on SDA GPIO2 and SCL GPIO1.");
    return false;
  }

  for (uint8_t address = 0x3C; address <= 0x3D; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() != 0) {
      continue;
    }

    // U8g2 takes the shifted address, while Wire uses the 7-bit address.
    display.setI2CAddress(address << 1);
    display.setBusClock(OLED_BUS_HZ);
    display.begin();
    Serial.print("OLED: I2C reply at 0x");
    Serial.println(address, HEX);
    return true;
  }

  Serial.println("OLED: no reply at 0x3C or 0x3D. Check power, SDA GPIO2 and SCL GPIO1.");
  return false;
}

bool readMpu(uint8_t address, uint8_t reg, uint8_t *data, size_t length) {
  Wire1.beginTransmission(address);
  Wire1.write(reg);
  if (Wire1.endTransmission(false) != 0 ||
      Wire1.requestFrom(address, length) != length) {
    return false;
  }

  for (size_t index = 0; index < length; ++index) {
    data[index] = Wire1.read();
  }
  return true;
}

bool writeMpu(uint8_t address, uint8_t reg, uint8_t value) {
  Wire1.beginTransmission(address);
  Wire1.write(reg);
  Wire1.write(value);
  return Wire1.endTransmission() == 0;
}

uint8_t beginMpu() {
  if (!Wire1.begin(MPU_SDA_PIN, MPU_SCL_PIN, 100000)) {
    Serial.println("MPU: could not start I2C on SDA GPIO6 and SCL GPIO5.");
    return 0;
  }

  Wire1.setTimeOut(15);
  // AD0 changes the I2C address independently of the chip identity.
  for (uint8_t address = 0x68; address <= 0x69; ++address) {
    uint8_t identity = 0;
    if (!readMpu(address, MPU_WHO_AM_I, &identity, 1)) {
      continue;
    }
    if (identity != MPU6050_ID && identity != MPU6500_ID) {
      Serial.printf("MPU: unexpected WHO_AM_I 0x%02X at address 0x%02X.\n",
                    unsigned(identity), unsigned(address));
      continue;
    }
    if (writeMpu(address, MPU_PWR_MGMT_1, 0x01) &&
        writeMpu(address, MPU_CONFIG, 0x03) &&
        writeMpu(address, MPU_GYRO_CONFIG, 0x08) &&   // ±500 degrees/s
        writeMpu(address, MPU_ACCEL_CONFIG, 0x08) &&  // ±4 g
        // On the MPU-6500, the accelerometer has its own low-pass filter setting.
        (identity != MPU6500_ID || writeMpu(address, MPU6500_ACCEL_CONFIG_2, 0x03))) {
      Serial.printf("%s ready at 0x%02X.\n",
                    identity == MPU6500_ID ? "MPU-6500" : "MPU-6050", unsigned(address));
      return address;
    }
    Serial.printf("MPU: configuration failed at 0x%02X.\n", unsigned(address));
  }

  Serial.println("MPU unavailable. Check power, GND, SCL GPIO5 and SDA GPIO6.");
  return 0;
}

float decodeAxis(const uint8_t *bytes, float countsPerUnit) {
  const uint16_t bits = (uint16_t(bytes[0]) << 8) | bytes[1];
  const int32_t counts = bits < 0x8000 ? bits : int32_t(bits) - 0x10000;
  return counts / countsPerUnit;
}

bool readMotion(MotionSample &sample) {
  uint8_t bytes[14];
  if (!readMpu(mpuAddress, MPU_ACCEL_XOUT_H, bytes, sizeof(bytes))) {
    return false;
  }

  sample.acceleration = {decodeAxis(bytes, MPU_ACCEL_COUNTS_PER_G),
                         decodeAxis(bytes + 2, MPU_ACCEL_COUNTS_PER_G),
                         decodeAxis(bytes + 4, MPU_ACCEL_COUNTS_PER_G)};
  // Bytes 6 and 7 are temperature, between acceleration and angular rate.
  sample.angularRate = {decodeAxis(bytes + 8, MPU_GYRO_COUNTS_PER_DPS),
                        decodeAxis(bytes + 10, MPU_GYRO_COUNTS_PER_DPS),
                        decodeAxis(bytes + 12, MPU_GYRO_COUNTS_PER_DPS)};
  return true;
}

float magnitudeSquared(const Axes &axes) {
  return axes.x * axes.x + axes.y * axes.y + axes.z * axes.z;
}

void printMotionStatus() {
  MotionSample sample;
  if (mpuAddress == 0 || !readMotion(sample)) {
    Serial.println("MPU: no motion readings.");
    return;
  }
  const Axes &a = sample.acceleration;
  const Axes &g = sample.angularRate;
  Serial.printf("MPU 0x%02X: accel %.2f %.2f %.2f g, gyro %.1f %.1f %.1f deg/s\n",
                unsigned(mpuAddress), a.x, a.y, a.z, g.x, g.y, g.z);
}

bool shakeDetected(uint32_t now) {
  if (mpuAddress == 0) {
    if (now - lastMpuAttemptAt >= MPU_RETRY_MS) {
      lastMpuAttemptAt = now;
      mpuAddress = beginMpu();
    }
    return false;
  }
  if (now - lastMotionSampleAt < MOTION_SAMPLE_MS) {
    return false;
  }
  lastMotionSampleAt = now;

  MotionSample sample;
  if (!readMotion(sample)) {
    baselineReady = false;
    impulsePending = false;
    if (++mpuReadFailures >= 3) {
      Serial.println("MPU: readings lost, retrying I2C.");
      mpuAddress = 0;
      mpuReadFailures = 0;
      lastMpuAttemptAt = now;
    }
    return false;
  }
  mpuReadFailures = 0;
  if (!baselineReady) {
    baseline = sample.acceleration;
    baselineReady = true;
    return false;
  }

  // Subtract a moving baseline so gravity and slow tilts do not count as shakes.
  const Axes &acceleration = sample.acceleration;
  const Axes impulse = {acceleration.x - baseline.x, acceleration.y - baseline.y,
                        acceleration.z - baseline.z};
  baseline.x += 0.15f * impulse.x;
  baseline.y += 0.15f * impulse.y;
  baseline.z += 0.15f * impulse.z;

  if (now - firstImpulseAt > SHAKE_WINDOW_MS) {
    impulsePending = false;
  }

  const float accelerationSquared = magnitudeSquared(impulse);
  const float rotationSquared = magnitudeSquared(sample.angularRate);
  const bool strong = accelerationSquared >= STRONG_SHAKE_G * STRONG_SHAKE_G ||
                      rotationSquared >= STRONG_SHAKE_DPS * STRONG_SHAKE_DPS;
  const bool moderate = accelerationSquared >= SHAKE_THRESHOLD_G * SHAKE_THRESHOLD_G ||
                        rotationSquared >= SHAKE_THRESHOLD_DPS * SHAKE_THRESHOLD_DPS;
  // One strong sample or two moderate samples catch short shakes on any axis.
  if (strong || (moderate && impulsePending)) {
    impulsePending = false;
    return true;
  }
  if (moderate) {
    firstImpulseAt = now;
    impulsePending = true;
  }
  return false;
}

bool newTouch(uint32_t now) {
  const bool touched = digitalRead(TOUCH_PIN) == TOUCH_ACTIVE_LEVEL;
  if (touched != touchRaw) {
    touchRaw = touched;
    touchChangedAt = now;
  }
  if (touchStable != touchRaw && now - touchChangedAt >= TOUCH_DEBOUNCE_MS) {
    touchStable = touchRaw;
    return touchStable;
  }
  return false;
}

void setFace(Face next, uint32_t now) {
  face = next;
  faceStartedAt = now;
  impulsePending = false;
}

void drawEyes(int centerY, int height) {
  const int radius = min(7, (height - 1) / 2);
  display.drawRBox(24, centerY - height / 2, 16, height, radius);
  display.drawRBox(88, centerY - height / 2, 16, height, radius);
}

void drawSmile(int y, int radius) {
  display.setClipWindow(0, y, 128, 64);
  display.drawDisc(64, y, radius);
  display.setDrawColor(0);
  display.drawDisc(64, y, max(1, radius - 5));
  display.setDrawColor(1);
  display.setMaxClipWindow();
}

void drawStartup(uint32_t elapsed) {
  if (elapsed < 360) {
    const int halfWidth = 1 + elapsed * 35 / 360;
    display.drawHLine(64 - halfWidth, 28, 2 * halfWidth + 1);
    return;
  }

  const uint32_t opening = min(elapsed - 360, uint32_t(660));
  drawEyes(28, 2 + opening * 22 / 660);
  if (opening > 220) {
    drawSmile(45, 4 + (opening - 220) * 8 / 440);
  }
}

void drawIdle(uint32_t elapsed) {
  const uint32_t blink = elapsed % 4200;
  drawEyes(28, blink >= 3920 && blink < 4080 ? 3 : 24);
  drawSmile(45, 12);
}

void drawAngry(uint32_t elapsed) {
  const int shift = int(elapsed / 80 % 3) * 2 - 2;
  const int lowerEye = 31 + int(elapsed / 110 % 2);
  display.drawTriangle(21 + shift, 16, 47 + shift, 27, 23 + shift, lowerEye + 3);
  display.drawTriangle(107 + shift, 16, 81 + shift, 27, 105 + shift, lowerEye + 3);
  display.drawLine(20 + shift, 12, 46 + shift, 23);
  display.drawLine(108 + shift, 12, 82 + shift, 23);
  display.drawLine(50 + shift, 55, 58 + shift, 47);
  display.drawHLine(58 + shift, 47, 13);
  display.drawLine(70 + shift, 47, 78 + shift, 55);
  display.drawHLine(56 + shift, 54, 17);

  const int steamY = 19 - int(elapsed % 360 / 30);
  display.drawLine(58, steamY + 4, 62, steamY);
  display.drawLine(69, steamY + 4, 73, steamY);
}

void drawHeart(int x, int y) {
  display.drawDisc(x - 2, y, 3);
  display.drawDisc(x + 2, y, 3);
  display.drawTriangle(x - 5, y, x + 5, y, x, y + 6);
}

void drawHappy(uint32_t elapsed) {
  const int bounce = abs(4 - int(elapsed / 42 % 9));
  const int eyeY = 30 - bounce;
  display.drawCircle(32, eyeY, 9, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
  display.drawCircle(96, eyeY, 9, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
  display.drawDisc(64, 45 - bounce, 13, U8G2_DRAW_LOWER_LEFT | U8G2_DRAW_LOWER_RIGHT);
  display.drawHLine(51, 45 - bounce, 27);
  drawHeart(12, 44 - int(elapsed % 600 / 18));
  drawHeart(116, 44 - int((elapsed + 300) % 600 / 18));
}

void renderFace(uint32_t elapsed) {
  static uint8_t previousFrame[128 * 64 / 8]{};
  display.clearBuffer();
  switch (face) {
    case Face::Startup: drawStartup(elapsed); break;
    case Face::Idle: drawIdle(elapsed); break;
    case Face::Angry: drawAngry(elapsed); break;
    case Face::Happy: drawHappy(elapsed); break;
  }
  // Avoid keeping the slower OLED bus busy while the face is still.
  const uint8_t *frame = display.getBufferPtr();
  if (std::memcmp(previousFrame, frame, sizeof(previousFrame)) != 0) {
    display.sendBuffer();
    std::memcpy(previousFrame, frame, sizeof(previousFrame));
  }
}

void playStartupGreeting() {
  I2SClass audio;
  audio.setPins(AUDIO_BCLK_PIN, AUDIO_LRC_PIN, AUDIO_DATA_PIN);
  if (!audio.begin(I2S_MODE_STD, AUDIO_SAMPLE_RATE_HZ, I2S_DATA_BIT_WIDTH_16BIT,
                   I2S_SLOT_MODE_STEREO)) {
    Serial.println("I2S initialization failed.");
    return;
  }

  // Let the amplifier settle after the I2S clocks start.
  delay(150);
  constexpr size_t framesPerWrite = 256;
  int16_t stereo[framesPerWrite][2];
  const int8_t *encoded = STARTUP_GREETING_DELTAS;
  int16_t sample = 0;
  for (size_t offset = 0; offset < STARTUP_GREETING_SAMPLE_COUNT; offset += framesPerWrite) {
    const size_t frames = min(STARTUP_GREETING_SAMPLE_COUNT - offset, framesPerWrite);
    for (size_t i = 0; i < frames; ++i) {
      const int8_t delta = *encoded++;
      if (delta == INT8_MIN) {
        const int8_t highByte = *encoded++;
        const uint8_t lowByte = static_cast<uint8_t>(*encoded++);
        sample = highByte * 256 + lowByte;
      } else {
        sample += delta;
      }
      // Use both slots so the amplifier can select either channel.
      stereo[i][0] = sample;
      stereo[i][1] = sample;
    }
    const size_t bytes = frames * sizeof(stereo[0]);
    if (audio.write(reinterpret_cast<uint8_t *>(stereo), bytes) != bytes) {
      Serial.println("I2S playback failed.");
      break;
    }
  }
  // Let queued DMA samples play before the I2S object is destroyed.
  delay(150);
}

void setup() {
  Serial.begin(115200);
  pinMode(TOUCH_PIN, INPUT_PULLDOWN);
  oledReady = beginDisplay();
  mpuAddress = beginMpu();
  lastOledAttemptAt = millis();
  lastMpuAttemptAt = lastOledAttemptAt;
  // I2S writes can block while DMA is full. Keep the face animation on the main task.
  if (xTaskCreate([](void *) {
        playStartupGreeting();
        vTaskDelete(nullptr);
      }, "startupGreeting", 4096, nullptr, 1, nullptr) != pdPASS) {
    Serial.println("Could not start the greeting task.");
  }
  if (oledReady) {
    setFace(Face::Startup, lastOledAttemptAt);
    lastFrameAt = faceStartedAt;
    renderFace(0);
  }
}

void loop() {
  if (!oledReady) {
    const uint32_t attemptAt = millis();
    if (attemptAt - lastOledAttemptAt < OLED_RETRY_MS) {
      return;
    }
    lastOledAttemptAt = attemptAt;
    oledReady = beginDisplay();
    if (!oledReady) {
      return;
    }
    setFace(Face::Startup, millis());
    lastFrameAt = faceStartedAt;
    renderFace(0);
  }

  const uint32_t now = millis();
  const uint32_t elapsed = now - faceStartedAt;
  if ((face == Face::Startup && elapsed >= STARTUP_DURATION_MS) ||
      (face == Face::Angry && elapsed >= ANGRY_DURATION_MS) ||
      (face == Face::Happy && elapsed >= HAPPY_DURATION_MS)) {
    setFace(Face::Idle, now);
  }

  const bool shaken = shakeDetected(now);
  if (Serial.available() > 0 && Serial.read() == '?') {
    printMotionStatus();
  }
  const bool touched = face != Face::Startup && newTouch(now);
  if ((face == Face::Idle || face == Face::Happy) && shaken) {
    Serial.println("Shake detected: angry face.");
    setFace(Face::Angry, now);
  } else if ((face == Face::Idle || face == Face::Happy) && touched) {
    setFace(Face::Happy, now);
  }

  if (now - lastFrameAt >= FRAME_INTERVAL_MS) {
    lastFrameAt = now;
    renderFace(now - faceStartedAt);
  }
}
