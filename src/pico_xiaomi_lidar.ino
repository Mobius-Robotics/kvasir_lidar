// Copyright 2023-2025 KAIA.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <Adafruit_NeoPixel.h>

// On-board NeoPixel on the VCC-GND YD-RP2040, used for status indication.
Adafruit_NeoPixel pixel(1, 23, NEO_GRB + NEO_KHZ800);

// Include just the LDS class for our specific model.
#include "LDS_LDS02RR.h"

// Serial1 on the Pico Arduino Core corresponds to UART0
#define LidarSerial Serial1

// Driver pins to control motor direction.
constexpr pin_size_t LIDAR_GPIO_IN1 = 21;
constexpr pin_size_t LIDAR_GPIO_IN2 = 20;

// Enable and speed control pins for the lidar.
// NB: LIDAR_GPIO_EN is not actually used by the LDS_LDS02RR class, we've just put it for correctness.
constexpr pin_size_t LIDAR_GPIO_EN = LIDAR_GPIO_IN1;
constexpr pin_size_t LIDAR_GPIO_PWM = 22;

// Baudrate for the serial output.
constexpr uint32_t SERIAL_MONITOR_BAUD = 115200;

// PWM constants.
constexpr uint32_t LIDAR_PWM_FREQ = 10000;
constexpr int LIDAR_PWM_BITS = 11;

// LED indicator flags.
volatile bool got_data = false;
volatile bool got_serial = false;

// The LIDAR class instance itself. (spoiler alert)
LDS_LDS02RR lidar;

void setupLidar() {
  // Setup LIDAR instance and callbacks
  lidar.setScanPointCallback(lidar_scan_point_callback);
  lidar.setSerialWriteCallback(lidar_serial_write_callback);
  lidar.setSerialReadCallback(lidar_serial_read_callback);
  lidar.setMotorPinCallback(lidar_motor_pin_callback);

  // Initialize the LIDAR serial interface and begin communications.
  LidarSerial.begin(lidar.getSerialBaudRate());
  lidar.init();
}

void setup() {
  // Setup and start the lidar.
  // NB: As said above, LIDAR_GPIO_EN is not actually used by our specific class, so we'll need to initialize and set the motor pins here.
  setupLidar();
  lidar.start();
  pinMode(LIDAR_GPIO_IN1, OUTPUT);
  digitalWrite(LIDAR_GPIO_IN1, HIGH);
  pinMode(LIDAR_GPIO_IN2, OUTPUT);
  digitalWrite(LIDAR_GPIO_IN2, LOW);
}

void setup1() {
  // Wait for the USB Serial to come up.
  Serial.begin(SERIAL_MONITOR_BAUD);
  while (!Serial) {}
  got_serial = true;
}

int lidar_serial_read_callback() {
  int ch = LidarSerial.read();
  got_data = got_data || (ch != -1);
  if (!got_data) pixel.setPixelColor(0, pixel.Color(150, 0, got_serial ? 150 : 0));
  else pixel.setPixelColor(0, pixel.Color(0, 150, got_serial ? 150 : 0));

  return ch;
}

size_t lidar_serial_write_callback(const uint8_t *buffer, size_t length) {
  return LidarSerial.write(buffer, length);
}

struct __attribute__((packed)) ScanPoint {
  uint16_t angle_deg;
  uint16_t distance_mm;
};

static constexpr size_t RING_BUFFER_SIZE = 1024;
static volatile size_t ring_head = 0;
static volatile size_t ring_tail = 0;
static struct ScanPoint ring_buffer[RING_BUFFER_SIZE];
static constexpr uint8_t SOF[2] = { 0xA5, 0x5A };

void ring_push(const struct ScanPoint &p) {
  size_t next = (ring_head + 1) % RING_BUFFER_SIZE;
  if (next == ring_tail) {
    // buffer full
    return;
  }
  ring_buffer[ring_head] = p;
  ring_head = next;
}

bool ring_pop(struct ScanPoint &p) {
  if (ring_tail == ring_head) {
    // buffer empty
    return false;
  }
  p = ring_buffer[ring_tail];
  ring_tail = (ring_tail + 1) % RING_BUFFER_SIZE;
  return true;
}

void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality, bool scan_completed) {
  ScanPoint p{ static_cast<uint16_t>(angle_deg), static_cast<uint16_t>(distance_mm) };
  ring_push(p);
}

void loop1() {
  ScanPoint p;

  // Buffer utilized to avoid two calls to Serial.write; SOF is always in the first two bites.
  uint8_t send_buffer[sizeof(SOF) + sizeof(ScanPoint)];
  memcpy(send_buffer, SOF, sizeof(SOF));

  // NB: this "busy wait" loop is here to ensure max throughput; yielding or something like that would probably be more energy-efficient.
  for (;;) {
    if (ring_pop(p)) {
      memcpy(send_buffer + sizeof(SOF), &p, sizeof(p)); // Copy scan point to correct offset.
      Serial.write(send_buffer, sizeof(send_buffer)); // Send it!
    }
  }
}

void lidar_motor_pin_callback(float value, LDS::lds_pin_t lidar_pin) {
  int actualPin = (lidar_pin == LDS::LDS_MOTOR_EN_PIN) ? LIDAR_GPIO_EN : LIDAR_GPIO_PWM;

  // Cast float to int so that:
  //   - exact negatives map to the enum states
  //   - any 0 ≤ value ≤ 1 maps to 0 (VALUE_PWM)
  auto state = value < 1.0 ? static_cast<LDS::lds_pin_state_t>(static_cast<int>(value)) : LDS::VALUE_PWM;

  switch (state) {
    case LDS::DIR_INPUT:
      pinMode(actualPin, INPUT);
      break;

    case LDS::DIR_OUTPUT_CONST:
      pinMode(actualPin, OUTPUT);
      break;

    case LDS::DIR_OUTPUT_PWM:
      pinMode(actualPin, OUTPUT);
      analogWriteFreq(LIDAR_PWM_FREQ);
      analogWriteResolution(LIDAR_PWM_BITS);
      break;

    case LDS::VALUE_LOW:
      digitalWrite(actualPin, LOW);
      break;

    case LDS::VALUE_HIGH:
      digitalWrite(actualPin, HIGH);
      break;

    case LDS::VALUE_PWM:
    default:  // Any non-negative value (including fractional 0..1) lands here
      analogWrite(actualPin, ((1 << LIDAR_PWM_BITS) - 1) * value);
      break;
  }
}

void loop() {
  lidar.loop();
  pixel.show();
}
