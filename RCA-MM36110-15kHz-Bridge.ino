/* RCA MM36110 15kHz-Bridge 
 *
 * Based on Martin Hejnfelt's TA1276AN I2C Bridge
 * and bsafta's fork. 
 * basfta fork: https://github.com/bsafta/ta1276an-i2c-bridge 
 * original: https://github.com/skumlos/ta1276an-i2c-bridge 
 * 
 * This software uses an Arduino Nano V3 and sits between a RCA MM36110's 
 * main microcontroller and the frequency detection IC U22407, a Toshiba TA1300AN. 
 * Acts as a man-in-the-middle to modify an errant I2C signal that causes
 * visual glitches when 15kHz content is displayed over VGA. This removes the infamous 
 * "garbage pixel artifacts" mentioned on the CRT database.
 *
 * Suppression is qualified by the TA1300AN's own H FREQ DET read (read byte 1, bits 6:0).
 * The 0x80 write to reg 0x00 (33.75kHz) is only replaced with 0x00 (15.75kHz) while VGA is
 * active AND the IC has consistently measured a ~15kHz horizontal input. If the measured
 * frequency leaves the 15kHz window, a suppressed 0x80 is replayed so a legitimate 33kHz
 * mode change is delayed, never lost.
 * 
 * It also outputs a signal when 24kHz horizontal frequency is detected, for the purpose of
 * activating my still-in-development 24kHz medium resolution mod. 
 */

 //Version 1.1

// Value that tells functions whether VGA inputs are active or not.
volatile bool VGAactive = false;

// READ_DELAY controls background frequency detection while VGA is active.
#define READ_DELAY 50
#define CHECK_DELAY 250 
#define BUS_IDLE_GUARD_US 3000
#define RX_FRAME_MAX 10
#define RX_QUEUE_SIZE 9

// TA1300AN H FREQ DET (read byte 1, bits 6:0) is Y = 5 * Z * fH, with Z ~= 476us.
// 15.75kHz -> Y ~= 37, 31.5kHz -> Y ~= 75, 33.75kHz -> Y ~= 80. 0 = no sync, 0x7F = >53kHz.
#define H_DET_MASK 0x7F
#define H_OUT_BIT 0x80        // read byte 1, D7: H-out self check (1 = H signal exists)
#define H15K_Y_MIN 30         // ~12.6kHz
#define H15K_Y_MAX 45         // ~18.9kHz
#define H15K_STABLE_COUNT 4   // consecutive in-window polls (x READ_DELAY) before suppression is armed
#define H15K_STALE_MS 1500    // no valid poll for this long -> stop suppressing (fail open)
// 24kHz detection: Y ~= 57-60 measured on a real 24kHz source (15kHz reads 38, 31.5kHz reads 75).
#define H24K_Y_MIN 50         // ~21kHz
#define H24K_Y_MAX 66         // ~27.7kHz
#define H24K_STABLE_COUNT 3   // consecutive in-window polls before the output is asserted
#define H24K_RELEASE_COUNT 4  // consecutive non-24kHz polls (Y != 0) before the output is released
#define H24K_STALE_MS 2000    // no 24kHz reading for this long -> release the output (fail safe)
#define PENDING_REPLAY_MS 1000 // how long a suppressed write may be replayed if frequency leaves 15kHz

// SoftI2CMaster specifies this timeout in milliseconds.
#define I2C_TIMEOUT 10
#define I2C_PULLUP 0
#define I2C_FASTMODE 1

// These work for Nano 3.0
#define SDA_PORT PORTD
#define SDA_PIN 4 // = PD4 - don't forget to add 2.7k pullup resistor to VCC
#define SCL_PORT PORTD
#define SCL_PIN 5 // = PD5 - - don't forget to add 2.7k pullup resistor to VCC
#define OUT24K_PIN 7 // = PD7 - driven high while the chassis is attempting 24kHz and VGA is active
#define VGA_PIN 6 // = PD6 - this is the pin that when high activates the mod - use a 10k pulldown to ground

#include <SoftI2CMaster.h>
#include <Wire.h>
#include <util/atomic.h>
#define TA1300AN_ADDR (0x24)
 
#define PROBLEM_REG (0x00) // the problematic register 
#define PROBLEM_VAL (0x80) // the problematic value

struct RxFrame {
  uint8_t len;
  uint8_t bytes[RX_FRAME_MAX];
};

volatile RxFrame rxQueue[RX_QUEUE_SIZE];
volatile uint8_t rxHead = 0;
volatile uint8_t rxTail = 0;
volatile uint8_t rxCount = 0;

volatile uint32_t lastMcuActivityUs = 0;

// Main-loop-only state derived from the TA1300AN's H FREQ DET reading.
uint8_t h15kCount = 0;           // consecutive polls inside the 15kHz window
uint32_t lastValidPollMs = 0;    // millis() of the last successful downstream poll
uint8_t h24kCount = 0;           // consecutive polls inside the 24kHz window
uint8_t h24kMiss = 0;            // consecutive polls clearly outside the 24kHz window
uint32_t lastH24kMs = 0;         // millis() of the last in-window 24kHz poll
bool out24k = false;             // current state of the 24kHz output pin
volatile bool suppressPending = false; // a 0x80 write was suppressed and may need replaying
uint32_t suppressPendingMs = 0;  // millis() when it was first suppressed

void setup() {
  //Commented out serial.begin as unnecessary. Re-enable for debug if necessary. 

  //Serial.begin(115200); 
  //Serial.print("RCA-MM36110-15kHz-Bridge\n");

  //I2C Startup
  bool iicinit = i2c_init();
  Wire.begin(TA1300AN_ADDR);
  Wire.onReceive(writeRequest);
  Wire.onRequest(readRequest);
  // if(!iicinit) Serial.println("I2C init failed"); commented out since serial is disabled

  //configure pins 
  pinMode(VGA_PIN, INPUT); //configure switch pin as input - 10k pulldown resistor to ground
  pinMode(OUT24K_PIN, OUTPUT);
  digitalWrite(OUT24K_PIN, LOW);
  VGAactive = (PIND & (1 << VGA_PIN)); // direct addressing, faster
}

void writeRegister(const uint8_t reg, const uint8_t val) {
  i2c_start((TA1300AN_ADDR<<1)|I2C_WRITE);
  i2c_write(reg);
  i2c_write(val);
  i2c_stop(); 
}

void writeRequest(int byteCount) {
  lastMcuActivityUs = micros();

  if (byteCount <= 0) {
    return;
  }

  uint8_t len = (byteCount > RX_FRAME_MAX) ? RX_FRAME_MAX : (uint8_t)byteCount;

  if (rxCount < RX_QUEUE_SIZE) {
    rxQueue[rxHead].len = len;
    for (uint8_t i = 0; i < len && Wire.available(); ++i) {
      rxQueue[rxHead].bytes[i] = Wire.read();
    }
    while (Wire.available()) {
      Wire.read();
    }
    rxHead = (rxHead + 1) % RX_QUEUE_SIZE;
    ++rxCount;
  } else {
    // Queue full: drain bytes quickly so the Wire buffer stays in sync.
    while (Wire.available()) {
      Wire.read();
    }
  }
}

void readRequest() {
  lastMcuActivityUs = micros();
  // Forward writes received before this read, including the register pointer.
  RxFrame frame;
  while (popFrame(frame)) {
    processFrame(frame);
  }

  uint8_t buf[2] = {0xFF, 0xFF};
  if (i2c_start((TA1300AN_ADDR << 1) | I2C_READ)) {
    buf[0] = i2c_read(false);
    buf[1] = i2c_read(true);
  }
  i2c_stop();
  Wire.write(buf, 2);
  lastMcuActivityUs = micros();
}

bool isMcuBusIdle() {
  uint32_t now = micros();
  return (uint32_t)(now - lastMcuActivityUs) >= BUS_IDLE_GUARD_US;
}

bool popFrame(RxFrame &out) {
  if (rxCount == 0) return false;
  out.len = rxQueue[rxTail].len;
  for (uint8_t i = 0; i < out.len; ++i) {
    out.bytes[i] = rxQueue[rxTail].bytes[i];
  }
  rxTail = (rxTail + 1) % RX_QUEUE_SIZE;
  --rxCount;
  return true;
}

bool hDetIs24k(const uint8_t b1) {
  const uint8_t y = b1 & H_DET_MASK;
  return (b1 & H_OUT_BIT) && y >= H24K_Y_MIN && y <= H24K_Y_MAX;
}

// Asserts OUT24K_PIN after a stable 24kHz reading and releases it after a stable non-24kHz
// reading. Y == 0 (sync lost during a mode change) holds the current state; a stale or
// failed poll, or VGA going inactive, releases it.
void update24kOutput(const uint32_t now) {
  bool next = out24k;
  if (!VGAactive || (uint32_t)(now - lastH24kMs) >= H24K_STALE_MS) {
    next = false;
  } else if (h24kCount >= H24K_STABLE_COUNT) {
    next = true;
  } else if (h24kMiss >= H24K_RELEASE_COUNT) {
    next = false;
  }
  if (next != out24k) {
    out24k = next;
    digitalWrite(OUT24K_PIN, out24k ? HIGH : LOW);
  }
}

bool hDetIs15k(const uint8_t b1) {
  const uint8_t y = b1 & H_DET_MASK;
  return (b1 & H_OUT_BIT) && y >= H15K_Y_MIN && y <= H15K_Y_MAX;
}

// True only when the IC has consistently and recently measured a 15kHz horizontal input.
bool input15kStable(const uint32_t now) {
  return h15kCount >= H15K_STABLE_COUNT && (uint32_t)(now - lastValidPollMs) < H15K_STALE_MS;
}

void processFrame(const RxFrame &frame) {
  if (frame.len == 0) {
    return;
  }

  // len==1 is treated as register-pointer write for a possible following read.
  i2c_start((TA1300AN_ADDR << 1) | I2C_WRITE);
  i2c_write(frame.bytes[0]);

  if (frame.len > 1) {
    uint8_t reg = frame.bytes[0];
    bool suppress = VGAactive && input15kStable(millis());

    for (uint8_t i = 1; i < frame.len; ++i) {
      uint8_t val = frame.bytes[i];
      if (reg == PROBLEM_REG) {
        if (val == PROBLEM_VAL && suppress) {
          val = 0x00;
          if (!suppressPending) {
            suppressPending = true;
            suppressPendingMs = millis();
          }
        } else {
          // Monitor wrote something else (e.g. its own 0x00 correction): nothing to replay.
          suppressPending = false;
        }
      }
      i2c_write(val);
      ++reg;
    }
  }

  i2c_stop();
}

void readDownstream() {
 
  if (!i2c_start((TA1300AN_ADDR << 1) | I2C_READ)) {
    i2c_stop();
    h15kCount = 0; // no ACK: fail open
    h24kCount = 0;
    update24kOutput(millis());
    return;
  }
  i2c_read(false);
  uint8_t tmp1 = i2c_read(true);
  i2c_stop();

  lastValidPollMs = millis();
  if (hDetIs15k(tmp1)) {
    if (h15kCount < 255) ++h15kCount;
  } else {
    h15kCount = 0;
  }

  const uint32_t now = millis();
  if (hDetIs24k(tmp1)) {
    if (h24kCount < 255) ++h24kCount;
    h24kMiss = 0;
    lastH24kMs = now;
  } else if ((tmp1 & H_DET_MASK) != 0) {
    h24kCount = 0;
    if (h24kMiss < 255) ++h24kMiss;
  } else {
    h24kCount = 0;
  }
  update24kOutput(now);
}

// If we suppressed a 0x80 write but the IC now reports a non-15kHz input, the write was
// legitimate: replay it. Give up quietly once the monitor has had time to self-correct.
void servicePendingReplay() {
  if (!suppressPending) {
    return;
  }
  uint32_t now = millis();
  if ((uint32_t)(now - suppressPendingMs) >= PENDING_REPLAY_MS) {
    suppressPending = false;
  } else if (h15kCount == 0) {
    suppressPending = false;
    writeRegister(PROBLEM_REG, PROBLEM_VAL);
  }
}

void loop() {

  static unsigned long lastCheck = 0;
  static unsigned long lastCheckRead = 0;
  unsigned long now = millis();

 // Non-blocking: check mod status every ~250ms.
  if (now - lastCheck >= CHECK_DELAY) {
    lastCheck = now;
    bool nextVGA = (PIND & (1 << VGA_PIN));
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      if (nextVGA != VGAactive) {
        h15kCount = 0;
        h24kCount = 0;
        h24kMiss = 0;
      }
      VGAactive = nextVGA;
    }
    update24kOutput(now);
  }

  {
    bool processed = false;
    // The Wire callbacks also use the downstream bus. Keep each transaction
    // and its queue removal atomic so the callback cannot interleave with it.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      RxFrame frame;
      if (isMcuBusIdle() && popFrame(frame)) {
        processFrame(frame);
        processed = true;
      }
    }
    if (processed) return;
  }

  if (now - lastCheckRead >= READ_DELAY) {
    lastCheckRead = now;
    if (VGAactive || suppressPending) {
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (isMcuBusIdle() && rxCount == 0) {
          readDownstream();
          servicePendingReplay();
        }
      }
    }
  }
}
  
