/* RCA MM36110 15kHz-Bridge 
 *
 * Based on Martin Hejnfelt's TA1276AN I2C Bridge
 * and bsafta's fork. 
 * basfta fork: https://github.com/bsafta/TDA9151B-i2c-bridge 
 * original: https://github.com/skumlos/TDA9151B-i2c-bridge 
 * 
 * This software uses an Arduino Nano V3 and sits between a RCA MM36110's 
 * main microcontroller and the input-switching IC labeled U38300, and acts as 
 * a man-in-the-middle to modify an errant I2C signal that causes
 * visual glitches when 15kHz content is displayed over VGA. This removes the infamous 
 * "garbage pixel artifacts" mentioned on the CRT database. 
 * The code has no other fuction at present, although it could theoretically have 
 * interesting applications for input switching modifications on the MM36110 and
 * potentially MM36100.
 */

 //Version 1.0

// Value that tells functions whether VGA inputs are active or not.
volatile bool VGAactive = false;

// READ_DELAY defines how often the bridge refreshes a cached downstream read value.
// Keep this relatively slow so the bridge spends less time bit-banging while the
// monitor MCU could start a transaction.
#define READ_DELAY 250
#define CHECK_DELAY 250 
#define BUS_IDLE_GUARD_US 7000
#define RX_FRAME_MAX 16
#define RX_QUEUE_SIZE 18

#define I2C_TIMEOUT 10000
#define I2C_PULLUP 0
#define I2C_FASTMODE 1

// These work for Nano 3.0
#define SDA_PORT PORTD
#define SDA_PIN 4 // = PD4 - don't forget to add 2.7k pullup resistor to VCC
#define SCL_PORT PORTD
#define SCL_PIN 5 // = PD5 - - don't forget to add 2.7k pullup resistor to VCC
#define VGA_PIN 6 // = PD6 - this is the pin that when high activates the mod - use a 10k pulldown to ground

#include <SoftI2CMaster.h>
#include <Wire.h>
#define U38300_ADDR (0x24)
 
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

void setup() {
  //Commented out serial.begin as unnecessary. Re-enable for debug if necessary. 

  //Serial.begin(115200); 
  //Serial.print("RCA-MM36110-15kHz-Bridge\n");

  //I2C Startup
  bool iicinit = i2c_init();
  Wire.begin(U38300_ADDR);
  Wire.onReceive(writeRequest);
  Wire.onRequest(readRequest);
  if(!iicinit) Serial.println("I2C init failed");

  //configure pins 
  pinMode(VGA_PIN, INPUT); //configure switch pin as input - 10k pulldown resistor to ground
  VGAactive = (PIND & (1 << VGA_PIN)); // direct addressing, faster
}

void writeRegister(const uint8_t reg, const uint8_t val) {
  i2c_start((U38300_ADDR<<1)|I2C_WRITE);
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

// Set up two-byte array to hold values read from U38300
volatile uint8_t r[2] = {0, 0};

void readRequest() {
  lastMcuActivityUs = micros();
  Wire.write(r,2);
}

bool isMcuBusIdle() {
  uint32_t now = micros();
  return (uint32_t)(now - lastMcuActivityUs) >= BUS_IDLE_GUARD_US;
}

bool popFrame(RxFrame &out) {
  noInterrupts();
  if (rxCount == 0) {
    interrupts();
    return false;
  }

  out.len = rxQueue[rxTail].len;
  for (uint8_t i = 0; i < out.len; ++i) {
    out.bytes[i] = rxQueue[rxTail].bytes[i];
  }
  rxTail = (rxTail + 1) % RX_QUEUE_SIZE;
  --rxCount;
  interrupts();
  return true;
}

void processFrame(const RxFrame &frame) {
  if (frame.len == 0) {
    return;
  }

  // len==1 is treated as register-pointer write for a possible following read.
  i2c_start((U38300_ADDR << 1) | I2C_WRITE);
  i2c_write(frame.bytes[0]);

  if (frame.len > 1) {
    uint8_t reg = frame.bytes[0];
    bool VGA = VGAactive;

    for (uint8_t i = 1; i < frame.len; ++i) {
      uint8_t val = frame.bytes[i];
      if (reg == PROBLEM_REG && val == PROBLEM_VAL && VGA) {
        val = 0x00;
      } 
      i2c_write(val);
      ++reg;
    }
  }

  i2c_stop();
}

void readDownstream() {
 
  i2c_start((U38300_ADDR << 1) | I2C_READ);
  uint8_t tmp0 = i2c_read(false); //read byte and proceed to next
  uint8_t tmp1 = i2c_read(true);  // send stop signal
  i2c_stop();
  r[0] = tmp0; //write to array
  r[1] = tmp1;
}

void loop() {

  static unsigned long lastCheck = 0;
  static unsigned long lastCheckRead = 0;
  unsigned long now = millis();

 // Non-blocking: check mod status every ~250ms.
  if (now - lastCheck >= CHECK_DELAY) {
    lastCheck = now;
    VGAactive = (PIND & (1 << VGA_PIN)); //NOTE this uses direct addressing for speed
  }

  if (isMcuBusIdle()) {
    RxFrame frame;
    if (popFrame(frame)) {
      processFrame(frame);
      // Favor write servicing over background reads.
      return;
    }
  }

  if (now - lastCheckRead >= READ_DELAY) {
    lastCheckRead = now;
    if (isMcuBusIdle() && rxCount == 0) {
      readDownstream();
    }
  }
}
  
