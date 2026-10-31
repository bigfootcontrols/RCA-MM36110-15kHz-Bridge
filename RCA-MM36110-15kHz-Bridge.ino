/* TDA9151B I2C Bridge 
 *
 * Based on Martin Hejnfelt's TA1276AN I2C Bridge
 * and bsafta's fork. 
 * basfta fork: https://github.com/bsafta/TDA9151B-i2c-bridge 
 * original: https://github.com/skumlos/TDA9151B-i2c-bridge 
 * 
 * This software uses an Arduino Nano V3 and sits between a monitor's 
 * main microcontroller and the TDA9151B convergence IC, and acts as 
 * a man-in-the-middle to modify I2C commands to the convergence IC. Its 
 * current implementation is for an RCA MM36110, with the two functions being
 * to shift the image horizontally when an RGB mod is activated, and to 
 * enable 16:9 mode which for some reason RCA didn't implement by default. 
 *
 */

 //Version 1.0

// THE FOLLOWING VALUE SHOULD BE SET BASED ON YOUR PERSONAL PREFERENCES FOR YOUR MONITOR.
// THIS IS SET TO WHAT WORKED FOR ME ON MY RCA MM36110, YOUR MILEAGE MAY VARY
// 0 = no offset, positive integers = positive offset, negative = negative

const int8_t horPhaseOffset = -7; //Tells the jungle how much to offset horizontal position. 

// Values that tell functions whether RGB and 16:9 mods are running or not. Don't mess with this except for testing purposes.
volatile bool RGBModEnabled = false;
volatile bool WidescreenEnabled = false;

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
#define SDA_PIN 4 // = PD4
#define SCL_PORT PORTD
#define SCL_PIN 5 // = PD5
#define RGB_PIN 6 // = PD6 - this is the pin that will be used to activate the mod - use a 10k pulldown to ground
#define WIDESCREEN_PIN 7 //PD7 - this pin when high will enable Widescreen - use a 10k pulldown to ground

#include <SoftI2CMaster.h>
#include <Wire.h>
// Address in the datasheet is said to be 0x8C. That is somewhat of a 
// "mistake" as i2c uses 7 bit addressing and the least significant bit
// is read (1) or write (0). Thus the address is shifted once
// to the right to get the "real" address which is then 46h/70
#define TDA9151B_ADDR (0x46)

// Not all of these are used. I just went ahead and listed them all for convenience
// for potential future versions. 
#define REG_VERT_AMP (0x00) // Vertical Amplitude
#define REG_VERT_S_COR (0x01) // Vertical S Correction
#define REG_VSS (0x02) // Vertical Start Scan
#define REG_VOCS_TRAP (0x03) // Vertical off-centre shift AND trapezium correction
#define REG_WIDTH (0x04) // EW width/width ratio
#define REG_PARABOLA (0x05) // EW parabola/width ratio
#define REG_CORNER (0x06) // EW corner/parabola ratio
#define REG_EHT (0x07) // EHT compensation
#define REG_HOR_PHASE (0x08) // Horizontal Phase
#define REG_HOR_SHIFT (0x09) //Horizontal off-center shift
#define REG_CLAMP_SHIFT (0x0A) //Clamp shift
#define REG_CONTROL_1 (0x0B) //Control 1
#define REG_VERT_SLOPE_MSB (0x0C) // Vertical Slope MSB
#define REG_VERT_SLOPE_LSB (0x0D) // Vert Slope LSB
#define REG_VERT_WAIT (0x0E) // Vertical Wait
#define REG_CONTROL_2 (0x0F) // Control 2

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
  //Commented out serial.begin to save on startup time, I don't run my arduino on standby power, it 
  //turns on when the TV does, and quick startup is a priority. Re-enable for debug if necessary. 

  //Serial.begin(115200); 
  //Serial.print("TDA9151B I2C Bridge\n");

  //I2C Startup
  bool iicinit = i2c_init();
  Wire.begin(TDA9151B_ADDR);
  Wire.onReceive(writeRequest);
  Wire.onRequest(readRequest);
  if(!iicinit) Serial.println("I2C init failed");

  //configure pins 
  pinMode(RGB_PIN, INPUT); //configure switch pin as input - 10k pulldown resistor to ground
  pinMode(WIDESCREEN_PIN, INPUT); //configure VM pin as input - 10k pulldown resistor to ground
  RGBModEnabled = (PIND & (1 << RGB_PIN)); //NOTE these use direct addressing and may not work if you reassign pins that are on different ports
  WidescreenEnabled = (PIND & (1 << WIDESCREEN_PIN));
}

void writeRegister(const uint8_t reg, const uint8_t val) {
  i2c_start((TDA9151B_ADDR<<1)|I2C_WRITE);
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


// Offsets standard contrast value by user-defined offset to arrive at RGB contrast value
uint8_t getHorPhaseRegValue(uint8_t val) {
  uint8_t regValue = add_with_signed_offset(val, horPhaseOffset);
  
  //for debugging, DO NOT uncomment unless you need to, slows down operation
  /*Serial.print("Original H Phase: "); 
  Serial.print(val, BIN);
  Serial.print(";   Mod. H Phase: ");
  Serial.print(regValue, BIN);
  Serial.print("\n\n");*/

  return regValue;
}

// Math logic used by each of the above offset functions, can accept negative values
uint8_t add_with_signed_offset(uint8_t base, int8_t offset) {
  int16_t sum = (int16_t)base + (int16_t)offset;
  if (sum > 255) return 255;
  if (sum < 0) return 0;
  return (uint8_t)sum;
}


volatile uint8_t r = 0;

void readRequest() {
  lastMcuActivityUs = micros();
  Wire.write(r);
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
  i2c_start((TDA9151B_ADDR << 1) | I2C_WRITE);
  i2c_write(frame.bytes[0]);

  if (frame.len > 1) {
    uint8_t reg = frame.bytes[0];
    bool rgbEnabled = RGBModEnabled;
    bool widescreenEnabled = WidescreenEnabled;

    for (uint8_t i = 1; i < frame.len; ++i) {
      uint8_t val = frame.bytes[i];
      if (reg == REG_HOR_PHASE && rgbEnabled) {
        val = getHorPhaseRegValue(val);
      } else if (reg == REG_CONTROL_2 && widescreenEnabled) {
        val = 0b00001000;
      }
      i2c_write(val);
      ++reg;
    }
  }

  i2c_stop();
}

void readDownstream() {
  
  /*if (!i2c_start((TDA9151B_ADDR << 1) | I2C_READ)) {
    i2c_stop();
    return;
  }*/
  i2c_start((TDA9151B_ADDR << 1) | I2C_READ);
  uint8_t tmp0 = i2c_read(true);
  i2c_stop();

  //noInterrupts();
  r = tmp0;
  //interrupts();
}

void loop() {

  static unsigned long lastCheck = 0;
  static unsigned long lastCheckRead = 0;
  unsigned long now = millis();

 // Non-blocking: check mod status every ~250ms.
  if (now - lastCheck >= CHECK_DELAY) {
    lastCheck = now;
    RGBModEnabled = (PIND & (1 << RGB_PIN)); //NOTE these use direct addressing and may not work if you reassign pins that are on different ports
    WidescreenEnabled = (PIND & (1 << WIDESCREEN_PIN));
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
  
