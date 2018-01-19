#include <SPI.h>
#include <cc1101.h>
#include <ccpacket.h>

CC1101 cc1101;

const int dataReadyPin = 2;
const int chipSelectPin = 10;
const int triggerPin = 16;

// Global variable to trigger GDO0 activity
volatile bool trigger = false;

// Packet count
uint32_t count;

// SETUP HERE
void setup()
{
  Serial.begin(9600);
  pinMode(triggerPin, OUTPUT);
  SPI.begin();
  pinMode(dataReadyPin, INPUT);
  pinMode(chipSelectPin, OUTPUT);
  // Set the CS and GDO0 Pins
  cc1101.cc11xx_cfg.CS_pin = chipSelectPin;
  cc1101.cc11xx_cfg.GDO0_pin = dataReadyPin;  

  cc1101.init();
  cc1101_config();  
  delay(1000);
  register_dump();
  attachInterrupt(digitalPinToInterrupt(2), isr, FALLING);
  SPI.usingInterrupt(2);
}

void ReadLQI()
{
  byte lqi = 0;
  byte value = 0;
  lqi = (cc1101.readReg(CC1101_LQI, CC1101_STATUS_REGISTER));
  value = 0x3F - (lqi & 0x3F);
  Serial.print("CC1101_LQI ");
  Serial.println(value);
}

void ReadRSSI()
{
  byte rssi = 0;
  byte value = 0;

  rssi = (cc1101.readReg(CC1101_RSSI, CC1101_STATUS_REGISTER));

  if (rssi >= 128)
  {
    value = 255 - rssi;
    value /= 2;
    value += 74;
  }
  else
  {
    value = rssi / 2;
    value += 74;
  }
  Serial.print("CC1101_RSSI ");
  Serial.println(value);
}

void isr()
{
  Serial.println("Interrupt triggered.");
  trigger = true;
}

void loop()
{
  if (trigger) {
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));  //SPI Begin
    // Disable wireless reception interrupt
    detachInterrupt(digitalPinToInterrupt(2));
    CCPACKET packet;

    // clear the flag
    trigger = false;

    if (cc1101.receiveData(&packet) > 0) {
      ReadRSSI();
      ReadLQI();
      Serial.print("packet: len ");
      Serial.println(packet.length);
      Serial.print(" data: ");
      for (int i = 1; i < packet.length; i++) {
        Serial.write(packet.data[i]);
        if (i == 50) {
        Serial.print(" ");
        }
      }
      Serial.println("");
      Serial.print("Packet count: ");
      Serial.println(count++);
    }

    // Enable wireless reception interrupt
    attachInterrupt(digitalPinToInterrupt(2), isr, FALLING);
    SPI.endTransaction();          // SPI End 
  }
}

void cc1101_config() {
 SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));  //SPI Begin
  uint8_t syncH = 0xEE; // 11101110 twice gives you a sync word of 1110111011101110
  uint8_t syncL = 0xEE;  
  cc1101.setSyncWord(syncH, syncL);
  cc1101.setCarrierFreq(CFREQ_315);
  //cc1101.writeReg(0x00, 0x0B); // IOCFG2 - GDO2 Output Pin Configuration
  cc1101.writeReg(0x02, 0x06); // IOCFG0 - GDO0 Output Pin Configuration
  //cc1101.writeReg(0x03, 0x02); // FIFOTHR - RX FIFO and TX FIFO Thresholds
  cc1101.writeReg(0x06, 0xFF); // PKTLEN - Packet Length
  cc1101.writeReg(0x07, 0x04); // PKTCTRL1 - Disabled is 0x04, enabled with broadcast (0x00) is 0x06.
  cc1101.writeReg(0x08, 0x05); // PKTCTRL0 - Packet Automation Control - CRC and variable packet length
  cc1101.writeReg(0x09, 0xDB); // ADDR - Effectively the Hostname
  cc1101.writeReg(0x0A, 0x00); // CHANNR - Channel Number - 0x00 is default
  cc1101.writeReg(0x0B, 0x06); // FSCTRL1 - Frequency Synthesizer Control
  cc1101.writeReg(0x0C, 0x00); // FSCTRL0 - Frequency Synthesizer Control - 0x00 is default
  cc1101.writeReg(0x0D, 0x0C); // FREQ2 - Frequency Control Word, High Byte
  cc1101.writeReg(0x0E, 0x1D); // FREQ1 - Frequency Control Word, Middle Byte
  cc1101.writeReg(0x0F, 0x89); // FREQ0 - Frequency Control Word, Low Byte
  cc1101.writeReg(0x10, 0xC8); // MDMCFG4 - channel bandwidth and exponent for calculating data rate - from 0xC5
  cc1101.writeReg(0x11, 0x83); // MDMCFG3 - Data Rate - DRATE = 1000000.0 * MHZ * (256+drate_m) * powf(2,drate_e) / powf(2,28);
  cc1101.writeReg(0x12, 0x33); // MDMCFG2 - Modulation type (OOK/ASK) / manchester / sync mode - dc block, GFSK, manchester, 16/16
  cc1101.writeReg(0x13, 0x20); // MDMCFG1 - FEC / preamble - 00000010 - No FEC, 2 bytes of preamble, reserved, two bit exponent of channel spacing - 03 **
  cc1101.writeReg(0x14, 0x00); // MDMCFG0 - Channel spacing
  cc1101.writeReg(0x15, 0x40); // Deviation
  cc1101.writeReg(0x19, 0x16); // FOCCFG - From 0x15 in the library
  cc1101.writeReg(0x21, 0x56); // FREND1 - Select PATABLE index to use when sending a '1'
  cc1101.writeReg(0x22, 0x11); // FREND0 - Select PATABLE index to use when sending a '1'

  cc1101.setRxState();
  SPI.endTransaction();          // SPI End
}

void register_dump(){
    Serial.println("Radio initialising\n");
    Serial.print("CC1101_PARTNUM");
    Serial.println(cc1101.readReg(CC1101_PARTNUM, CC1101_STATUS_REGISTER & 0xF0));
    Serial.print("CC1101_VERSION - 0x");
    Serial.println(cc1101.readReg(CC1101_VERSION, CC1101_STATUS_REGISTER), HEX);
    Serial.print("CC1101_MARCSTATE - 0x");
    Serial.println(cc1101.readReg(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) & 0x1F, HEX);
    Serial.print("IOCFG2: GDO2 Output Pin Configuration - 0x");
    Serial.println(cc1101.readReg(0x00, CC1101_CONFIG_REGISTER), HEX);
    Serial.print("IOCFG1: GDO1 Output Pin Configuration - 0x");
    Serial.println(cc1101.readReg(0x01, CC1101_CONFIG_REGISTER), HEX);
    Serial.print("IOCFG0: GDO0 Output Pin Configuration - 0x");
    Serial.println(cc1101.readReg(0x02, CC1101_CONFIG_REGISTER), HEX);
    Serial.print("FIFOTHR - RX FIFO and TX FIFO Thresholds - 0x");
    Serial.println(cc1101.readReg(0x03, CC1101_CONFIG_REGISTER), HEX);
    Serial.print("SYNC1: Sync Word High Byte - 0x");
    Serial.println(cc1101.readReg(0x04, CC1101_CONFIG_REGISTER), HEX);
    Serial.print("SYNC0: Sync Word Low Byte - 0x");
    Serial.println(cc1101.readReg(0x05, CC1101_CONFIG_REGISTER), HEX);
    Serial.print("PKTLEN: Fixed = Packet Length; Variable = Maximum allowed. — 0x");
    Serial.println(cc1101.readReg(0x06, CC1101_CONFIG_REGISTER), HEX);
    Serial.print("PKTCTRL1: PQT / RSSI, LQI / Address Check / CRC OK - 0x");
    Serial.println(cc1101.readReg(0x07, CC1101_CONFIG_REGISTER), HEX);
    Serial.print("PKTCTRL0: Data whitening / Packet format / CRC Check / Packet length - 0x");
    Serial.println(cc1101.readReg(0x08, CC1101_CONFIG_REGISTER), HEX);
    Serial.print("ADDR - Device Address: 0x");
    Serial.println(cc1101.readReg(0x09, CC1101_CONFIG_REGISTER), HEX);
    Serial.print("FSCTRL1: 0x");
    Serial.println(cc1101.readReg(0x0B, CC1101_CONFIG_REGISTER), HEX);
    Serial.print("FSCTRL0: 0x");
    Serial.println(cc1101.readReg(0x0C, CC1101_CONFIG_REGISTER), HEX);
    Serial.print("FREQ2: 0x");
    Serial.println(cc1101.readReg(0x0D, CC1101_CONFIG_REGISTER), HEX);
    Serial.print("FREQ1: 0x");
    Serial.println(cc1101.readReg(0x0E, CC1101_CONFIG_REGISTER), HEX);
    Serial.print("FREQ0: 0x");
    Serial.println(cc1101.readReg(0x0F, CC1101_CONFIG_REGISTER), HEX);
    Serial.print("MDMCFG4: Channel BW - 0x");
    Serial.println(cc1101.readReg(0x10, CC1101_CONFIG_REGISTER), HEX);
    Serial.print("MDMCFG3: Data Rate (Baud) - 0x");
    Serial.println(cc1101.readReg(0x11, CC1101_CONFIG_REGISTER), HEX);
    Serial.print("MDMCFG2: Modulation / Manchester / Sync Mode - 0x");
    Serial.println(cc1101.readReg(0x12, CC1101_CONFIG_REGISTER), HEX);
    Serial.print("MDMCFG1: FEC / Num Preamble / Channel Spacing - 0x");
    Serial.println(cc1101.readReg(0x13, CC1101_CONFIG_REGISTER), HEX);
    Serial.print("MDMCFG0: Mantissa of channel spacing - 0x");
    Serial.println(cc1101.readReg(0x14, CC1101_CONFIG_REGISTER), HEX);
    Serial.print("FREND1: Front end RX configuration - 0x");
    Serial.println(cc1101.readReg(0x21, CC1101_CONFIG_REGISTER), HEX);
    Serial.print("FREND0: Front end TX configuration - 0x");
    Serial.println(cc1101.readReg(0x22, CC1101_CONFIG_REGISTER), HEX);
    Serial.println("device initialized"); 
}
