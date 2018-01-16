#include <WiFi.h>
#include <SPI.h>
#include <cc1101.h>
#include <ccpacket.h>

CC1101 cc1101;

const int dataReadyPin = 21;
const int chipSelectPin = 5;
const int triggerPin = 16;

const char* ssid = "SSID";
const char* password = "SSIDpassword";

// Global variable to trigger GDO0 activity
volatile bool trigger = false;

// Packet count
uint32_t rxcount;
uint32_t txcount;

void cc1101_config(void);
void cc1101_registerDump(void);
void send_data(void);

void setup() {
  Serial.begin(9600);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  pinMode(triggerPin, OUTPUT);
  //digitalWrite(triggerPin, 0); //Using this to Trigger my Logic Analyzer, set the pin Low
  //delay(250);
  //digitalWrite(triggerPin, 1); //Using this to Trigger my Logic Analyzer, set the pin High (Rising Edge)
  SPI.begin();
  pinMode(dataReadyPin, INPUT);
  pinMode(chipSelectPin, OUTPUT);

  cc1101.parameter.SPI_cs = chipSelectPin;
  cc1101.parameter.GDO0_pin = dataReadyPin;
  
  cc1101.init();
  cc1101_config();
  cc1101_registerDump();
  // give the sensor time to set up:
  delay(500);

}

void loop() {
  send_data();
  delay(3000);
}

void cc1101_config() {
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));  //SPI Begin
  digitalWrite(chipSelectPin, 0); // Set the CS Pin LOW 
  uint8_t syncH = 0xEE; // 11101110 twice gives you a sync word of 1110111011101110
  uint8_t syncL = 0xEE;  
  cc1101.setSyncWord(syncH, syncL);
  cc1101.setCarrierFreq(CFREQ_433);
  //cc1101.writeReg(0x00, 0x0B); // IOCFG2 - GDO2 Output Pin Configuration
  cc1101.writeReg(0x02, 0x06); // IOCFG0 - GDO0 Output Pin Configuration
  //cc1101.writeReg(0x03, 0x02); // FIFOTHR - RX FIFO and TX FIFO Thresholds
  cc1101.writeReg(0x06, 0x3D); // PKTLEN - Packet Length
  cc1101.writeReg(0x07, 0x01); // PKTCTRL1 - Disabled is 0x04, enabled with broadcast (0x00) is 0x06.
  cc1101.writeReg(0x08, 0x05); // PKTCTRL0 - Packet Automation Control - CRC and variable packet length
  cc1101.writeReg(0x09, 0xDB); // ADDR - Effectively the Hostname
  cc1101.writeReg(0x0A, 0x00); // CHANNR - Channel Number - 0x00 is default
  cc1101.writeReg(0x0B, 0x06); // FSCTRL1 - Frequency Synthesizer Control
  cc1101.writeReg(0x0C, 0x00); // FSCTRL0 - Frequency Synthesizer Control - 0x00 is default
  cc1101.writeReg(0x0D, 0x10); // FREQ2 - Frequency Control Word, High Byte
  cc1101.writeReg(0x0E, 0xA7); // FREQ1 - Frequency Control Word, Middle Byte
  cc1101.writeReg(0x0F, 0x62); // FREQ0 - Frequency Control Word, Low Byte
  cc1101.writeReg(0x10, 0xC6); // MDMCFG4 - channel bandwidth and exponent for calculating data rate - from 0xC5
  cc1101.writeReg(0x11, 0xE7); // MDMCFG3 - Data Rate - DRATE = 1000000.0 * MHZ * (256+drate_m) * powf(2,drate_e) / powf(2,28);
  cc1101.writeReg(0x12, 0x1A); // MDMCFG2 - Modulation type (OOK/ASK) / manchester / sync mode - dc block, GFSK, manchester, 16/16
  cc1101.writeReg(0x13, 0x02); // MDMCFG1 - FEC / preamble - 00000010 - No FEC, 2 bytes of preamble, reserved, two bit exponent of channel spacing - 03 **
  cc1101.writeReg(0x14, 0x11); // MDMCFG0 - Channel spacing
  cc1101.writeReg(0x15, 0x36); // Deviation
  cc1101.writeReg(0x19, 0x17); // FOCCFG - From 0x15 in the library
  cc1101.writeReg(0x21, 0xB6); // FREND1 - Select PATABLE index to use when sending a '1'
  cc1101.writeReg(0x22, 0x11); // FREND0 - Select PATABLE index to use when sending a '1'
  set_patable();
  digitalWrite(chipSelectPin, 1); // Set the CS Pin HIGH 
  SPI.endTransaction();          // SPI End
}

void cc1101_registerDump() {
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));  //SPI Begin
  digitalWrite(chipSelectPin, 0); // Set the CS Pin LOW
  Serial.println("Radio initialising\n");
  Serial.print("CC1101_PARTNUM: ");
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
  Serial.print("Deviation - 0x");
  Serial.println(cc1101.readReg(0x15, CC1101_CONFIG_REGISTER), HEX);
  Serial.print("FOCCFG - 0x");
  Serial.println(cc1101.readReg(0x19, CC1101_CONFIG_REGISTER), HEX);
  Serial.print("FREND1: Front end RX configuration - 0x");
  Serial.println(cc1101.readReg(0x21, CC1101_CONFIG_REGISTER), HEX);
  Serial.print("FREND0: Front end TX configuration - 0x");
  Serial.println(cc1101.readReg(0x22, CC1101_CONFIG_REGISTER), HEX);
  Serial.println("device initialized"); 
  digitalWrite(chipSelectPin, 1); // Set the CS Pin HIGH   
  SPI.endTransaction();          // SPI End 
}

void send_data() {
  CCPACKET data;
  //Hack the Planet!!!!
  byte thing[] = {0x20, 0x48, 0x61, 0x63, 0x6b, 0x20, 0x74, 0x68, 0x65, 0x20, 0x50, 0x6c, 0x61, 0x6e, 0x65, 0x74, 0x21, 0x21, 0x21, 0x21, 0x21};
  // Hi Punk
  //byte thing[] = {0x20, 0x48, 0x69, 0x20, 0x50, 0x75, 0x6e, 0x6b};

  memcpy(data.data, thing, sizeof(data.data));

  data.length = sizeof(thing);
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));  //SPI Begin
  if (cc1101.sendData(data)) {
    for (int i = 1; i < data.length; i++) {
      Serial.write(data.data[i]);
      Serial.print("");
    }
    Serial.println("");
    Serial.print("TX count: ");
    Serial.println(txcount++);
    Serial.println("");
  } else {
    Serial.println("sent failed :(");
  }    
  SPI.endTransaction();          // SPI End 
}


void set_patable()
{
  byte PA_TABLE[] = {0x00, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  cc1101.writeBurstReg(0x3E, PA_TABLE, 8);
}

