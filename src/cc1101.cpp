/**
 * Copyright (c) 2011 panStamp <contact@panstamp.com>
 * Copyright (c) 2016 Tyler Sommer <contact@tylersommer.pro>
 * Copyright (c) 2017 Danny Kimbler <contact@sorscode.com>
 *
 * This file is part of the CC1101 project.
 *
 * CC1101 is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * any later version.
 *
 * CC1101 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with CC1101; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 
 * USA
 * 
 * Author: Daniel Berenguer
 * receiveData fixes: Samy Kamkar
 * Creation date: 03/03/2011
 */

#include "cc1101.h"

/**
 * Macros
 */
// Select (SPI) CC1101
#define cc1101_Select()  digitalWrite(cc11xx_cfg.CS_pin, LOW)
// Deselect (SPI) CC1101
#define cc1101_Deselect()  digitalWrite(cc11xx_cfg.CS_pin, HIGH)
// Wait until SPI MISO line goes low
#define wait_Miso()  while(digitalRead(MISO)>0)
// Get GDO0 pin state
#define getGDO0state()  digitalRead(cc11xx_cfg.GDO0_pin)
// Wait until GDO0 line goes high
#define wait_GDO0_high()  while(!getGDO0state())
// Wait until GDO0 line goes low
#define wait_GDO0_low()  while(getGDO0state())
// Read CC1101 Config register
#define readConfigReg(regAddr)    readReg(regAddr, CC1101_CONFIG_REGISTER)
// Read CC1101 Status register
#define readStatusReg(regAddr)    readReg(regAddr, CC1101_STATUS_REGISTER)
// Get Marcstate
#define getMarcstate() (readStatusReg(CC1101_MARCSTATE) & 0x1F)

/**
 * CC1101
 * 
 * Class constructor
 */
CC1101::CC1101(void)
{
  paTableByte = PA_LowPower;            // Priority = Low power
}

/**
 * wakeUp
 * 
 * Wake up CC1101 from Power Down state
 */
void CC1101::wakeUp(void)
{
  cc1101_Select();                      // Select CC1101
  wait_Miso();                          // Wait until MISO goes low
  cc1101_Deselect();                    // Deselect CC1101
}

/**
 * writeReg
 * 
 * Write single register into the CC1101 IC via SPI
 * 
 * 'regAddr'	Register address
 * 'value'	Value to be writen
 */
void CC1101::writeReg(byte regAddr, byte value) 
{
  cc1101_Select();                      // Select CC1101
  wait_Miso();                          // Wait until MISO goes low
  SPI.transfer(regAddr);                    // Send register address
  SPI.transfer(value);                      // Send value
  cc1101_Deselect();                    // Deselect CC1101
}

/**
 * writeBurstReg
 * 
 * Write multiple registers into the CC1101 IC via SPI
 * 
 * 'regAddr'	Register address
 * 'buffer'	Data to be writen
 * 'len'	Data length
 */
void CC1101::writeBurstReg(byte regAddr, byte* buffer, byte len)
{
  byte addr, i;
  
  addr = regAddr | WRITE_BURST;         // Enable burst transfer
  cc1101_Select();                      // Select CC1101
  wait_Miso();                          // Wait until MISO goes low
  SPI.transfer(addr);                       // Send register address
  
  for(i=0 ; i<len ; i++)
    SPI.transfer(buffer[i]);                // Send value

  cc1101_Deselect();                    // Deselect CC1101  
}

/**
 * cmdStrobe
 * 
 * Send command strobe to the CC1101 IC via SPI
 * 
 * 'cmd'	Command strobe
 */     
void CC1101::cmdStrobe(byte cmd) 
{
  cc1101_Select();                      // Select CC1101
  wait_Miso();                          // Wait until MISO goes low
  SPI.transfer(cmd);                        // Send strobe command
  cc1101_Deselect();                    // Deselect CC1101
}

/**
 * readReg
 * 
 * Read CC1101 register via SPI
 * 
 * 'regAddr'	Register address
 * 'regType'	Type of register: CC1101_CONFIG_REGISTER or CC1101_STATUS_REGISTER
 * 
 * Return:
 * 	Data byte returned by the CC1101 IC
 */
byte CC1101::readReg(byte regAddr, byte regType) 
{
  byte addr, val;

  addr = regAddr | regType;
  cc1101_Select();                      // Select CC1101
  wait_Miso();                          // Wait until MISO goes low
  SPI.transfer(addr);                       // Send register address
  val = SPI.transfer(0x00);                 // Read result
  cc1101_Deselect();                    // Deselect CC1101

  return val;
}

/**
 * readBurstReg
 * 
 * Read burst data from CC1101 via SPI
 * 
 * 'buffer'	Buffer where to copy the result to
 * 'regAddr'	Register address
 * 'len'	Data length
 */
void CC1101::readBurstReg(byte * buffer, byte regAddr, byte len) 
{
  byte addr, i;
  
  addr = regAddr | READ_BURST;
  cc1101_Select();                      // Select CC1101
  wait_Miso();                          // Wait until MISO goes low
  SPI.transfer(addr);                       // Send register address
  for(i=0 ; i<len ; i++)
    buffer[i] = SPI.transfer(0x00);         // Read result byte by byte
  cc1101_Deselect();                    // Deselect CC1101
}

/**
 * reset
 * 
 * Reset CC1101
 */
void CC1101::reset(void) 
{
  cc1101_Deselect();                    // Deselect CC1101
  delayMicroseconds(5);
  cc1101_Select();                      // Select CC1101
  delayMicroseconds(10);
  cc1101_Deselect();                    // Deselect CC1101
  delayMicroseconds(41);
  cc1101_Select();                      // Select CC1101

  wait_Miso();                          // Wait until MISO goes low
  SPI.transfer(CC1101_SRES);                // Send reset command strobe
  wait_Miso();                          // Wait until MISO goes low

  cc1101_Deselect();                    // Deselect CC1101

  setDefaultRegs();                     // Reconfigure CC1101
}

/**
 * setDefaultRegs
 * 
 * Configure CC1101 registers
 */
void CC1101::setDefaultRegs(void) 
{
  writeReg(CC1101_IOCFG2,  CC1101_DEFVAL_IOCFG2);
  writeReg(CC1101_IOCFG1,  CC1101_DEFVAL_IOCFG1);
  writeReg(CC1101_IOCFG0,  CC1101_DEFVAL_IOCFG0);
  writeReg(CC1101_FIFOTHR,  CC1101_DEFVAL_FIFOTHR);
  writeReg(CC1101_PKTLEN,  CC1101_DEFVAL_PKTLEN);
  writeReg(CC1101_PKTCTRL1,  CC1101_DEFVAL_PKTCTRL1);
  writeReg(CC1101_PKTCTRL0,  CC1101_DEFVAL_PKTCTRL0);

  // Set default synchronization word
  setSyncWord(CC1101_DEFVAL_SYNC1, CC1101_DEFVAL_SYNC0);

  // Set default device address
  setDevAddress(CC1101_DEFVAL_ADDR);
  // Set default frequency channel
  setChannel(CC1101_DEFVAL_CHANNR);
  

  writeReg(CC1101_CHANNR,  CC1101_DEFVAL_CHANNR);
  writeReg(CC1101_ADDR,  CC1101_DEFVAL_ADDR);



  writeReg(CC1101_FSCTRL1,  CC1101_DEFVAL_FSCTRL1);
  writeReg(CC1101_FSCTRL0,  CC1101_DEFVAL_FSCTRL0);

  // Set default carrier frequency = 433 MHz
  setCarrierFreq(CFREQ_433);

  writeReg(CC1101_MDMCFG4,  CC1101_DEFVAL_MDMCFG4);
  writeReg(CC1101_MDMCFG3,  CC1101_DEFVAL_MDMCFG3);
  writeReg(CC1101_MDMCFG2,  CC1101_DEFVAL_MDMCFG2);
  writeReg(CC1101_MDMCFG1,  CC1101_DEFVAL_MDMCFG1);
  writeReg(CC1101_MDMCFG0,  CC1101_DEFVAL_MDMCFG0);
  writeReg(CC1101_DEVIATN,  CC1101_DEFVAL_DEVIATN);

  writeReg(CC1101_PKTLEN,  CC1101_DEFVAL_PKTLEN);
  writeReg(CC1101_PKTCTRL1,  CC1101_DEFVAL_PKTCTRL1);
  writeReg(CC1101_PKTCTRL0,  CC1101_DEFVAL_PKTCTRL0);

  writeReg(CC1101_MCSM2,  CC1101_DEFVAL_MCSM2);
  writeReg(CC1101_MCSM1,  CC1101_DEFVAL_MCSM1);
  writeReg(CC1101_MCSM0,  CC1101_DEFVAL_MCSM0);
  writeReg(CC1101_FOCCFG,  CC1101_DEFVAL_FOCCFG);
  writeReg(CC1101_BSCFG,  CC1101_DEFVAL_BSCFG);
  writeReg(CC1101_AGCCTRL2,  CC1101_DEFVAL_AGCCTRL2);
  writeReg(CC1101_AGCCTRL1,  CC1101_DEFVAL_AGCCTRL1);
  writeReg(CC1101_AGCCTRL0,  CC1101_DEFVAL_AGCCTRL0);
  writeReg(CC1101_WOREVT1,  CC1101_DEFVAL_WOREVT1);
  writeReg(CC1101_WOREVT0,  CC1101_DEFVAL_WOREVT0);
  writeReg(CC1101_WORCTRL,  CC1101_DEFVAL_WORCTRL);
  writeReg(CC1101_FREND1,  CC1101_DEFVAL_FREND1);
  writeReg(CC1101_FREND0,  CC1101_DEFVAL_FREND0);
  writeReg(CC1101_FSCAL3,  CC1101_DEFVAL_FSCAL3);
  writeReg(CC1101_FSCAL2,  CC1101_DEFVAL_FSCAL2);
  writeReg(CC1101_FSCAL1,  CC1101_DEFVAL_FSCAL1);
  writeReg(CC1101_FSCAL0,  CC1101_DEFVAL_FSCAL0);
  writeReg(CC1101_RCCTRL1,  CC1101_DEFVAL_RCCTRL1);
  writeReg(CC1101_RCCTRL0,  CC1101_DEFVAL_RCCTRL0);
  writeReg(CC1101_FSTEST,  CC1101_DEFVAL_FSTEST);
  writeReg(CC1101_PTEST,  CC1101_DEFVAL_PTEST);
  writeReg(CC1101_AGCTEST,  CC1101_DEFVAL_AGCTEST);
  writeReg(CC1101_TEST2,  CC1101_DEFVAL_TEST2);
  writeReg(CC1101_TEST1,  CC1101_DEFVAL_TEST1);
  writeReg(CC1101_TEST0,  CC1101_DEFVAL_TEST0);
}

/**
 * init
 * 
 * Initialize CC1101
 */
void CC1101::init(void) 
{
	reset();
 
  writeReg(CC1101_PATABLE, paTableByte);
}

/**
 * setSyncWord
 * 
 * Set synchronization word
 * 
 * 'syncH'	Synchronization word - High byte
 * 'syncL'	Synchronization word - Low byte
 */
void CC1101::setSyncWord(uint8_t syncH, uint8_t syncL) 
{
  if ((syncWord[0] != syncH) || (syncWord[1] != syncL))
  {
    writeReg(CC1101_SYNC1, syncH);
    writeReg(CC1101_SYNC0, syncL);
    syncWord[0] = syncH;
    syncWord[1] = syncL;
  }
}

/**
 * setSyncWord (overriding method)
 * 
 * Set synchronization word
 * 
 * 'syncH'	Synchronization word - pointer to 2-byte array
 */
void CC1101::setSyncWord(byte *sync) 
{
  CC1101::setSyncWord(sync[0], sync[1]);
}

/**
 * setDevAddress
 * 
 * Set device address
 * 
 * 'addr'	Device address
 */
void CC1101::setDevAddress(byte addr) 
{
  if (devAddress != addr)
  {
    writeReg(CC1101_ADDR, addr);
    devAddress = addr;
  }
}

/**
 * setChannel
 * 
 * Set frequency channel
 * 
 * 'chnl'	Frequency channel
 */
void CC1101::setChannel(byte chnl) 
{
  if (channel != chnl)
  {
    writeReg(CC1101_CHANNR,  chnl);
    channel = chnl;
  }
}

/**
 * setCarrierFreq
 * 
 * Set carrier frequency
 * 
 * 'freq'	New carrier frequency
 */
void CC1101::setCarrierFreq(byte freq)
{
  switch(freq)
  {
    case CFREQ_915:
      writeReg(CC1101_FREQ2,  CC1101_DEFVAL_FREQ2_915);
      writeReg(CC1101_FREQ1,  CC1101_DEFVAL_FREQ1_915);
      writeReg(CC1101_FREQ0,  CC1101_DEFVAL_FREQ0_915);
      break;
    case CFREQ_433:
      writeReg(CC1101_FREQ2,  CC1101_DEFVAL_FREQ2_433);
      writeReg(CC1101_FREQ1,  CC1101_DEFVAL_FREQ1_433);
      writeReg(CC1101_FREQ0,  CC1101_DEFVAL_FREQ0_433);
      break;
    case CFREQ_315:
      writeReg(CC1101_FREQ2,  CC1101_DEFVAL_FREQ2_315);
      writeReg(CC1101_FREQ1,  CC1101_DEFVAL_FREQ1_315);
	  writeReg(CC1101_FREQ0,  CC1101_DEFVAL_FREQ0_315);
      break;  
    default:
      writeReg(CC1101_FREQ2,  CC1101_DEFVAL_FREQ2_868);
      writeReg(CC1101_FREQ1,  CC1101_DEFVAL_FREQ1_868);
      writeReg(CC1101_FREQ0,  CC1101_DEFVAL_FREQ0_868);
      break;
  }
   
  carrierFreq = freq;  
}

/**
 * setPowerDownState
 * 
 * Put CC1101 into power-down state
 */
void CC1101::setPowerDownState() 
{
  // Comming from RX state, we need to enter the IDLE state first
  cmdStrobe(CC1101_SIDLE);
  // Enter Power-down state
  cmdStrobe(CC1101_SPWD);
}

/**
 * sendData
 * 
 * Send data packet via RF
 * 
 * 'packet'	Packet to be transmitted. First byte is the destination address
 *
 *  Return:
 *    True if the transmission succeeds
 *    False otherwise
 */
boolean CC1101::sendData(CCPACKET packet)
{
  Serial.println("Test Point 1");
  byte marcState;
  bool res = false;
 
  // Declare to be in Tx state. This will avoid receiving packets whilst
  // transmitting
  Serial.println("Test Point 2");
  rfState = RFSTATE_TX;
  Serial.println("Test Point 3");
  // Enter RX state
  setRxState();
  Serial.println("Test Point 4");
  // Check that the RX state has been entered
  while (((marcState = readStatusReg(CC1101_MARCSTATE)) & 0x1F) != 0x0D)
  {
    if (marcState == 0x11)        // RX_OVERFLOW
      flushRxFifo();              // flush receive queue
  }
  Serial.println("Test Point 5");
  delayMicroseconds(500);
  Serial.println("Test Point 6");
  // Set data length at the first position of the TX FIFO
  // writeReg(CC1101_TXFIFO,  packet.length);
  // Write data into the TX FIFO
  writeBurstReg(CC1101_TXFIFO, packet.data, packet.length);
  Serial.println("Test Point 7");
  // CCA enabled: will enter TX state only if the channel is clear
  setTxState();
  Serial.println("Test Point 8");
  // Check that TX state is being entered (state = RXTX_SETTLING)
  marcState = readStatusReg(CC1101_MARCSTATE) & 0x1F;
  if((marcState != 0x13) && (marcState != 0x14) && (marcState != 0x15))
  {
    setIdleState();       // Enter IDLE state
    flushTxFifo();        // Flush Tx FIFO
    setRxState();         // Back to RX state

    // Declare to be in Rx state
    rfState = RFSTATE_RX;
    return false;
  }
  Serial.println("Test Point 9");
  // Wait for the sync word to be transmitted
  wait_GDO0_high();
  Serial.println("Test Point 10");
  // Wait until the end of the packet transmission
  wait_GDO0_low();
  Serial.println("Test Point 11");
  // Check that the TX FIFO is empty
  if((readStatusReg(CC1101_TXBYTES) & 0x7F) == 0)
    res = true;
  Serial.println("Test Point 12");
  setIdleState();       // Enter IDLE state
  flushTxFifo();        // Flush Tx FIFO
  Serial.println("Test Point 13");
  // Enter back into RX state
  setRxState();
  Serial.println("Test Point 14");
  // Declare to be in Rx state
  rfState = RFSTATE_RX;
  Serial.println("Test Point 15");
  return res;
}

/**
 * receiveData
 * 
 * Read data packet from RX FIFO
 *
 * 'packet'	Container for the packet received
 * 
 * Return:
 * 	Amount of bytes received
 */
byte CC1101::receiveData(CCPACKET * packet)
{
  byte val;
  byte rxbytes = readStatusReg(CC1101_RXBYTES);

  // Rx FIFO overflow?
  if (getMarcstate() == 0x11)
  {
    setIdleState();       // Enter IDLE state
    flushRxFifo();        // Flush Rx FIFO
    packet->length = 0;
  }
  // Any byte waiting to be read?
  else if (rxbytes & 0x7F)
  {
    // Read data length
    packet->length = readStatusReg(CC1101_RXBYTES) & 0x7F;

    // If packet is too long
    if (packet->length > CC1101_DATA_LEN)
      packet->length = 0;   // Discard packet
    else
    {
      // Read data packet
      readBurstReg(packet->data, CC1101_RXFIFO, packet->length);
      // Read RSSI
      packet->rssi = readConfigReg(CC1101_RXFIFO);
      // Read LQI and CRC_OK
      val = readConfigReg(CC1101_RXFIFO);
      packet->lqi = val & 0x7F;
      packet->crc_ok = bitRead(val, 7);
    }

    // empty fifo here
    flushRxFifo();
  }
  else
    packet->length = 0;

  // Back to RX state
  setRxState();

  return packet->length;
}
