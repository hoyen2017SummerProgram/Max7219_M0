
#include "drv_Max7219.h"
#include "string.h"



/* The array for shifting the data to the devices */
uint8_t spidata[16];
/* We keep track of the led-status for all 8 devices in this array */
uint8_t status[64];
/* The maximum number of devices we use */
uint8_t maxDevices;
extern SPI_HandleTypeDef hspi1;

void LedControl( uint16_t numDevices)
{
  if(numDevices<=0 || numDevices>8 )
      numDevices=8;
  maxDevices=numDevices;

  CS_HIGH();

  memset(status, 0, 64);

  for(int i=0;i<maxDevices;i++)
  {
    spiTransfer(i,OP_DISPLAYTEST,0);
    //scanlimit is set to max on startup
    setScanLimit(i,7);
    //decode is done in source
    spiTransfer(i,OP_DECODEMODE,0);
    clearDisplay(i);
    //we go into shutdown-mode on startup
    shutdown(i,true);
  }
}

uint8_t getDeviceCount(void)
{
  return maxDevices;
}

void shutdown(uint16_t addr, bool b)
{
  if(addr<0 || addr>=maxDevices)
      return;
  if(b)
      spiTransfer(addr, OP_SHUTDOWN,0);
  else
      spiTransfer(addr, OP_SHUTDOWN,1);
}

void setScanLimit(uint16_t addr, uint16_t limit)
{
  if(addr<0 || addr>=maxDevices)
      return;
  if(limit>=0 && limit<8)
      spiTransfer(addr, OP_SCANLIMIT,limit);
}


void setIntensity(uint16_t addr, uint16_t intensity)
{
  if(addr<0 || addr>=maxDevices)
      return;
  if(intensity>=0 && intensity<16)	
      spiTransfer(addr, OP_INTENSITY,intensity);  
}

void clearDisplay(uint16_t addr)
{
  int offset;

  if(addr<0 || addr>=maxDevices)
      return;
  offset=addr*8;
  for(int i=0;i<8;i++) {
      status[offset+i]=0;
      spiTransfer(addr, i+1,status[offset+i]);
  }
}

void setLed(uint16_t addr, uint16_t row, uint16_t column, bool state)
{
  int offset;
  uint8_t val=0x00;

  if(addr<0 || addr>=maxDevices)
      return;
  if(row<0 || row>7 || column<0 || column>7)
      return;
  offset=addr*8;
  val= 0x80 >> column;
  if(state)
      status[offset+row]=status[offset+row]|val;
  else 
  {
      val=~val;
      status[offset+row]=status[offset+row]&val;
  }
  spiTransfer(addr, row+1,status[offset+row]);
}

void setRow(uint16_t addr, uint16_t row, uint8_t value)
{
  int offset;
  if(addr<0 || addr>=maxDevices)
      return;
  if(row<0 || row>7)
      return;
  offset=addr*8;
  status[offset+row]=value;
  spiTransfer(addr, row+1,status[offset+row]);
}

void setColumn(uint16_t addr, uint16_t col, uint8_t value)
{
  uint8_t val;

  if(addr<0 || addr>=maxDevices)
      return;
  if(col<0 || col>7) 
      return;
  for(int row=0;row<8;row++) {
      val=value >> (7-row);
      val=val & 0x01;
      setLed(addr,row,col,val);
  }  
}

void setDigit(uint16_t addr, uint16_t digit, uint8_t value, bool dp)
{
  int offset;
  uint8_t v;

  if(addr<0 || addr>=maxDevices)
      return;
  if(digit<0 || digit>7 || value>15)
      return;
//  offset=addr*8;
//  v=pgm_read_byte_near(charTable + value); 
//  if(dp)
//      v|=B10000000;
//  status[offset+digit]=v;
//  spiTransfer(addr, digit+1,v);
}

void setChar(uint16_t addr, uint16_t digit, char value, bool dp)
{
    int offset;
    uint8_t index,v;

    if(addr<0 || addr>=maxDevices)
        return;
    if(digit<0 || digit>7)
        return;
    offset=addr*8;
    index=(uint8_t)value;
    if(index >127) {
        //no defined beyond index 127, so we use the space char
        index=32;
    }
//    v=pgm_read_byte_near(charTable + index); 
//    if(dp)
//        v|=B10000000;
//    status[offset+digit]=v;
//    spiTransfer(addr, digit+1,v);
}

void spiTransfer(uint16_t addr, volatile uint8_t opcode, volatile uint8_t data)
{
  int offset=addr*2;
//  int maxbytes=maxDevices*2;

  memset(spidata, 0, 16);
  //put our device data into the array
  spidata[offset]=opcode;
  spidata[offset+1]=data;
  CS_LOW();
  //Now shift out the data 
  HAL_SPI_Transmit(&hspi1, spidata, 2, 100);
  CS_HIGH();
}