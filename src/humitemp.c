/***************************************************************************//**
 * @file
 * @brief Relative humidity and temperature sensor demo for SLSTK3402A
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "cpt112s_config.h"
#include "i2cspm.h"
#include "si7013.h"
#include "glib.h"
#include "rtcdriver.h"
#include "graphics.h"
#include "em_adc.h"
#include "bspconfig.h"
#include "udelay.h"
/***************************************************************************//**
 * Local defines
 ******************************************************************************/

/** Time (in ms) between periodic updates of the measurements. */
#define PERIODIC_UPDATE_MS      2000
#define IR_SLAVE_ADDRESS       0x00
#define IR_MEASURE_COMMAND      0xAF
#define IR_READ_SENSOR_OR  0x01  // logical "or" into slave address to specify read or write
#define IR_WRITE_SENSOR_OR 0x00

/*Calculation Defines*/
#define TC 		-.0046

/*
Global Variables
*/
/*Calculation variables calculated in readEEPROM()*/
float Tref;
float k4comp;
float k3comp;
float k2comp;
float k1comp;
float k0comp;

float k4obj;
float k3obj;
float k2obj;
float k1obj;
float k0obj;

I2CSPM_Init_TypeDef i2cInit;


union 
{
  uint16_t iValue[2];
  float fValue;
}Myunion;

/***************************************************************************//**
 * Local variables
 ******************************************************************************/
/* RTC callback parameters. */
static void (*rtcCallback)(void*) = 0;
static void * rtcCallbackArg = 0;

/** This flag tracks if we need to update the display
 *  (animations or measurements). */
static volatile bool updateDisplay = true;
/** This flag tracks if we need to perform a new
 *  measurement. */
static volatile bool updateMeasurement = true;

/** Timer used for periodic update of the measurements. */
RTCDRV_TimerID_t periodicUpdateTimerId;

/***************************************************************************//**
 * Local prototypes
 ******************************************************************************/
static void gpioSetup(void);
static void periodicUpdateCallback(RTCDRV_TimerID_t id, void *user);
static void memLcdCallback(RTCDRV_TimerID_t id, void *user);

/***************************************************************************//**
 * @brief  Helper function to perform data measurements.
 ******************************************************************************/
//static int performMeasurements(I2C_TypeDef *i2c, uint32_t *rhData, int32_t *tData)
//{
//  Si7013_MeasureRHAndTemp(i2c, SI7021_ADDR, rhData, tData);
//  return 0;
//}
static int32_t i2c_write_eeprom(I2C_TypeDef *i2c, uint8_t addr, int32_t *data,
                              uint8_t command)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[2];
  uint8_t                    i2c_write_data[1];

  seq.addr  = addr;
  seq.flags = I2C_FLAG_WRITE_READ;
  /* Select command to issue */
  i2c_write_data[0] = command;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 1;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 1;

  ret = I2CSPM_Transfer(i2c, &seq);

  if (ret != i2cTransferDone) {
    *data = 0;
    return((int) ret);
  }

  // *data = ((uint32_t) i2c_read_data[0] << 8) + (i2c_read_data[1] & 0xfc);

  return((int32_t) 0);
}
static int32_t i2c_read_eeprom(I2C_TypeDef *i2c, uint8_t addr, int32_t *data,
                              uint8_t command)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[3];
  uint8_t                    i2c_write_data[1];

  seq.addr  = addr;
  seq.flags = I2C_FLAG_WRITE_READ;
  /* Select command to issue */
  i2c_write_data[0] = command;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 0;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 3;

  ret = I2CSPM_Transfer(i2c, &seq);

  if (ret != i2cTransferDone) {
    *data = 0;
    return((int) ret);
  }

  *data = (int32_t) (((int32_t)i2c_read_data[0] << 16) + ((int32_t)i2c_read_data[1] << 8) + ((int32_t)i2c_read_data[2]));

  return((int32_t) 2);
}
static int32_t i2c_write_sensor(I2C_TypeDef *i2c, uint8_t addr, int64_t *data,
                              uint8_t command)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[2];
  uint8_t                    i2c_write_data[1];

  seq.addr  = addr;
  seq.flags = I2C_FLAG_WRITE_READ;
  /* Select command to issue */
  i2c_write_data[0] = command;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 1;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 1;

  ret = I2CSPM_Transfer(i2c, &seq);

  if (ret != i2cTransferDone) {
    *data = 0;
    return((int) ret);
  }

  // *data = ((uint32_t) i2c_read_data[0] << 8) + (i2c_read_data[1] & 0xfc);

  return((int32_t) 0);
}

static int32_t i2c_read_sensor(I2C_TypeDef *i2c, uint8_t addr, int64_t *data,
                              uint8_t command)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[7];
  uint8_t                    i2c_write_data[1];

  seq.addr  = addr;
  seq.flags = I2C_FLAG_WRITE_READ;
  /* Select command to issue */
  i2c_write_data[0] = command;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 0;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 7;

  ret = I2CSPM_Transfer(i2c, &seq);

  if (ret != i2cTransferDone) {
    *data = 0;
    return((int) ret);
  }

  *data = (int64_t) (((int64_t)i2c_read_data[0] << 48) + ((int64_t)i2c_read_data[1] << 40) + ((int64_t)i2c_read_data[2] << 32) + ((int64_t)i2c_read_data[3] << 24) +
                    ((int64_t)i2c_read_data[4] << 16) + ((int64_t)i2c_read_data[5] << 8) + ((int64_t)i2c_read_data[6]));

  return((int32_t) 2);
}

float getEEPROMData(uint8_t address)
{
  int32_t			highEepromData;
  int32_t			lowEepromData;
  i2c_write_eeprom(i2cInit.port, IR_SLAVE_ADDRESS | IR_WRITE_SENSOR_OR, &highEepromData, address);
  UDELAY_Delay(100000);
  i2c_read_eeprom(i2cInit.port, IR_SLAVE_ADDRESS | IR_READ_SENSOR_OR, &highEepromData, address);
  UDELAY_Delay(100000);
  i2c_write_eeprom(i2cInit.port, IR_SLAVE_ADDRESS | IR_WRITE_SENSOR_OR, &lowEepromData, address + 1);
  UDELAY_Delay(100000);
  i2c_read_eeprom(i2cInit.port, IR_SLAVE_ADDRESS | IR_READ_SENSOR_OR, &lowEepromData, address + 1); 
  Myunion.iValue[0] = lowEepromData;
  Myunion.iValue[1] = highEepromData;
  return Myunion.fValue; 
}

void readEEPROM(void)
{
  for(int i = 0x20; i <= 0x36; i += 2)
  {
    float readVal = getEEPROMData(i);
    switch(i)
    {
      case 0x20:
        Tref = readVal;
        break;
      case 0x22:
        k4comp = readVal;
        break;
      case 0x24:
        k3comp = readVal;
        break;
      case 0x26:
        k2comp = readVal;
        break;
      case 0x28:
        k1comp = readVal;
        break;
      case 0x2A:
        k0comp = readVal;
        break;
      case 0x2C:
        break;
      case 0x2E:
        k4obj = readVal;
        break;
      case 0x30:
        k3obj = readVal;
        break;
      case 0x32:
        k2obj = readVal;
        break;
      case 0x34:
        k1obj = readVal;
        break;
      case 0x36:
        k0obj = readVal;
        break;
    }
  }
   
  
  
}

/***************************************************************************//**
 * @brief Setup GPIO, enable sensor isolation switch
 ******************************************************************************/
static void gpioSetup(void)
{
  /* Enable GPIO clock */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Enable Si7021 sensor isolation switch */
  GPIO_PinModeSet(CS0_SENSOR_EN_PORT, CS0_SENSOR_EN_PIN, gpioModePushPull, 1);
}

/***************************************************************************//**
 * @brief   The actual callback for Memory LCD toggling
 * @param[in] id
 *   The id of the RTC timer (not used)
 ******************************************************************************/
static void memLcdCallback(RTCDRV_TimerID_t id, void *user)
{
  (void) id;
  (void) user;
  rtcCallback(rtcCallbackArg);
}

/***************************************************************************//**
 * @brief   Register a callback function at the given frequency.
 *
 * @param[in] pFunction  Pointer to function that should be called at the
 *                       given frequency.
 * @param[in] argument   Argument to be given to the function.
 * @param[in] frequency  Frequency at which to call function at.
 *
 * @return  0 for successful or
 *         -1 if the requested frequency does not match the RTCC frequency.
 ******************************************************************************/
int rtccIntCallbackRegister(void (*pFunction)(void*),
                            void* argument,
                            unsigned int frequency)
{
  RTCDRV_TimerID_t timerId;
  rtcCallback    = pFunction;
  rtcCallbackArg = argument;

  RTCDRV_AllocateTimer(&timerId);

  RTCDRV_StartTimer(timerId, rtcdrvTimerTypePeriodic, 1000 / frequency,
                    memLcdCallback, 0);

  return 0;
}

/***************************************************************************//**
 * @brief Callback used to count between measurement updates
 ******************************************************************************/
static void periodicUpdateCallback(RTCDRV_TimerID_t id, void *user)
{
  (void) id;
  (void) user;
  updateDisplay = true;
  updateMeasurement = true;
}


/***************************************************************************//**
 * @brief  Main function
 ******************************************************************************/
int main(void)
{
  // I2CSPM_Init_TypeDef i2cInit = I2CSPM_INIT_DEFAULT;
  i2cInit.port = I2C0;
  i2cInit.sclPort = gpioPortC;
  i2cInit.sclPin = 11;
  i2cInit.sdaPort = gpioPortC;
  i2cInit.sdaPin = 10;
  i2cInit.portLocationScl = 15;
  i2cInit.portLocationSda = 15;
  i2cInit.i2cRefFreq = 0;
  i2cInit.i2cMaxFreq = I2C_FREQ_STANDARD_MAX;
  i2cInit.i2cClhr = i2cClockHLRStandard;

  EMU_DCDCInit_TypeDef dcdcInit = EMU_DCDCINIT_STK_DEFAULT;
  CMU_HFXOInit_TypeDef hfxoInit = CMU_HFXOINIT_DEFAULT;
//  bool             si7013_status;
  int64_t          tempData = 0;

  
// IR Temp local variables
  uint64_t      ADCsen = 0;
  uint64_t      ADCobj = 0;
  float      ADCcomp = 0;
  float      ADCcompTC = 0;
  float      Tsen = 0;
  float      Fsen = 0;
  uint8_t statusByte = 0;
  float temp = 0;
  float TCF;
  float offset;
  float offsetTC;
  float Tobj;
  float Fobj;


  /* Chip errata */
  CHIP_Init();

  /* Initalize hardware */
  EMU_DCDCInit(&dcdcInit);
  CMU_HFXOInit(&hfxoInit);

  /* Switch HFCLK to HFXO and disable HFRCO */
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
  CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);

  /* Initalize other peripherals and drivers */
  gpioSetup();
  RTCDRV_Init();
  GRAPHICS_Init();
  I2CSPM_Init(&i2cInit);

  /* Get initial sensor status */
  // si7013_status = Si7013_Detect(i2cInit.port, SI7021_ADDR, 0);
  readEEPROM();
  /* Set up periodic update of the display. */
  RTCDRV_AllocateTimer(&periodicUpdateTimerId);
  RTCDRV_StartTimer(periodicUpdateTimerId, rtcdrvTimerTypePeriodic,
                    PERIODIC_UPDATE_MS, periodicUpdateCallback, 0);

//  GRAPHICS_ShowStatus(si7013_status);
  EMU_EnterEM2(false);

  updateDisplay = true;

  while (true)
  {
    // if (updateMeasurement) 
    // {
    //   // performMeasurements(i2cInit.port, &rhData, &tempData);
    //   updateMeasurement = false;
    // }

    // if (updateDisplay) 
    // {
    //   updateDisplay = false;
    //   GRAPHICS_Draw(tempData, rhData);
    // }
    // EMU_EnterEM2(false);


    i2c_write_sensor(i2cInit.port, IR_SLAVE_ADDRESS | IR_WRITE_SENSOR_OR, &tempData, IR_MEASURE_COMMAND);
    UDELAY_Delay(100000);
    i2c_read_sensor(i2cInit.port, IR_SLAVE_ADDRESS | IR_READ_SENSOR_OR, &tempData, IR_MEASURE_COMMAND);
    statusByte = (uint8_t)(tempData >> 48);
    if(statusByte != 0x60)  // check if we are busy or memory error
    {
      ADCsen = (tempData & 0xFFFFFF);
      // temp = (float)ADCsen / (2*2*2*2*2*2*2*2*2*2*2*2*2*2*2*2*2*2*2*2*2*2*2*2);
      temp = (float)ADCsen / (1 << 24);
      Tsen = (temp * 105) - 20;
      Fsen = (1.8 * Tsen) + 32;

      TCF = 1 + ((Tsen - Tref) * TC);
      offset = (k4comp * (Tsen * Tsen * Tsen * Tsen)) +
                (k3comp * (Tsen * Tsen * Tsen)) + 
                (k2comp * (Tsen * Tsen)) + 
                (k1comp * Tsen) + 
                (k0comp); 
      offsetTC = offset * TCF;
      ADCobj = tempData >> 24;
      ADCobj &= 0xFFFFFF;
      // ADCcomp = offsetTC - (2*2*2*2*2*2*2*2*2*2*2*2*2*2*2*2*2*2*2*2*2*2*2) + (float)ADCobj;
      ADCcomp = offsetTC - (1 << 23) + (float)ADCobj;
      ADCcompTC = ADCcomp / TCF;
      Tobj = (k4obj * ADCcompTC * ADCcompTC * ADCcompTC * ADCcompTC) +
              (k3obj * ADCcompTC * ADCcompTC * ADCcompTC) + 
              (k2obj * ADCcompTC * ADCcompTC) + 
              (k1obj * ADCcompTC) + 
              (k0obj);
      Fobj = 1.8 * Tobj + 32;
    }
    EMU_EnterEM2(false);

  }
}
