/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_I2C) && !defined(SOFT_I2C)

#include "build/debug.h"

#include "common/time.h"

#include "drivers/system.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/rcc.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"

#define IOCFG_I2C_PU IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_OD, GPIO_PuPd_UP)
#define IOCFG_I2C    IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_OD, GPIO_PuPd_NOPULL)

#define I2C_HIGHSPEED_TIMING  0x00500E30  // 1000 Khz, 72Mhz Clock, Analog Filter Delay ON, Setup 40, Hold 4.
#define I2C_STANDARD_TIMING   0x00E0257A  // 400 Khz, 72Mhz Clock, Analog Filter Delay ON, Rise 100, Fall 10.

#define I2C_GPIO_AF         GPIO_AF_4

static volatile uint16_t i2cErrorCount = 0;

const i2cHardware_t i2cHardware[I2CDEV_COUNT] = {
#ifdef USE_I2C_DEVICE_1
    {
        .device = I2CDEV_1,
        .reg = I2C1,
        .sclPins = { I2CPINDEF(PA15), I2CPINDEF(PB6), I2CPINDEF(PB8) },
        .sdaPins = { I2CPINDEF(PA14), I2CPINDEF(PB7), I2CPINDEF(PB9) },
        .rcc = RCC_APB1(I2C1),
    },
#endif
#ifdef USE_I2C_DEVICE_2
    {
        .device = I2CDEV_2,
        .reg = I2C2,
        .sclPins = { I2CPINDEF(PA9), I2CPINDEF(PF6) },
        .sdaPins = { I2CPINDEF(PA10) },
        .rcc = RCC_APB1(I2C2),
    },
#endif
};

i2cDevice_t i2cDevice[I2CDEV_COUNT];

///////////////////////////////////////////////////////////////////////////////
// I2C TimeoutUserCallback
///////////////////////////////////////////////////////////////////////////////

#define I2C_TIMEOUT_TRANSFER    7000   // us
#define I2C_TIMEOUT_WAIT        100    // us

typedef struct {
  timeUs_t start, period;
} timeoutState_t;

#define I2C_TIMEOUT_CONS(p) { micros(), p }

static bool timeoutOccurred(timeoutState_t *state)
{
  if(state->period > 0 && micros() > state->start+state->period) {
    if(state->period > I2C_TIMEOUT_WAIT)
      // It's a transfer level timeout
      i2cErrorCount++;
    
    return true;
  }
  return false;
}

bool i2cWaitOnFlagTO(I2C_TypeDef *I2Cx, uint32_t flag, FlagStatus status, timeUs_t timeout_)
{
  timeoutState_t timeout = I2C_TIMEOUT_CONS(timeout_);
  
  while (I2C_GetFlagStatus(I2Cx, flag) == status)
    if(timeoutOccurred(&timeout))
      return false;

  return true;
}

bool i2cWaitOnFlag(I2C_TypeDef *I2Cx, uint32_t flag, FlagStatus status)
{
  return i2cWaitOnFlagTO(I2Cx, flag, status, I2C_TIMEOUT_TRANSFER);
}

bool i2cTimeoutUserCallback(void)
{
  i2cErrorCount++;
  return false;
}

void i2cInit(I2CDevice device)
{
    if (device == I2CINVALID || device > I2CDEV_COUNT) {
        return;
    }

    i2cDevice_t *pDev = &i2cDevice[device];
    const i2cHardware_t *hw = pDev->hardware;

    if (!hw) {
        return;
    }

    I2C_TypeDef *I2Cx = pDev->reg;

    IO_t scl = pDev->scl;
    IO_t sda = pDev->sda;

    RCC_ClockCmd(hw->rcc, ENABLE);
    RCC_I2CCLKConfig(I2Cx == I2C2 ? RCC_I2C2CLK_SYSCLK : RCC_I2C1CLK_SYSCLK);

    IOInit(scl, OWNER_I2C_SCL, RESOURCE_INDEX(device));
    IOConfigGPIOAF(scl, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, GPIO_AF_4);

    IOInit(sda, OWNER_I2C_SDA, RESOURCE_INDEX(device));
    IOConfigGPIOAF(sda, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, GPIO_AF_4);

    I2C_InitTypeDef i2cInit = {
        .I2C_Mode = I2C_Mode_I2C,
        .I2C_AnalogFilter = I2C_AnalogFilter_Enable,
        .I2C_DigitalFilter = 0x00,
        .I2C_OwnAddress1 = 0x00,
        .I2C_Ack = I2C_Ack_Enable,
        .I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit,
        .I2C_Timing = (pDev->overClock ? I2C_HIGHSPEED_TIMING : I2C_STANDARD_TIMING)
    };

    I2C_Init(I2Cx, &i2cInit);

    I2C_StretchClockCmd(I2Cx, ENABLE);

    I2C_Cmd(I2Cx, ENABLE);
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

bool i2cWriteGeneric(I2CDevice device, uint8_t target_, uint8_t addrSize, uint8_t *addr, uint8_t len, uint8_t *data)
{
    if (device == I2CINVALID || device > I2CDEV_COUNT) {
        return false;
    }

    I2C_TypeDef *I2Cx = i2cDevice[device].reg;

    if (!I2Cx) {
        return false;
    }

    target_ <<= 1;

    /* Test on BUSY Flag */

    if(!i2cWaitOnFlag(I2Cx, I2C_ISR_BUSY, SET))
      return false;

    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(I2Cx, target_, addrSize+len, I2C_AutoEnd_Mode, I2C_Generate_Start_Write);

    while(addrSize > 0 || len > 0) {
      /* Wait until TXIS flag is set */

      if(!i2cWaitOnFlag(I2Cx, I2C_ISR_TXIS, RESET))
	return false;

      if(addrSize > 0) {
	/* Send Register address */
	I2C_SendData(I2Cx, (uint8_t) *addr++);
	addrSize--;
      } else {
	I2C_SendData(I2Cx, (uint8_t) *data++);
	len--;
      }
    }

    // Wait until Stop
    
    if(!i2cWaitOnFlag(I2Cx, I2C_ISR_STOPF, RESET))
      return false;

    /* Clear STOPF flag */
    I2C_ClearFlag(I2Cx, I2C_ICR_STOPCF);

    return true;
}

bool i2cWriteBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
  return i2cWriteGeneric(device, addr_, 1, &reg_, len_, data);
}

bool i2cWrite(I2CDevice device, uint8_t addr_, uint8_t reg, uint8_t data)
{
  return i2cWriteBuffer(device, addr_, reg, 1, &data);
}

uint8_t i2cWait(I2CDevice device, uint8_t target_)
{
    timeoutState_t major = I2C_TIMEOUT_CONS(I2C_TIMEOUT_TRANSFER);
    bool ack = false;
    
    if (device == I2CINVALID || device > I2CDEV_COUNT) {
        return 1;
    }

    I2C_TypeDef *I2Cx = i2cDevice[device].reg;

    if (!I2Cx) {
        return 2;
    }

    target_ <<= 1;

    /* Test on BUSY Flag */
    
    if(!i2cWaitOnFlag(I2Cx, I2C_ISR_BUSY, SET))
      return 3;

    do {
      I2C_TransferHandling(I2Cx, target_, 1, I2C_AutoEnd_Mode, I2C_Generate_Start_Write);

      if((ack = i2cWaitOnFlagTO(I2Cx, I2C_ISR_TXIS, RESET, I2C_TIMEOUT_WAIT)))
	I2C_SendData(I2Cx, 0);
    } while(!ack && !timeoutOccurred(&major));

    /* Wait until STOPF flag is set */

    if(!i2cWaitOnFlag(I2Cx, I2C_ISR_STOPF, RESET))
      return 4;

    /* Clear STOPF flag */
    I2C_ClearFlag(I2Cx, I2C_ICR_STOPCF);
    
    /* If all operations OK */
    return 0;
}

bool i2cReadGeneric(I2CDevice device, uint8_t target_, uint8_t addrSize, uint8_t *addr, uint8_t len, uint8_t* buf)
{
    if (device == I2CINVALID || device > I2CDEV_COUNT) {
        return false;
    }

    I2C_TypeDef *I2Cx = i2cDevice[device].reg;

    if (!I2Cx) {
        return false;
    }

    target_ <<= 1;

    /* Test on BUSY Flag */
    
    if(!i2cWaitOnFlag(I2Cx, I2C_ISR_BUSY, SET))
      return false;

    if(addrSize > 0) {
      I2C_TransferHandling(I2Cx, target_, addrSize, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

      while(addrSize > 0) {
	/* Wait until TXIS flag is set */
	
	if(!i2cWaitOnFlag(I2Cx, I2C_ISR_TXIS, RESET))
	  return false;

	/* Send Register address */
	I2C_SendData(I2Cx, (uint8_t) *addr++);
	addrSize--;
      }

      /* Wait until TC flag is set */

      if(!i2cWaitOnFlag(I2Cx, I2C_ISR_TC, RESET))
	return false;
    }
    
    I2C_TransferHandling(I2Cx, target_, len, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

    /* Wait until all data are received */
    while(len > 0) {
        /* Wait until RXNE flag is set */

      if(!i2cWaitOnFlag(I2Cx, I2C_ISR_RXNE, RESET))
	return false;

      /* Read data from RXDR */
      *buf++ = I2C_ReceiveData(I2Cx);
      len--;
    }

    /* Wait until STOPF flag is set */
    
    if(!i2cWaitOnFlag(I2Cx, I2C_ISR_STOPF, RESET))
      return false;

    /* Clear STOPF flag */
    I2C_ClearFlag(I2Cx, I2C_ICR_STOPCF);

    /* If all operations OK */
    return true;
}

bool i2cRead(I2CDevice device, uint8_t addr_, uint8_t reg, uint8_t len, uint8_t* buf)
{
  return i2cReadGeneric(device, addr_, 1, &reg, len, buf);
}

#endif
