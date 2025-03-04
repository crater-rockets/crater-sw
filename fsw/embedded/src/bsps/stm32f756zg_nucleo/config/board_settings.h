/***************************************************************************
 *   Copyright (C) 2018 by Terraneo Federico                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   As a special exception, if other files instantiate templates or use   *
 *   macros or inline functions from this file, or you compile this file   *
 *   and link it with other works to produce a work based on this file,    *
 *   this file does not by itself cause the resulting work to be covered   *
 *   by the GNU General Public License. However the source code for this   *
 *   file must still be made available in accordance with the GNU General  *
 *   Public License. This exception does not invalidate any other reasons  *
 *   why a work based on this file might be covered by the GNU General     *
 *   Public License.                                                       *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, see <http://www.gnu.org/licenses/>   *
 ***************************************************************************/

 #pragma once

 #include "interfaces/gpio.h"
 
 /**
  * \internal
  * Versioning for board_settings.h for out of git tree projects
  */
 #define BOARD_SETTINGS_VERSION 300
 
 namespace miosix {
 
 /**
  * \addtogroup Settings
  * \{
  */
 
 /// Size of stack for main().
 /// The C standard library is stack-heavy (iprintf requires 1KB)
 const unsigned int MAIN_STACK_SIZE=4*1024;
 
 /// Serial port
 /// Serial ports 1 to 8 are available
 const unsigned int defaultSerial=3;
 const unsigned int defaultSerialSpeed=115200;
 const bool defaultSerialFlowctrl=false;
 const bool defaultSerialDma=true;
 // Default serial 1 pins (uncomment when using serial 1)
 //using defaultSerialTxPin = Gpio<GPIOA_BASE,9>;
 //using defaultSerialRxPin = Gpio<GPIOA_BASE,10>;
 //using defaultSerialRtsPin = Gpio<GPIOA_BASE,12>;
 //using defaultSerialCtsPin = Gpio<GPIOA_BASE,11>;
 // Default serial 2 pins (uncomment when using serial 2)
 //using defaultSerialTxPin = Gpio<GPIOA_BASE,2>;
 //using defaultSerialRxPin = Gpio<GPIOA_BASE,3>;
 //using defaultSerialRtsPin = Gpio<GPIOA_BASE,1>;
 //using defaultSerialCtsPin = Gpio<GPIOA_BASE,0>;
 // Default serial 3 pins (uncomment when using serial 3)
 using defaultSerialTxPin = Gpio<GPIOD_BASE,8>;
 using defaultSerialRxPin = Gpio<GPIOD_BASE,9>;
 using defaultSerialRtsPin = Gpio<GPIOD_BASE,12>;
 using defaultSerialCtsPin = Gpio<GPIOD_BASE,11>;
 
 //SD card driver
 static const unsigned char sdVoltage=33; //Board powered @ 3.3V
 // #define SD_ONE_BIT_DATABUS
 #define SD_SDMMC 1 //Select either SDMMC1 or SDMMC2
 
 /**
  * \}
  */
 
 }  // namespace miosix
 