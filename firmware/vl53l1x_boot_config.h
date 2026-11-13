/**
 ******************************************************************************
 * @file    vl53l0x_class.cpp
 * @author  IMG
 * @version V0.0.1
 * @date    14-December-2018
 * @brief   Implementation file for the VL53L1X driver class
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
*/
/* Adapted from ST/stm32duino VL53L1X; 2V8 I2C enabled at 0x002E.
 * Keep GPIO_HV_PAD__CTRL (0x002F) at ST's default 1V8 setting:
 * this carrier's XSHUT was measured at 2.00 V.
 * https://github.com/stm32duino/VL53L1X/blob/main/src/vl53l1x_class.cpp
 */
static const uint8_t tof_boot_config[91]={
    0x00,0x01,0x00,0x01,0x02,0x00,0x02,0x08,0x00,0x08,0x10,0x01,
    0x01,0x00,0x00,0x00,0x00,0xff,0x00,0x0f,0x00,0x00,0x00,0x00,
    0x00,0x20,0x0b,0x00,0x00,0x02,0x0a,0x21,0x00,0x00,0x05,0x00,
    0x00,0x00,0x00,0xc8,0x00,0x00,0x38,0xff,0x01,0x00,0x08,0x00,
    0x00,0x01,0xcc,0x0f,0x01,0xf1,0x0d,0x01,0x68,0x00,0x80,0x08,
    0xb8,0x00,0x00,0x00,0x00,0x0f,0x89,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x01,0x0f,0x0d,0x0e,0x0e,0x00,0x00,0x02,0xc7,0xff,
    0x9b,0x00,0x00,0x00,0x01,0x00,0x00
};
