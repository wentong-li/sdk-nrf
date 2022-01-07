/**
 * @file cs47l63_syscfg_regs.c
 *
 * @brief Register values to be applied after CS47L63 Driver boot().
 *
 * @copyright
 * Copyright (c) Cirrus Logic 2020 All Rights Reserved, http://www.cirrus.com/
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
/***********************************************************************************************************************
 * INCLUDES
 **********************************************************************************************************************/
#include "cs47l63_syscfg_regs.h"
#include "cs47l63_spec.h"

/***********************************************************************************************************************
 * GLOBAL VARIABLES
 **********************************************************************************************************************/
const syscfg_reg_t cs47l63_syscfg_regs[] =
{
    {0x00001400, 0xFFFFFFFF, 0x00000041}, 
    {0x00001C04, 0xFFFFFFFF, 0x28601177}, 
    {0x00001C08, 0xFFFFFFFF, 0x00010000}, 
    {0x00001C0C, 0xFFFFFFFF, 0x23F05004}, 
    {0x00001C00, 0xFFFFFFFF, 0x00000006}, 
    {0x00001404, 0xFFFFFFFF, 0x00000404}, 
    {0x00001420, 0xFFFFFFFF, 0x00000003}, 
    {0x00001510, 0xFFFFFFFF, 0x25800004}, 
    {0x00000C08, 0xFFFFFFFF, 0xE1000000}, 
    {0x00000C0C, 0xFFFFFFFF, 0xE1000000}, 
    {0x00000C10, 0xFFFFFFFF, 0xE1000000}, 
    {0x00000C14, 0xFFFFFFFF, 0xE1000000}, 
    {0x00006004, 0xFFFFFFFF, 0x00000033}, 
    {0x00006008, 0xFFFFFFFF, 0x20200200}, 
    {0x00006000, 0xFFFFFFFF, 0x00030003}, 
};

