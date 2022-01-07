/**
 * @file system_test_hw_0_bsp.c
 *
 * @brief Implementation of the BSP for the system_test_hw_0 platform.
 *
 * @copyright
 * Copyright (c) Cirrus Logic 2021 All Rights Reserved, http://www.cirrus.com/
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
#include <string.h>
#include "hw_0_bsp.h"
#include "cs47l63.h"
#include "cs47l63_ext.h"
#include "cs47l63_syscfg_regs.h"
#include "cs47l63_fw_img.h"

/***********************************************************************************************************************
 * LOCAL LITERAL SUBSTITUTIONS
 **********************************************************************************************************************/

#define CS47L63_SRC_TONE_GENERATOR1 (0x4)

#define CS47L63_DSP1_CHANNEL1 (0x100)
#define CS47L63_DSP1_CHANNEL2 (0x101)

/***********************************************************************************************************************
 * LOCAL VARIABLES
 **********************************************************************************************************************/
static cs47l63_t cs47l63_driver;
static fw_img_boot_state_t boot_state;

static cs47l63_bsp_config_t bsp_config =
{
    .bsp_dev_id = BSP_DUT_DEV_ID,
    .bsp_reset_gpio_id = BSP_GPIO_ID_DUT_CDC_RESET,
    .bsp_dcvdd_supply_id = BSP_SUPPLY_ID_LN2_DCVDD,
    .bsp_int_gpio_id = BSP_GPIO_ID_DUT_CDC_INT,
    .bus_type = BSP_BUS_TYPE_SPI,
    .notification_cb = &bsp_notification_callback,
    .notification_cb_arg = NULL
};

/***********************************************************************************************************************
 * GLOBAL VARIABLES
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * LOCAL FUNCTIONS
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * API FUNCTIONS
 **********************************************************************************************************************/
uint32_t bsp_dut_initialize(void)
{
    uint32_t ret = BSP_STATUS_OK;

    cs47l63_config_t codec_config;

    memset(&codec_config, 0, sizeof(cs47l63_config_t));

    // Initialize chip drivers
    ret = cs47l63_initialize(&cs47l63_driver);
    if (ret == CS47L63_STATUS_OK)
    {
        codec_config.bsp_config = bsp_config;

        codec_config.syscfg_regs = cs47l63_syscfg_regs;
        codec_config.syscfg_regs_total = CS47L63_SYSCFG_REGS_TOTAL;

        ret = cs47l63_configure(&cs47l63_driver, &codec_config);
    }

    if (ret != CS47L63_STATUS_OK)
    {
        ret = BSP_STATUS_FAIL;
    }

    // Enable 32kHz clock routing to CS47L63
    uint32_t temp_buffer = __builtin_bswap32(0x001F8003);
    bsp_i2c_write(BSP_LN2_DEV_ID, (uint8_t *)&temp_buffer, 4, NULL, NULL);
    // Bypass LN2 FPGA
    temp_buffer = __builtin_bswap32(0x00EE0000);
    bsp_i2c_write(BSP_LN2_DEV_ID, (uint8_t *)&temp_buffer, 4, NULL, NULL);
    // Enable MICVDD at 1v8
    temp_buffer = __builtin_bswap32(0x011B001D);
    bsp_i2c_write(BSP_LN2_DEV_ID, (uint8_t *)&temp_buffer, 4, NULL, NULL);
    temp_buffer = __builtin_bswap32(0x01198000);
    bsp_i2c_write(BSP_LN2_DEV_ID, (uint8_t *)&temp_buffer, 4, NULL, NULL);
    // Route MICBIAS2 to P2
    temp_buffer = __builtin_bswap32(0x00E40010);
    bsp_i2c_write(BSP_LN2_DEV_ID, (uint8_t *)&temp_buffer, 4, NULL, NULL);
    temp_buffer = __builtin_bswap32(0x00E50100);
    bsp_i2c_write(BSP_LN2_DEV_ID, (uint8_t *)&temp_buffer, 4, NULL, NULL);
    temp_buffer = __builtin_bswap32(0x00E38000);
    bsp_i2c_write(BSP_LN2_DEV_ID, (uint8_t *)&temp_buffer, 4, NULL, NULL);

    cs47l63_wait(2000);

    return ret;
}

uint32_t bsp_dut_reset(void)
{
    uint32_t ret;

    ret = cs47l63_reset(&cs47l63_driver);

    if (ret != CS47L63_STATUS_OK)
    {
        return BSP_STATUS_FAIL;
    }

    return BSP_STATUS_OK;
}

uint32_t bsp_dut_boot(void)
{
    uint32_t ret;
    const uint8_t *fw_img;
    const uint8_t *fw_img_end;
    uint32_t write_size;

    fw_img = cs47l63_fw_img;
    fw_img_end = cs47l63_fw_img + FW_IMG_SIZE(cs47l63_fw_img);

    // Inform the driver that any current firmware is no longer available by passing a NULL
    // fw_info pointer to cs47l63_boot
    ret = cs47l63_boot(&cs47l63_driver, 1, NULL);
    if (ret != CS47L63_STATUS_OK)
    {
        return ret;
    }

    // Free anything malloc'ed in previous boots
    if (boot_state.fw_info.sym_table)
        bsp_free(boot_state.fw_info.sym_table);
    if (boot_state.fw_info.alg_id_list)
        bsp_free(boot_state.fw_info.alg_id_list);
    if (boot_state.block_data)
        bsp_free(boot_state.block_data);

    // Ensure your fw_img_boot_state_t struct is initialised to zero.
    memset(&boot_state, 0, sizeof(fw_img_boot_state_t));

    // Emulate a system where only 1k fw_img blocks can be processed at a time
    write_size = 1024;

    // Initialise pointer to the currently available fw_img data
    boot_state.fw_img_blocks = (uint8_t *) fw_img;
    boot_state.fw_img_blocks_size = write_size;

    // Read in the fw_img header
    ret = fw_img_read_header(&boot_state);
    if (ret)
    {
        return BSP_STATUS_FAIL;
    }

    // malloc enough memory to hold the symbol table, using sym_table_size in the previously
    // read in fw_img header
    boot_state.fw_info.sym_table = (fw_img_v1_sym_table_t *)bsp_malloc(boot_state.fw_info.header.sym_table_size *
                                                                   sizeof(fw_img_v1_sym_table_t));
    if (boot_state.fw_info.sym_table == NULL)
    {
        return BSP_STATUS_FAIL;
    }

    // malloc enough memory to hold the alg_id list, using the alg_id_list_size in the fw_img header
    boot_state.fw_info.alg_id_list = (uint32_t *) bsp_malloc(boot_state.fw_info.header.alg_id_list_size * sizeof(uint32_t));
    if (boot_state.fw_info.alg_id_list == NULL)
    {
        return BSP_STATUS_FAIL;
    }

    // Finally malloc enough memory to hold the largest data block in the fw_img being processed.
    // This may have been configured during fw_img creation.
    // If your control interface has specific memory requirements (dma-able, etc), then this memory
    // should adhere to them.
    // From fw_img_v2 forward, the max_block_size is stored in the fw_img header itself
    boot_state.block_data_size = boot_state.fw_info.header.max_block_size;
    boot_state.block_data = (uint8_t *) bsp_malloc(boot_state.block_data_size);
    if (boot_state.block_data == NULL)
    {
        return BSP_STATUS_FAIL;
    }

    while (fw_img < fw_img_end)
    {
        // Start processing the rest of the fw_img
        ret = fw_img_process(&boot_state);
        if (ret == FW_IMG_STATUS_DATA_READY)
        {
            // Data is ready to be sent to the device, so pass it to the driver
            ret = cs47l63_write_block(&cs47l63_driver, boot_state.block.block_addr,
                                      boot_state.block_data, boot_state.block.block_size);
            if (ret == CS47L63_STATUS_FAIL)
            {
                return BSP_STATUS_FAIL;
            }
            // There is still more data in this fw_img block, so don't provide new data
            continue;
        }
        if (ret == FW_IMG_STATUS_FAIL)
        {
            return BSP_STATUS_FAIL;
        }

        // This fw_img block has been processed, so fetch the next block.
        // In this example, we just increment the pointer.
        fw_img += write_size;

        if (ret == FW_IMG_STATUS_NODATA)
        {
            if (fw_img_end - fw_img < write_size)
            {
                write_size = fw_img_end - fw_img;
            }

            boot_state.fw_img_blocks = (uint8_t *) fw_img;
            boot_state.fw_img_blocks_size = write_size;
        }
    }

    // fw_img processing is complete, so inform the driver and pass it the fw_info block
    ret = cs47l63_boot(&cs47l63_driver, 1, &boot_state.fw_info);

    return ret;
}


static uint32_t wait_fll(uint32_t wait_value)
{
    uint32_t ret, val, wait_count = 20;
    while (wait_count > 0)
    {
        ret = cs47l63_read_reg(&cs47l63_driver, CS47L63_IRQ1_STS_6, &val);
        if (ret != CS47L63_STATUS_OK)
        {
            return CS47L63_STATUS_FAIL;
        }

        if ((val & CS47L63_FLL1_LOCK_STS1_MASK) == wait_value)
        {
            return CS47L63_STATUS_OK;
        }
        --wait_count;
        bsp_set_timer(5, NULL, NULL);
    }

    return CS47L63_STATUS_FAIL;
}

static uint32_t enable_fll(void)
{
    uint32_t ret, fll1_control2_mask, fll1_control2, fll1_control3_mask, fll1_control3, fll1_control4_mask, fll1_control4;
    fll1_control2_mask = CS47L63_FLL1_LOCKDET_THR_MASK
                       | CS47L63_FLL1_PHASEDET_MASK
                       | CS47L63_FLL1_REFCLK_DIV_MASK
                       | CS47L63_FLL1_N_MASK;
    fll1_control2 = (8 << CS47L63_FLL1_LOCKDET_SHIFT)
                  | (1 << CS47L63_FLL1_PHASEDET_SHIFT)
                  | (0 << CS47L63_FLL1_REFCLK_DIV_SHIFT)
                  | (4 << CS47L63_FLL1_N_SHIFT);
    ret = cs47l63_update_reg(&cs47l63_driver, CS47L63_FLL1_CONTROL2, fll1_control2_mask, fll1_control2);
    if (ret != CS47L63_STATUS_OK)
    {
        return CS47L63_STATUS_FAIL;
    }

    fll1_control3_mask = CS47L63_FLL1_LAMBDA_MASK | CS47L63_FLL1_THETA_MASK;
    fll1_control3 = (1 << CS47L63_FLL1_LAMBDA_SHIFT) | (0 << CS47L63_FLL1_THETA_SHIFT);
    ret = cs47l63_update_reg(&cs47l63_driver, CS47L63_FLL1_CONTROL3, fll1_control3_mask, fll1_control3);
    if (ret != CS47L63_STATUS_OK)
    {
        return CS47L63_STATUS_FAIL;
    }

    fll1_control4_mask = CS47L63_FLL1_FD_GAIN_COARSE_MASK
                       | CS47L63_FLL1_HP_MASK
                       | CS47L63_FLL1_FB_DIV_MASK;
    fll1_control4 = (0x21F0 << CS47L63_FLL1_FD_GAIN_COARSE_SHIFT)
                  | (1 << CS47L63_FLL1_HP_SHIFT)
                  | (1 << CS47L63_FLL1_FB_DIV_SHIFT);
    ret = cs47l63_update_reg(&cs47l63_driver, CS47L63_FLL1_CONTROL4, fll1_control4_mask, fll1_control4);
    if (ret != CS47L63_STATUS_OK)
    {
        return CS47L63_STATUS_FAIL;
    }

    // Use internal oscillator
    ret = cs47l63_update_reg(&cs47l63_driver,
                             CS47L63_FLL1_CONTROL2,
                             CS47L63_FLL1_REFCLK_SRC_MASK,
                             (2 << CS47L63_FLL1_REFCLK_SRC_SHIFT));
    if (ret != CS47L63_STATUS_OK)
    {
        return CS47L63_STATUS_FAIL;
    }

    ret = cs47l63_update_reg(&cs47l63_driver, CS47L63_RCO_CTRL1, CS47L63_RCO_EN_MASK, CS47L63_RCO_EN_MASK);
    if (ret != CS47L63_STATUS_OK)
    {
        return CS47L63_STATUS_FAIL;
    }

    ret = cs47l63_update_reg(&cs47l63_driver, CS47L63_FLL1_CONTROL1, CS47L63_FLL1_EN_MASK, CS47L63_FLL1_EN);
    if (ret != CS47L63_STATUS_OK)
    {
        return CS47L63_STATUS_FAIL;
    }

    ret = cs47l63_update_reg(&cs47l63_driver,
                             CS47L63_FLL1_CONTROL2,
                             CS47L63_FLL1_LOCKDET_MASK,
                             CS47L63_FLL1_LOCKDET);
    if (ret != CS47L63_STATUS_OK)
    {
        return CS47L63_STATUS_FAIL;
    }

    ret = cs47l63_update_reg(&cs47l63_driver,
                             CS47L63_FLL1_CONTROL1,
                             CS47L63_FLL1_CTRL_UPD_MASK,
                             CS47L63_FLL1_CTRL_UPD);
    if (ret != CS47L63_STATUS_OK)
    {
        return CS47L63_STATUS_FAIL;
    }

    ret = cs47l63_update_reg(&cs47l63_driver, CS47L63_FLL1_CONTROL1, CS47L63_FLL1_HOLD_MASK, 0);
    if (ret != CS47L63_STATUS_OK)
    {
        return CS47L63_STATUS_FAIL;
    }

    // Wait for the FLL to be enabled
    ret = wait_fll(CS47L63_FLL1_LOCK_STS1);
    if (ret != CS47L63_STATUS_OK)
    {
        return CS47L63_STATUS_FAIL;
    }

    return ret;
}

static uint32_t disable_fll(void)
{
    uint32_t ret;
    ret = cs47l63_update_reg(&cs47l63_driver, CS47L63_FLL1_CONTROL1, CS47L63_FLL1_HOLD_MASK, CS47L63_FLL1_HOLD);
    if (ret != CS47L63_STATUS_OK)
    {
        return CS47L63_STATUS_FAIL;
    }

    ret = cs47l63_update_reg(&cs47l63_driver, CS47L63_FLL1_CONTROL1, CS47L63_FLL1_EN, 0);
    if (ret != CS47L63_STATUS_OK)
    {
        return CS47L63_STATUS_FAIL;
    }

    ret = cs47l63_update_reg(&cs47l63_driver,
                             CS47L63_FLL1_CONTROL2,
                             CS47L63_FLL1_LOCKDET_MASK,
                             CS47L63_FLL1_LOCKDET);
    if (ret != CS47L63_STATUS_OK)
    {
        return CS47L63_STATUS_FAIL;
    }

    ret = cs47l63_update_reg(&cs47l63_driver, CS47L63_FLL1_CONTROL1, CS47L63_FLL1_EN_MASK, 0);
    if (ret != CS47L63_STATUS_OK)
    {
        return CS47L63_STATUS_FAIL;
    }

    ret = cs47l63_update_reg(&cs47l63_driver, CS47L63_RCO_CTRL1, CS47L63_RCO_EN_MASK, 0);
    if (ret != CS47L63_STATUS_OK)
    {
        return CS47L63_STATUS_FAIL;
    }

    // Wait for the FLL to be disabled
    ret = wait_fll(0);
    if (ret != CS47L63_STATUS_OK)
    {
        return CS47L63_STATUS_FAIL;
    }

    return ret;
}

uint32_t bsp_dut_use_case(uint32_t use_case)
{
    uint32_t ret = BSP_STATUS_OK;

    switch (use_case) {
        case BSP_USE_CASE_TG_HP_EN:
            cs47l63_update_reg(&cs47l63_driver,
                               CS47L63_DSP_CLOCK1,
                               CS47L63_DSP_CLK_FREQ_MASK,
                               (0x24DD << CS47L63_DSP_CLK_FREQ_SHIFT));
            // TODO: Use the FLL implmentation when it is available
            enable_fll();
            cs47l63_update_reg(&cs47l63_driver, CS47L63_SYSTEM_CLOCK1, CS47L63_SYSCLK_EN_MASK, CS47L63_SYSCLK_EN);
            cs47l63_update_reg(&cs47l63_driver, CS47L63_OUT1L_INPUT1, CS47L63_OUT1L_SRC1_MASK, CS47L63_SRC_TONE_GENERATOR1);
            cs47l63_update_reg(&cs47l63_driver, CS47L63_TONE_GENERATOR1, CS47L63_TONE1_EN_MASK, CS47L63_TONE1_EN);
            cs47l63_write_reg(&cs47l63_driver, CS47L63_OUTPUT_ENABLE_1, CS47L63_OUT1L_EN_MASK);
            cs47l63_write_reg(&cs47l63_driver, CS47L63_OUT1L_VOLUME_1, CS47L63_OUT_VU | 0x60);
            break;
        case BSP_USE_CASE_TG_HP_DIS:
            cs47l63_write_reg(&cs47l63_driver, CS47L63_OUT1L_VOLUME_1, CS47L63_OUT_VU | CS47L63_OUT1L_MUTE | 0x60);
            cs47l63_write_reg(&cs47l63_driver, CS47L63_OUTPUT_ENABLE_1, 0);
            cs47l63_update_reg(&cs47l63_driver, CS47L63_TONE_GENERATOR1, CS47L63_TONE1_EN_MASK, 0);
            cs47l63_update_reg(&cs47l63_driver, CS47L63_OUT1L_INPUT1, CS47L63_OUT1L_SRC1_MASK, 0);
            cs47l63_update_reg(&cs47l63_driver, CS47L63_SYSTEM_CLOCK1, CS47L63_SYSCLK_EN_MASK, 0);
            // TODO: Use the FLL implmentation when it is available
            disable_fll();
            break;
        case BSP_USE_CASE_DSP_PRELOAD_PT_EN:
            ret = cs47l63_power(&cs47l63_driver, 1 , CS47L63_POWER_MEM_ENA);
            bsp_dut_boot();
            break;
        case BSP_USE_CASE_DSP_PRELOAD_PT_DIS:
            ret = cs47l63_power(&cs47l63_driver, 1 , CS47L63_POWER_MEM_DIS);
            break;
        case BSP_USE_CASE_TG_DSP_HP_EN:
            cs47l63_update_reg(&cs47l63_driver,
                               CS47L63_DSP_CLOCK1,
                               CS47L63_DSP_CLK_FREQ_MASK,
                               (0x24DD << CS47L63_DSP_CLK_FREQ_SHIFT));
            // TODO: Use the FLL implmentation when it is available
            enable_fll();
            cs47l63_update_reg(&cs47l63_driver, CS47L63_SYSTEM_CLOCK1, CS47L63_SYSCLK_EN_MASK, CS47L63_SYSCLK_EN);
            cs47l63_update_reg(&cs47l63_driver, CS47L63_DSP1RX1_INPUT1, CS47L63_DSP1RX1_SRC1_MASK, CS47L63_SRC_TONE_GENERATOR1);
            cs47l63_update_reg(&cs47l63_driver, CS47L63_DSP1RX2_INPUT1, CS47L63_DSP1RX2_SRC1_MASK, CS47L63_SRC_TONE_GENERATOR1);
            cs47l63_update_reg(&cs47l63_driver, CS47L63_OUT1L_INPUT1, CS47L63_OUT1L_SRC1_MASK, CS47L63_DSP1_CHANNEL1);
            cs47l63_update_reg(&cs47l63_driver, CS47L63_OUT1L_INPUT2, CS47L63_OUT1L_SRC1_MASK, CS47L63_DSP1_CHANNEL2);
            cs47l63_update_reg(&cs47l63_driver, CS47L63_TONE_GENERATOR1, CS47L63_TONE1_EN_MASK, CS47L63_TONE1_EN);
            cs47l63_write_reg(&cs47l63_driver, CS47L63_OUTPUT_ENABLE_1, CS47L63_OUT1L_EN_MASK);
            cs47l63_write_reg(&cs47l63_driver, CS47L63_OUT1L_VOLUME_1, CS47L63_OUT_VU | 0x60);
            ret = cs47l63_power(&cs47l63_driver, 1, CS47L63_POWER_UP);
            break;
        case BSP_USE_CASE_TG_DSP_HP_DIS:
            cs47l63_write_reg(&cs47l63_driver, CS47L63_OUT1L_VOLUME_1, CS47L63_OUT_VU | CS47L63_OUT1L_MUTE | 0x60);
            cs47l63_write_reg(&cs47l63_driver, CS47L63_OUTPUT_ENABLE_1, 0);
            cs47l63_update_reg(&cs47l63_driver, CS47L63_TONE_GENERATOR1, CS47L63_TONE1_EN_MASK, 0);
            cs47l63_update_reg(&cs47l63_driver, CS47L63_OUT1L_INPUT1, CS47L63_OUT1L_SRC1_MASK, 0);
            cs47l63_update_reg(&cs47l63_driver, CS47L63_OUT1L_INPUT2, CS47L63_OUT1L_SRC1_MASK, 0);
            cs47l63_update_reg(&cs47l63_driver, CS47L63_DSP1RX1_INPUT1, CS47L63_DSP1RX1_SRC1_MASK, 0);
            cs47l63_update_reg(&cs47l63_driver, CS47L63_DSP1RX2_INPUT1, CS47L63_DSP1RX2_SRC1_MASK, 0);
            cs47l63_update_reg(&cs47l63_driver, CS47L63_SYSTEM_CLOCK1, CS47L63_SYSCLK_EN_MASK, 0);
            // TODO: Use the FLL implmentation when it is available
            disable_fll();
            ret = cs47l63_power(&cs47l63_driver, 1, CS47L63_POWER_DOWN);
            break;
        case BSP_USE_CASE_MIC_DSP_HP_EN:
            ret = cs47l63_power(&cs47l63_driver, 1, CS47L63_POWER_UP);
            break;
        case BSP_USE_CASE_MIC_DSP_HP_DIS:
            ret = cs47l63_power(&cs47l63_driver, 1, CS47L63_POWER_DOWN);
            break;
        default:
            break;
    }

    return ret;
}

uint32_t bsp_dut_process(void)
{
    uint32_t ret;

    ret = cs47l63_process(&cs47l63_driver);

    if (ret != CS47L63_STATUS_OK)
    {
        return BSP_STATUS_FAIL;
    }

    return BSP_STATUS_OK;
}
