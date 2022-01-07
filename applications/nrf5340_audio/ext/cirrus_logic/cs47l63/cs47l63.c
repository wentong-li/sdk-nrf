/**
 * @file cs47l63.c
 *
 * @brief The CS47L63 Driver module
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
#include <stddef.h>
#include "cs47l63.h"
#include "bsp_driver_if.h"
#include "string.h"

/***********************************************************************************************************************
 * LOCAL LITERAL SUBSTITUTIONS
 **********************************************************************************************************************/

/**
 * @defgroup CS47L63_POLL_
 * @brief Polling constants for polling times and counts
 *
 * @{
 */
#define CS47L63_POLL_ACK_CTRL_MS                (10)    ///< Delay in ms between polling ACK controls
#define CS47L63_POLL_ACK_CTRL_MAX               (10)    ///< Maximum number of times to poll ACK controls
/** @} */

/**
 * @defgroup CS47L63_REGION_LOCK_
 * @brief Region lock codes
 *
 * @{
 */
#define CS47L63_REGION_UNLOCK_CODE0             (0x5555)       ///< First code required to unlock a region
#define CS47L63_REGION_UNLOCK_CODE1             (0xAAAA)       ///< Second code required to unlock a region
#define CS47L63_REGION_LOCK_CODE                (0x0)          ///< A code that will lock a region
/** @} */

/***********************************************************************************************************************
 * ENUMS, STRUCTS, UNIONS, TYPEDEFS
 **********************************************************************************************************************/

/**
 * @defgroup CS47L63_reg_sequence_
 * @brief A structure for describing a register address and value to write to it
 *
 * @{
 */
typedef struct
{
    uint32_t reg_addr;
    uint32_t reg_val;
    uint32_t delay_us;
} cs47l63_reg_sequence_t;
/** @} */

/***********************************************************************************************************************
 * LOCAL VARIABLES
 **********************************************************************************************************************/

/**
 * @defgroup CS47L63_DSP_RAM_BANK_
 * @brief CS47L63 DSP ram banks register offset array
 *
 * @{
 */
static const cs47l63_dsp_ram_bank_t cs47l63_dsp1_ram_banks[] = {
    {CS47L63_DSP1_XM_SRAM_IBUS_SETUP_1, CS47L63_DSP1_XM_SRAM_IBUS_SETUP_11},
    {CS47L63_DSP1_YM_SRAM_IBUS_SETUP_1, CS47L63_DSP1_YM_SRAM_IBUS_SETUP_6},
    {CS47L63_DSP1_PM_SRAM_IBUS_SETUP_1, CS47L63_DSP1_PM_SRAM_IBUS_SETUP_5}
};
/** @} */

/**
 * @defgroup CS47L63_DSP_RAM_BANK_
 * @brief Number of DSP ram bank entries
 *
 * @{
 */
#define N_DSP1_RAM_BANKS (sizeof(cs47l63_dsp1_ram_banks) / sizeof(cs47l63_dsp_ram_bank_t))
/** @} */

/**
 * @defgroup CS47L63_DSP_RAM_BANK_
 * @brief Flag representing both odd and even parts of a DSP ram bank
 *
 * @{
 */
#define CS47L63_DSP_RAM_BANK_ODD_EVEN           (CS47L63_DSP1_XM_SRAM_IBUS_O_EXT_N_1 \
                                               | CS47L63_DSP1_XM_SRAM_IBUS_E_EXT_N_1)
/** @} */

/**
 * CS47L63 interrupt regs to check
 *
 * Each element is in format of {irq register offset from base, mask, flag associated with this event}
 *
 * @see cs47l63_event_handler
 */
const irq_reg_t cs47l63_event_data[] =
{
    { 0x4,  CS47L63_BOOT_DONE_MASK1_MASK,         CS47L63_EVENT_FLAG_BOOT_DONE},    //< CS47L63_IRQ1_EINT_2
    { 0x0,  CS47L63_SYSCLK_FAIL_MASK1_MASK,       CS47L63_EVENT_FLAG_SYSCLK_FAIL},  //< CS47L63_IRQ1_EINT_1
    { 0x0,  CS47L63_SYSCLK_ERR_MASK1_MASK,        CS47L63_EVENT_FLAG_SYSCLK_ERR},   //< CS47L63_IRQ1_EINT_1
    { 0x0,  CS47L63_CTRLIF_ERR_MASK1_MASK,        CS47L63_EVENT_FLAG_CTRLIF_ERR},   //< CS47L63_IRQ1_EINT_1
    { 0x18, CS47L63_DSP1_MPU_ERR_MASK1_MASK,      CS47L63_EVENT_FLAG_MPU_ERR},      //< CS47L63_IRQ1_EINT_7
    { 0x20, CS47L63_DSP1_IRQ0_MASK1_MASK,         CS47L63_EVENT_FLAG_DSP1_IRQ0},    //< CS47L63_IRQ1_EINT_9
    { 0x18, CS47L63_DSP1_WDT_EXPIRE_STS1_MASK,    CS47L63_EVENT_FLAG_WDT_EXPIRE},   //< CS47L63_IRQ1_EINT_7
    { 0x18, CS47L63_DSP1_AHB_SYS_ERR_MASK1_MASK,  CS47L63_EVENT_FLAG_AHB_SYS_ERR},  //< CS47L63_IRQ1_EINT_7
    { 0x18, CS47L63_DSP1_AHB_PACK_ERR_MASK1_MASK, CS47L63_EVENT_FLAG_AHB_PACK_ERR}  //< CS47L63_IRQ1_EINT_7
};

/**
 * Number of entries in the CS47L63 interrupt regs structure
 *
 * @see cs47l63_event_handler
 */
#define N_IRQ_REGS  ((sizeof(cs47l63_event_data)) / (sizeof(irq_reg_t)))

/***********************************************************************************************************************
 * LOCAL FUNCTIONS
 **********************************************************************************************************************/
#ifdef CS47L63_USEFUL_UNUSED

/**
 * Find if an algorithm is in the algorithm list and return true if it is.
 * Returns false if not.
 */
static bool find_algid(fw_img_info_t *fw_info, uint32_t algid_id)
{
    if (fw_info)
    {
        for (uint32_t i = 0; i < fw_info->header.alg_id_list_size; i++)
        {
            if (fw_info->alg_id_list[i] == algid_id)
                return true;
        }
    }

    return false;
}

bool cs47l63_find_algid(cs47l63_t *driver, uint32_t dsp_core, uint32_t algid_id)
{
    bool ret;

    if (dsp_core > CS47L63_NUM_DSP)
        return false;

    if (dsp_core != 0)
    {
        return find_algid(driver->dsp_info[dsp_core - 1].fw_info, algid_id);
    }
    else
    {
        // search all DSPs if dsp_core is 0
        for (uint32_t i = 0; i < CS47L63_NUM_DSP; i++)
        {
            ret = find_algid(driver->dsp_info[i].fw_info, algid_id);

            if (ret)
                return true;
        }
    }

    return false;
}
#endif

/**
 * Find if a symbol is in the symbol table and return its address if it is.
 *
 * This will search through the symbol table pointed to in the 'fw_info' member of the driver state and return
 * the control port register address to use for access.  The 'symbol_id' parameter must be from the group CS47L63_SYM_.
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] symbol_id        id of symbol to search for
 *
 * @return
 * - non-0 - symbol register address
 * - 0 - symbol not found.
 *
 */
static uint32_t find_symbol(fw_img_info_t *fw_info, uint32_t symbol_id)
{
    if (fw_info)
    {
        for (uint32_t i = 0; i < fw_info->header.sym_table_size; i++)
        {
            if (fw_info->sym_table[i].sym_id == symbol_id)
                return fw_info->sym_table[i].sym_addr;
        }
    }

    return 0;
}

uint32_t cs47l63_find_symbol(cs47l63_t *driver, uint32_t dsp_core, uint32_t symbol_id)
{
    uint32_t ret;

    if (dsp_core > CS47L63_NUM_DSP)
        return false;

    if (dsp_core != 0)
    {
        return find_symbol(driver->dsp_info[dsp_core - 1].fw_info, symbol_id);
    }
    else
    {
        // search all DSPs if dsp_core is 0
        for (uint32_t i = 0; i < CS47L63_NUM_DSP; i++)
        {
            ret = find_symbol(driver->dsp_info[i].fw_info, symbol_id);

            if (ret)
                return ret;
        }
    }

    return 0;
}

/**
 * Writes from byte array to consecutive number of Control Port memory addresses
 *
 * This control port call only supports non-blocking calls.
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] addr             32-bit address to be read
 * @param [in] bytes            Pointer to array of bytes to write via Control Port bus
 * @param [in] length           Number of bytes to write
 *
 * @return
 * - CS47L63_STATUS_FAIL        if the call to BSP failed
 * - CS47L63_STATUS_OK          otherwise
 *
 * @warning Contains platform-dependent code.
 *
 */
static uint32_t cs47l63_cp_bulk_write_block(cs47l63_t *driver, uint32_t addr, uint8_t *bytes, uint32_t length)
{
    uint32_t ret = CS47L63_STATUS_OK;
    uint32_t bsp_status;
    cs47l63_bsp_config_t *b = &(driver->config.bsp_config);
    uint8_t write_buffer[4];

    /*
     * Place contents of uint32_t 'addr' to Big-Endian format byte stream required for Control Port
     * transaction.
     */
    write_buffer[0] = GET_BYTE_FROM_WORD(addr, 3);
    write_buffer[1] = GET_BYTE_FROM_WORD(addr, 2);
    write_buffer[2] = GET_BYTE_FROM_WORD(addr, 1);
    write_buffer[3] = GET_BYTE_FROM_WORD(addr, 0);

    bsp_status = bsp_driver_if_g->spi_write(b->bsp_dev_id,
                                            &write_buffer[0],
                                            4,
                                            bytes,
                                            length,
                                            4);

    if (bsp_status == BSP_STATUS_FAIL)
    {
        ret = CS47L63_STATUS_FAIL;
    }

    return ret;
}

/**
 * Reads byte array from consecutive number of addresses
 *
 * This control port call only supports non-blocking calls.
 *
 * @param [in]  driver           Pointer to the driver state
 * @param [in]  addr             32-bit address to be read
 * @param [in]  length           Number of bytes to read
 * @param [out] bytes            Pointer to array of bytes to store received data
 *
 * @return
 * - CS47L63_STATUS_FAIL         if the call to BSP failed
 * - CS47L63_STATUS_OK           otherwise
 *
 * @warning Contains platform-dependent code.
 *
 */
static uint32_t cs47l63_cp_bulk_read_block(cs47l63_t *driver, uint32_t addr, uint32_t length, uint8_t *bytes)
{
    uint32_t ret = CS47L63_STATUS_OK;
    uint32_t bsp_status;
    cs47l63_bsp_config_t *b = &(driver->config.bsp_config);
    uint8_t write_buffer[4];

    /*
     * Place contents of uint32_t 'addr' to Big-Endian format byte stream required for Control Port
     * transaction.
     */
    write_buffer[0] = GET_BYTE_FROM_WORD(addr, 3);
    write_buffer[1] = GET_BYTE_FROM_WORD(addr, 2);
    write_buffer[2] = GET_BYTE_FROM_WORD(addr, 1);
    write_buffer[3] = GET_BYTE_FROM_WORD(addr, 0);

    // Set the R/W bit
    write_buffer[0] = 0x80 | write_buffer[0];

    bsp_status = bsp_driver_if_g->spi_read(b->bsp_dev_id,
                                           &write_buffer[0],
                                           4,
                                           bytes,
                                           length,
                                           4);

    if (bsp_status == BSP_STATUS_FAIL)
    {
        ret = CS47L63_STATUS_FAIL;
    }

    return ret;
}


/**
 * Notify the driver when the CS47L63 INTb GPIO drops low.
 *
 * This callback is registered with the BSP in the register_gpio_cb() API call.
 *
 * The primary task of this callback is to transition the driver mode from CS47L63_MODE_HANDLING_CONTROLS to
 * CS47L63_MODE_HANDLING_EVENTS, in order to signal to the main thread to process events.
 *
 * @param [in] status           BSP status for the INTb IRQ.
 * @param [in] cb_arg           A pointer to callback argument registered.  For the driver, this arg is used for a
 *                              pointer to the driver state cs47l63_t.
 *
 * @return none
 *
 * @see bsp_driver_if_t member register_gpio_cb.
 * @see bsp_callback_t
 *
 */
static void cs47l63_irq_callback(uint32_t status, void *cb_arg)
{
    cs47l63_t *d;

    d = (cs47l63_t *) cb_arg;

    if (status == BSP_STATUS_OK)
    {
        // Switch driver mode to CS47L63_MODE_HANDLING_EVENTS
        d->mode = CS47L63_MODE_HANDLING_EVENTS;
    }

    return;
}

/**
 * Reads the contents of a single register/memory address
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] addr             32-bit address to be read
 * @param [out] val             Pointer to register value read
 *
 * @return
 * - CS47L63_STATUS_FAIL        if the call to BSP failed
 * - CS47L63_STATUS_OK          otherwise
 *
 * @warning Contains platform-dependent code.
 *
 */
uint32_t cs47l63_read_reg(cs47l63_t *driver, uint32_t addr, uint32_t *val)
{
    uint32_t ret = CS47L63_STATUS_FAIL;
    cs47l63_bsp_config_t *b = &(driver->config.bsp_config);
    uint8_t write_buffer[4];
    uint8_t read_buffer[4];

    /*
     * Place contents of uint32_t 'addr' to Big-Endian format byte stream required for Control Port
     * transaction.
     */
    write_buffer[0] = GET_BYTE_FROM_WORD(addr, 3);
    write_buffer[1] = GET_BYTE_FROM_WORD(addr, 2);
    write_buffer[2] = GET_BYTE_FROM_WORD(addr, 1);
    write_buffer[3] = GET_BYTE_FROM_WORD(addr, 0);

    // Currently only SPI transactions are supported
    if (b->bus_type == CS47L63_BUS_TYPE_SPI)
    {
        uint32_t bsp_status;

        // Set the R/W bit
        write_buffer[0] = 0x80 | write_buffer[0];

        bsp_status = bsp_driver_if_g->spi_read(b->bsp_dev_id,
                                               &write_buffer[0],
                                               4,
                                               &read_buffer[0],
                                               4,
                                               4);
        if (BSP_STATUS_OK == bsp_status)
        {
            /*
             * Switch from Big-Endian format byte stream required for Control Port transaction to
             * uint32_t 'val'
             */
            ADD_BYTE_TO_WORD(*val, read_buffer[0], 3);
            ADD_BYTE_TO_WORD(*val, read_buffer[1], 2);
            ADD_BYTE_TO_WORD(*val, read_buffer[2], 1);
            ADD_BYTE_TO_WORD(*val, read_buffer[3], 0);

            ret = CS47L63_STATUS_OK;
        }
    }

    return ret;
}

/**
 * Writes the contents of a single register/memory address
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] addr             32-bit address to be written
 * @param [in] val              32-bit value to be written
 *
 * @return
 * - CS47L63_STATUS_FAIL        if the call to BSP failed
 * - CS47L63_STATUS_OK          otherwise
 *
 * @warning Contains platform-dependent code.
 *
 */
uint32_t cs47l63_write_reg(cs47l63_t *driver, uint32_t addr, uint32_t val)
{
    uint32_t ret = CS47L63_STATUS_FAIL;
    uint32_t bsp_status = BSP_STATUS_OK;
    cs47l63_bsp_config_t *b = &(driver->config.bsp_config);
    uint8_t write_buffer[8];

    /*
     * Place contents of uint32_t 'addr' and 'val' to Big-Endian format byte stream required for Control Port
     * transaction.
     */
    write_buffer[0] = GET_BYTE_FROM_WORD(addr, 3);
    write_buffer[1] = GET_BYTE_FROM_WORD(addr, 2);
    write_buffer[2] = GET_BYTE_FROM_WORD(addr, 1);
    write_buffer[3] = GET_BYTE_FROM_WORD(addr, 0);

    write_buffer[4] = GET_BYTE_FROM_WORD(val, 3);
    write_buffer[5] = GET_BYTE_FROM_WORD(val, 2);
    write_buffer[6] = GET_BYTE_FROM_WORD(val, 1);
    write_buffer[7] = GET_BYTE_FROM_WORD(val, 0);

    // Currently only SPI transactions are supported
    if (b->bus_type == CS47L63_BUS_TYPE_SPI)
    {
        bsp_status = bsp_driver_if_g->spi_write(b->bsp_dev_id,
                                                &write_buffer[0],
                                                4,
                                                &write_buffer[4],
                                                4,
                                                4);
    }

    if (BSP_STATUS_OK == bsp_status)
    {
        ret = CS47L63_STATUS_OK;
    }

    return ret;
}

uint32_t cs47l63_update_reg(cs47l63_t *driver, uint32_t addr, uint32_t mask, uint32_t val)
{
    uint32_t tmp, ret, orig;

    ret = cs47l63_read_reg(driver, addr, &orig);
    if (ret == CS47L63_STATUS_FAIL)
    {
        return ret;
    }

    tmp = (orig & ~mask) | val;

    if (tmp != orig)
    {
        ret = cs47l63_write_reg(driver, addr, tmp);
        if (ret == CS47L63_STATUS_FAIL)
        {
            return ret;
        }
    }

    return CS47L63_STATUS_OK;
}

/**
 * Writes the contents of a single register/memory address that ACK's with a default value
 *
 * This performs the same function as cs47l63_write_reg, with the addition of, after writing the value to the address
 * specified, will periodically read back the register and verify that a default value is restored (0)
 * indicating the write succeeded.
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] addr             32-bit address to be written
 * @param [in] val              32-bit value to be written
 *
 * @return
 * - CS47L63_STATUS_FAIL        if the call to BSP failed or if register is never restored to 0
 * - CS47L63_STATUS_OK          otherwise
 *
 * @warning Contains platform-dependent code.
 *
 */
/* static */ uint32_t cs47l63_write_acked_reg(cs47l63_t *driver, uint32_t addr, uint32_t val)
{
    int count;
    uint32_t temp_val;

    cs47l63_write_reg(driver, addr, val);

    for (count = 0 ; count < CS47L63_POLL_ACK_CTRL_MAX; count++)
    {
        bsp_driver_if_g->set_timer(CS47L63_POLL_ACK_CTRL_MS, NULL, NULL);

        cs47l63_read_reg(driver, addr, &temp_val);
        if (temp_val == 0)
        {
            return CS47L63_STATUS_OK;
        }
    }
    return CS47L63_STATUS_FAIL;
}

/**
 * Writes the contents of multiple register/memory addresses
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] reg_sequence     An array of register/address write entries
 * @param [in] length           The length of the array
 *
 * @return
 * - CS47L63_STATUS_FAIL        if the call to BSP failed
 * - CS47L63_STATUS_OK          otherwise
 *
 */
uint32_t cs47l63_write_reg_sequence(cs47l63_t *driver, cs47l63_reg_sequence_t *reg_sequence, uint32_t length)
{
    uint32_t ret = CS47L63_STATUS_FAIL;
    cs47l63_reg_sequence_t *sequence_entry = reg_sequence;

    for (uint32_t index = 0; index < length; ++index)
    {
        // Write the next register/address in the sequence
        ret = cs47l63_write_reg(driver, sequence_entry->reg_addr, sequence_entry->reg_val);
        if (ret != CS47L63_STATUS_OK)
        {
            return CS47L63_STATUS_FAIL;
        }

        // Delay if required
        if (sequence_entry->delay_us > 0)
        {
            bsp_driver_if_g->set_timer(sequence_entry->delay_us, NULL, NULL);
        }
        ++sequence_entry;
    }

    return ret;
}

/**
 * Power up from Standby
 *
 * This function performs all necessary steps to transition the CS47L63 to be ready to generate events.
 * Completing this results in the driver transition to POWER_UP state.
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] dsp_info         Pointer to the dsp info state
 *
 * @return
 * - CS47L63_STATUS_FAIL if:
 *      - OTP_BOOT_DONE is not set
 *      - DSP Scratch register is not cleared
 * - CS47L63_STATUS_OK          otherwise
 *
 */
static uint32_t cs47l63_power_up(cs47l63_t *driver, cs47l63_dsp_t *dsp_info)
{
    uint32_t ret, val;
    cs47l63_reg_sequence_t config[] = {
        { dsp_info->base_addr + CS47L63_DSP_OFF_MPU_LOCK_CONFIG,     CS47L63_REGION_UNLOCK_CODE0, 0},
        { dsp_info->base_addr + CS47L63_DSP_OFF_MPU_LOCK_CONFIG,     CS47L63_REGION_UNLOCK_CODE1, 0},
        { dsp_info->base_addr + CS47L63_DSP_OFF_MPU_XMEM_ACCESS_0,   0xFFFFFFFF,                  0},
        { dsp_info->base_addr + CS47L63_DSP_OFF_MPU_YMEM_ACCESS_0,   0xFFFFFFFF,                  0},
        { dsp_info->base_addr + CS47L63_DSP_OFF_MPU_WINDOW_ACCESS_0, 0x0,                         0},
        { dsp_info->base_addr + CS47L63_DSP_OFF_MPU_XREG_ACCESS_0,   0x0,                         0},
        { dsp_info->base_addr + CS47L63_DSP_OFF_MPU_YREG_ACCESS_0,   0x0,                         0},
        { dsp_info->base_addr + CS47L63_DSP_OFF_MPU_XMEM_ACCESS_1,   0xFFFFFFFF,                  0},
        { dsp_info->base_addr + CS47L63_DSP_OFF_MPU_YMEM_ACCESS_1,   0xFFFFFFFF,                  0},
        { dsp_info->base_addr + CS47L63_DSP_OFF_MPU_WINDOW_ACCESS_1, 0x0,                         0},
        { dsp_info->base_addr + CS47L63_DSP_OFF_MPU_XREG_ACCESS_1,   0x0,                         0},
        { dsp_info->base_addr + CS47L63_DSP_OFF_MPU_YREG_ACCESS_1,   0x0,                         0},
        { dsp_info->base_addr + CS47L63_DSP_OFF_MPU_XMEM_ACCESS_2,   0xFFFFFFFF,                  0},
        { dsp_info->base_addr + CS47L63_DSP_OFF_MPU_YMEM_ACCESS_2,   0xFFFFFFFF,                  0},
        { dsp_info->base_addr + CS47L63_DSP_OFF_MPU_WINDOW_ACCESS_2, 0x0,                         0},
        { dsp_info->base_addr + CS47L63_DSP_OFF_MPU_XREG_ACCESS_2,   0x0,                         0},
        { dsp_info->base_addr + CS47L63_DSP_OFF_MPU_YREG_ACCESS_2,   0x0,                         0},
        { dsp_info->base_addr + CS47L63_DSP_OFF_MPU_XMEM_ACCESS_3,   0xFFFFFFFF,                  0},
        { dsp_info->base_addr + CS47L63_DSP_OFF_MPU_YMEM_ACCESS_3,   0xFFFFFFFF,                  0},
        { dsp_info->base_addr + CS47L63_DSP_OFF_MPU_WINDOW_ACCESS_3, 0x0,                         0},
        { dsp_info->base_addr + CS47L63_DSP_OFF_MPU_XREG_ACCESS_3,   0x0,                         0},
        { dsp_info->base_addr + CS47L63_DSP_OFF_MPU_YREG_ACCESS_3,   0x0,                         0},
        { dsp_info->base_addr + CS47L63_DSP_OFF_MPU_LOCK_CONFIG,     CS47L63_REGION_LOCK_CODE,    0}
    };

    // Lock all regions
    ret = cs47l63_write_reg_sequence(driver, config, sizeof(config) / sizeof(cs47l63_reg_sequence_t));
    if (ret != CS47L63_STATUS_OK)
    {
        return CS47L63_STATUS_FAIL;
    }

    ret = cs47l63_update_reg(driver, CS47L63_DSP_CLOCK1, CS47L63_DSP_CLK_EN_MASK, CS47L63_DSP_CLK_EN);
    if (ret != CS47L63_STATUS_OK)
    {
        return CS47L63_STATUS_FAIL;
    }

    ret = cs47l63_read_reg(driver, CS47L63_DSP_CLOCK1, &val);
    if (ret != CS47L63_STATUS_OK)
    {
        return CS47L63_STATUS_FAIL;
    }

    // Get the frequency
    val = (val & CS47L63_DSP_CLK_FREQ_MASK) >> CS47L63_DSP_CLK_FREQ_SHIFT;

    // Set the DSP clock frequency
    ret = cs47l63_update_reg(driver,
                             dsp_info->base_addr + CS47L63_DSP_OFF_CLOCK_FREQ,
                             CS47L63_DSP1_CLK_FREQ_SEL_MASK,
                             val);
    if (ret != CS47L63_STATUS_OK)
    {
        return CS47L63_STATUS_FAIL;
    }

    // Enable the DSP
    ret = cs47l63_update_reg(driver,
                             dsp_info->base_addr + CS47L63_DSP_OFF_CCM_CORE_CONTROL,
                             CS47L63_DSP1_CCM_CORE_EN_MASK,
                             CS47L63_DSP1_CCM_CORE_EN);
    if (ret != CS47L63_STATUS_OK)
    {
        return CS47L63_STATUS_FAIL;
    }

    return ret;
}

/**
 * Power down to Standby
 *
 * This function performs all necessary steps to transition the CS47L63 to be in Standby power mode. This includes
 * disabling clocks to the HALO Core DSP.  Completing this results in the driver transition to CAL_STANDBY or
 * DSP_STANDBY state.
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] dsp_info         Pointer to the dsp info state
 *
 * @return
 * - CS47L63_STATUS_FAIL if:
 *      - Control port activity fails
 *      - Firmware control addresses cannot be resolved by Symbol ID
 * - CS47L63_STATUS_OK          otherwise
 *
 */
static uint32_t cs47l63_power_down(cs47l63_t *driver, cs47l63_dsp_t *dsp_info)
{
    uint32_t ret;

    // Stop Watchdog Timer
    ret = cs47l63_update_reg(driver, dsp_info->base_addr + CS47L63_DSP_OFF_WDT_CONTROL, CS47L63_DSP1_WDT_EN_MASK, 0);
    if (ret != CS47L63_STATUS_OK)
    {
        return CS47L63_STATUS_FAIL;
    }

    // Disable DSP
    ret = cs47l63_update_reg(driver,
                             dsp_info->base_addr + CS47L63_DSP_OFF_CCM_CORE_CONTROL,
                             CS47L63_DSP1_CCM_CORE_EN_MASK,
                             0);
    if (ret != CS47L63_STATUS_OK)
    {
        return CS47L63_STATUS_FAIL;
    }

    ret = cs47l63_update_reg(driver, CS47L63_DSP_OFF_CORE_SOFT_RESET, CS47L63_SFT_RESET_MASK, CS47L63_SFT_RESET_MASK);

    if (ret != CS47L63_STATUS_OK)
    {
        return CS47L63_STATUS_FAIL;
    }

    ret = cs47l63_update_reg(driver, CS47L63_DSP_CLOCK1, CS47L63_DSP_CLK_EN_MASK, 0);

    if (ret != CS47L63_STATUS_OK)
    {
        return CS47L63_STATUS_FAIL;
    }

    return ret;
}

/**
 * Memory enable
 *
 * This function performs all necessary steps to enable the memory of the DSP core on the CS47L63
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] dsp_info         Pointer to the DSP data structure
 *
 * @return
 * - CS47L63_STATUS_FAIL if:
 *      - Control port activity fails
 * - CS47L63_STATUS_OK          otherwise
 *
 */
static uint32_t cs47l63_power_mem_ena(cs47l63_t *driver, cs47l63_dsp_t *dsp_info)
{
    uint32_t ret = 0;
    const cs47l63_dsp_ram_bank_t *ram_bank_ptr = dsp_info->ram_banks;

    for (uint32_t index = 0; index < dsp_info->n_ram_banks; ++index)
    {
        uint32_t reg_end = ram_bank_ptr->reg_end;
        for (uint32_t reg_addr = ram_bank_ptr->reg_start; reg_addr <= reg_end; reg_addr += 4)
        {
            ret = cs47l63_write_reg(driver, reg_addr, CS47L63_DSP_RAM_BANK_ODD_EVEN);
            if (ret != CS47L63_STATUS_OK)
            {
                return CS47L63_STATUS_FAIL;
            }
        }
        ram_bank_ptr++;
    }

    return ret;
}

/**
 * Memory disable
 *
 * This function performs all necessary steps to disable the memory of the DSP core on the CS47L63.  After
 * calling this function, the contents of DSP memory will be lost.
 *
 * @param [in] driver           Pointer to the driver state
 * @param [in] dsp_info         Pointer to the DSP data structure
 *
 * @return
 * - CS47L63_STATUS_FAIL if:
 *      - Control port activity fails
 * - CS47L63_STATUS_OK          otherwise
 *
 */
static uint32_t cs47l63_power_mem_dis(cs47l63_t *driver, cs47l63_dsp_t *dsp_info)
{
    uint32_t ret = 0;
    const cs47l63_dsp_ram_bank_t *ram_bank_ptr = dsp_info->ram_banks;

    for (uint32_t index = 0; index < dsp_info->n_ram_banks; ++index)
    {
        uint32_t reg_end = ram_bank_ptr->reg_end;
        for (uint32_t reg_addr = ram_bank_ptr->reg_start; reg_addr <= reg_end; reg_addr += 4)
        {
            ret = cs47l63_write_reg(driver, reg_addr, 0);
            if (ret != CS47L63_STATUS_OK)
            {
                return CS47L63_STATUS_FAIL;
            }
        }
        ram_bank_ptr++;
    }

    return ret;
}

/**
 * Handle events indicated by the IRQ pin ALERTb
 *
 * This function performs all steps to handle IRQ and other asynchronous events the driver is aware of,
 * resulting in calling of the notification callback (cs47l63_notification_callback_t).
 *
 * @param [in] driver           Pointer to the driver state
 *
 * @return
 * - CS47L63_STATUS_FAIL        Control port activity fails
 * - CS47L63_STATUS_OK          otherwise
 *
 * @see CS47L63_EVENT_FLAG_
 * @see cs47l63_notification_callback_t
 *
 */
static uint32_t cs47l63_event_handler(cs47l63_t *driver)
{
    uint32_t ret;
    uint32_t temp_reg_val;
    uint32_t old_reg = 0;
    uint32_t new_reg;

    driver->event_flags = 0;

    for (uint32_t i = 0; i < N_IRQ_REGS; i++)
    {
        new_reg = CS47L63_IRQ1_EINT_1 + cs47l63_event_data[i].irq_reg_offset;
        if (old_reg != new_reg)
        {
            ret = cs47l63_read_reg(driver, new_reg, &temp_reg_val);
            if (ret != CS47L63_STATUS_OK)
            {
                return CS47L63_STATUS_FAIL;
            }
        }
        old_reg = new_reg;

        if (temp_reg_val & cs47l63_event_data[i].mask)
        {
            driver->event_flags |= cs47l63_event_data[i].event_flag;
            ret = cs47l63_write_reg(driver,
                                    CS47L63_IRQ1_EINT_1 + cs47l63_event_data[i].irq_reg_offset,
                                    cs47l63_event_data[i].mask);
        }
    }

    return CS47L63_STATUS_OK;
}

/***********************************************************************************************************************
 * API FUNCTIONS
 **********************************************************************************************************************/

/**
 * Initialize driver state/handle
 *
 */
uint32_t cs47l63_initialize(cs47l63_t *driver)
{
    uint32_t ret = CS47L63_STATUS_FAIL;

    if (NULL != driver)
    {
        /*
         * The memset() call sets all members to 0, including the following semantics:
         * - 'state' is set to UNCONFIGURED
         */
        memset(driver, 0, sizeof(cs47l63_t));

        ret = CS47L63_STATUS_OK;
    }

    return ret;
}

/**
 * Configures driver state/handle
 *
 */
uint32_t cs47l63_configure(cs47l63_t *driver, cs47l63_config_t *config)
{
    uint32_t ret = CS47L63_STATUS_FAIL;

    if ((NULL != driver) && \
        (NULL != config))
    {
        driver->config = *config;

        ret = bsp_driver_if_g->register_gpio_cb(driver->config.bsp_config.bsp_int_gpio_id,
                                                &cs47l63_irq_callback,
                                                driver);

        if (ret == BSP_STATUS_FAIL)
        {
            return ret;
        }

        // Configure DSP Core 1
        driver->dsp_info[0].dsp_core = 1;
        driver->dsp_info[0].base_addr = CS47L63_DSP_BASE_ADDR;
        driver->dsp_info[0].ram_banks = cs47l63_dsp1_ram_banks;
        driver->dsp_info[0].n_ram_banks = N_DSP1_RAM_BANKS;

        // Advance driver to CONFIGURED state
        driver->state = CS47L63_STATE_CONFIGURED;
    }

    return ret;
}

/**
 * Processes driver events and notifications
 *
 */
uint32_t cs47l63_process(cs47l63_t *driver)
{
    // check for driver state
    if ((driver->state != CS47L63_STATE_UNCONFIGURED) && (driver->state != CS47L63_STATE_ERROR))
    {
        // check for driver mode
        if (driver->mode == CS47L63_MODE_HANDLING_EVENTS)
        {
            // Check for valid state to process events
            if ((driver->state == CS47L63_STATE_STANDBY))
            {
                // run through event handler
                if (CS47L63_STATUS_OK == cs47l63_event_handler(driver))
                {
                    driver->mode = CS47L63_MODE_HANDLING_CONTROLS;
                }
                else
                {
                    return CS47L63_STATUS_FAIL;
                }
            }
            // If in invalid state for handling events (i.e. BHM, Calibration), simply switch back to Handling Controls
            else
            {
                driver->mode = CS47L63_MODE_HANDLING_CONTROLS;
            }
        }

        if (driver->event_flags)
        {
            if (driver->config.bsp_config.notification_cb != NULL)
            {
                driver->config.bsp_config.notification_cb(driver->event_flags,
                                                          driver->config.bsp_config.notification_cb_arg);
            }

            driver->event_flags = 0;
        }
    }

    return CS47L63_STATUS_OK;
}

/**
 * Reset the CS47L63
 *
 */
uint32_t cs47l63_reset(cs47l63_t *driver)
{
    uint32_t temp_reg_val;
    uint32_t ret;
    uint32_t iter_timeout = 0;

    // Drive RESET low
    bsp_driver_if_g->set_gpio(driver->config.bsp_config.bsp_reset_gpio_id, BSP_GPIO_LOW);
    bsp_driver_if_g->set_timer(2, NULL, NULL);

    // Enable DCVDD with RESET low
    bsp_driver_if_g->set_supply(driver->config.bsp_config.bsp_dcvdd_supply_id, BSP_SUPPLY_ENABLE);
    bsp_driver_if_g->set_timer(2, NULL, NULL);

    // Drive RESET high
    bsp_driver_if_g->set_gpio(driver->config.bsp_config.bsp_reset_gpio_id, BSP_GPIO_HIGH);

    // Wait for boot sequence to finish
    do
    {
        // Delay to allow boot before checking BOOT_DONE_EINT1
        bsp_driver_if_g->set_timer(10, NULL, NULL);

        // Read BOOT_DONE_EINT1
        ret = cs47l63_read_reg(driver, CS47L63_IRQ1_EINT_2, &temp_reg_val);
        if (ret != CS47L63_STATUS_OK)
        {
            return CS47L63_STATUS_FAIL;
        }
        iter_timeout++;
        if (iter_timeout > 20)
        {
            return CS47L63_STATUS_FAIL;
        }
    } while ((temp_reg_val & CS47L63_BOOT_DONE_EINT1_MASK) == 0);

    // Read device ID and revision ID
    ret = cs47l63_read_reg(driver, CS47L63_DEVID, &temp_reg_val);
    if (ret != CS47L63_STATUS_OK)
    {
        return CS47L63_STATUS_FAIL;
    }
    driver->devid = temp_reg_val;

    ret = cs47l63_read_reg(driver, CS47L63_REVID, &temp_reg_val);
    if (ret != CS47L63_STATUS_OK)
    {
        return CS47L63_STATUS_FAIL;
    }
    driver->revid = temp_reg_val;

    // Write configuration data
    // Since cs47l63 is configured via WISCE scripts, can write without masking individual bits
    for (uint32_t count = 0; count < driver->config.syscfg_regs_total; count++)
    {
        ret = cs47l63_write_reg(driver, driver->config.syscfg_regs[count].address, driver->config.syscfg_regs[count].value);
        if (ret != CS47L63_STATUS_OK)
        {
            return CS47L63_STATUS_FAIL;
        }
    }

    // Unmask interrupts
    // Omit first mask register, as BOOT_DONE_EINT1 is enabled by default
    for (uint32_t index = 1; index < N_IRQ_REGS; ++index)
    {
        ret = cs47l63_update_reg(driver,
                                 CS47L63_IRQ1_MASK_1 + cs47l63_event_data[index].irq_reg_offset,
                                 cs47l63_event_data[index].mask, 0);
        if (ret != CS47L63_STATUS_OK)
        {
            return CS47L63_STATUS_FAIL;
        }
    }

    driver->state = CS47L63_STATE_STANDBY;

    return CS47L63_STATUS_OK;
}

/**
 * Write block of data to the CS47L63 register file
 *
 */
uint32_t cs47l63_write_block(cs47l63_t *driver, uint32_t addr, uint8_t *data, uint32_t length)
{
    if ((data == NULL) || (length == 0) || (length % 4 != 0))
    {
        return CS47L63_STATUS_FAIL;
    }

    return cs47l63_cp_bulk_write_block(driver, addr, data, length);
}

/**
 * Read block of data from the CS47L63
 *
 */
uint32_t cs47l63_read_block(cs47l63_t *driver, uint32_t addr, uint8_t *data, uint32_t length)
{
    if ((data == NULL) || (length == 0) || (length % 4 != 0))
    {
        return CS47L63_STATUS_FAIL;
    }

    return cs47l63_cp_bulk_read_block(driver, addr, length, data);
}

/**
 * Wait for the provided number of miliseconds.
 *
 */
uint32_t cs47l63_wait(uint32_t time_in_ms)
{
    uint32_t bsp_status, ret = CS47L63_STATUS_OK;

    bsp_status = bsp_driver_if_g->set_timer(time_in_ms, NULL, NULL);
    if (bsp_status == BSP_STATUS_FAIL)
    {
        ret = CS47L63_STATUS_FAIL;
    }

    return ret;
}

/**
 * Finish booting the CS47L63
 *
 */
uint32_t cs47l63_boot(cs47l63_t *driver, uint32_t dsp_core, fw_img_info_t *fw_info)
{
    cs47l63_dsp_t *dsp_info;

    if (dsp_core > CS47L63_NUM_DSP || dsp_core == 0)
    {
        return CS47L63_STATUS_FAIL;
    }

    dsp_info = &driver->dsp_info[dsp_core - 1];
    dsp_info->fw_info = fw_info;

    return CS47L63_STATUS_OK;
}

/**
 * Change the power state
 *
 */
uint32_t cs47l63_power(cs47l63_t *driver, uint32_t dsp_core, uint32_t power_state)
{
    uint32_t ret = CS47L63_STATUS_OK;
    cs47l63_dsp_t *dsp_info;

    if (dsp_core > CS47L63_NUM_DSP || dsp_core == 0)
    {
        return CS47L63_STATUS_FAIL;
    }

    dsp_info = &driver->dsp_info[dsp_core - 1];

    switch (power_state)
    {
        case CS47L63_POWER_MEM_ENA:
            ret = cs47l63_power_mem_ena(driver, dsp_info);
            break;

        case CS47L63_POWER_UP:
            ret = cs47l63_power_up(driver, dsp_info);
            break;

        case CS47L63_POWER_DOWN:
            ret = cs47l63_power_down(driver, dsp_info);
            break;

        case CS47L63_POWER_MEM_DIS:
            ret = cs47l63_power_mem_dis(driver, dsp_info);
            break;

        default:
            ret = CS47L63_STATUS_FAIL;
            break;
    }

    return ret;
}

/*!
 * \mainpage Introduction
 *
 * This document outlines the driver source code included in the MCU Driver Software Package for the CS47L63 Driver.
 * This guide is primarily intended for those involved in end-system implementation, integration, and testing, who
 * will use the CS47L63 MCU Driver Software Package to integrate the CS47L63 driver source code into the end-system's
 * host MCU software.  After reviewing this guide, the reader will be able to begin software integration
 * of the CS47L63 MCU driver and then have the ability to initialize, reset, boot, configure, and service events from
 * the CS47L63.  This guide should be used along with the CS47L63 Datasheet.
 *
 *  In order to obtain any additional materials, and for any questions regarding this guide, the MCU Driver
 *  Software Package, or CS47L63 system integration, please contact your Cirrus Logic Representative.
 */
