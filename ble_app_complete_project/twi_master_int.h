 /* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#ifndef TWI_MASTER_H__
#define TWI_MASTER_H__

/*lint ++flb "Enter library region" */

#include <stdbool.h>
#include <stdint.h>
#include <nrf51.h>

// Set the interrupt priority, different values for use with or without a SoftDevice
#define TWI_IRQ_PRIORITY_SD     3   // NRF_APP_PRIORITY_HIGH = 1, NRF_APP_PRIORITY_LOW = 3, only these values are accepted

enum {TWI_FREQ_100KHZ = 0, TWI_FREQ_400KHZ = 1};

typedef struct
{
  uint8_t twi_ppi_ch;  
	NRF_TWI_Type *twi;
	bool twi_operation_complete;
	bool twi_ack_received;
}twi_config_t;

typedef struct
{
  uint8_t twi_pinselect_scl;
  uint8_t twi_pinselect_sda;
	IRQn_Type twi_interrupt_no;
	uint8_t frequency     : 1;
}twi_init_config_t;

/** @file
* @brief Software controlled TWI Master driver.
*
*
* @defgroup lib_driver_twi_master Software controlled TWI Master driver
* @{
* @ingroup nrf_drivers
* @brief Software controlled TWI Master driver.
*
* Supported features:
* - Repeated start
* - No multi-master
* - Only 7-bit addressing
* - Supports clock stretching (with optional SMBus style slave timeout)
* - Tries to handle slaves stuck in the middle of transfer
*/

#define TWI_READ_BIT                 (0x01)        //!< If this bit is set in the address field, transfer direction is from slave to master.

#define TWI_ISSUE_STOP               ((bool)true)  //!< Parameter for @ref twi_master_transfer
#define TWI_DONT_ISSUE_STOP          ((bool)false) //!< Parameter for @ref twi_master_transfer

/* These macros are needed to see if the slave is stuck and we as master send dummy clock cycles to end its wait */
/*lint -e717 -save "Suppress do {} while (0) for these macros" */
/*lint ++flb "Enter library region" */
#define TWI_SCL_HIGH()   do { NRF_GPIO->OUTSET = (1UL << cfg->twi_pinselect_scl); } while(0)   /*!< Pulls SCL line high */
#define TWI_SCL_LOW()    do { NRF_GPIO->OUTCLR = (1UL << cfg->twi_pinselect_scl); } while(0)   /*!< Pulls SCL line low  */
#define TWI_SDA_HIGH()   do { NRF_GPIO->OUTSET = (1UL << cfg->twi_pinselect_sda);  } while(0)   /*!< Pulls SDA line high */
#define TWI_SDA_LOW()    do { NRF_GPIO->OUTCLR = (1UL << cfg->twi_pinselect_sda);  } while(0)   /*!< Pulls SDA line low  */
#define TWI_SDA_INPUT()  do { NRF_GPIO->DIRCLR = (1UL << cfg->twi_pinselect_sda);  } while(0)   /*!< Configures SDA pin as input  */
#define TWI_SDA_OUTPUT() do { NRF_GPIO->DIRSET = (1UL << cfg->twi_pinselect_sda);  } while(0)   /*!< Configures SDA pin as output */
#define TWI_SCL_OUTPUT() do { NRF_GPIO->DIRSET = (1UL << cfg->twi_pinselect_scl); } while(0)   /*!< Configures SCL pin as output */
/*lint -restore */

#define TWI_SDA_READ() ((NRF_GPIO->IN >> cfg->twi_pinselect_sda) & 0x1UL)                     /*!< Reads current state of SDA */
#define TWI_SCL_READ() ((NRF_GPIO->IN >> cfg->twi_pinselect_scl) & 0x1UL)                    /*!< Reads current state of SCL */

#define TWI_DELAY() nrf_delay_us(4) /*!< Time to wait when pin states are changed. For fast-mode the delay can be zero and for standard-mode 4 us delay is sufficient. */


/**
 * Initializes TWI bus IO pins and checks the bus is operational.
 *
 * Both pins are configured as Standard-0, No-drive-1 (open drain).
 *
 * @return
 * @retval true TWI bus is clear for transfers.
 * @retval false TWI bus is stuck.
 */
bool twi_master_init(twi_init_config_t *init_cfg,twi_config_t *cfg);

bool twi_master_write(uint8_t address, uint8_t *data, uint8_t data_length,twi_config_t *cfg);

bool twi_master_read(uint8_t address, uint8_t *data, uint8_t data_length,twi_config_t *cfg);

bool twi_master_write_read(uint8_t address, uint8_t *tx_data, uint8_t tx_data_length, uint8_t *rx_data, uint8_t rx_data_length,twi_config_t *cfg);

/*lint --flb "Leave library region" */
#endif
