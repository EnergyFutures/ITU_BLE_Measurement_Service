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

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "twi_master_int.h"
#include "ble.h"
#include "nrf_soc.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"

volatile uint8_t tx_bytes_to_send, rx_bytes_to_receive;
volatile uint8_t *tx_data_ptr, *rx_data_ptr;
static twi_config_t* active_config;


static void twi_interrupt_internal(NRF_TWI_Type * twi_master)
{
    if(twi_master->EVENTS_TXDSENT)
    {
        twi_master->EVENTS_TXDSENT = 0;
        if(tx_bytes_to_send)
        {
            twi_master->TXD = *tx_data_ptr++;  
            tx_bytes_to_send--;
        }
        else 
        {
            if(rx_bytes_to_receive == 0)
            {
                twi_master->TASKS_STOP = 1; 
            }
            else
            {
                sd_ppi_channel_enable_set(1 << active_config->twi_ppi_ch);

                twi_master->TASKS_STARTRX = 1;
            }
        }
    }
    if(twi_master->EVENTS_STOPPED)
    {
        twi_master->EVENTS_STOPPED = 0;
        active_config->twi_operation_complete = true;
        active_config->twi_ack_received = true;
    }
    if(twi_master->EVENTS_RXDREADY)
    {
        twi_master->EVENTS_RXDREADY = 0;
        *rx_data_ptr++ = twi_master->RXD;
        if (--rx_bytes_to_receive == 1)
        {
            sd_ppi_channel_assign(active_config->twi_ppi_ch, &twi_master->EVENTS_BB, &twi_master->TASKS_STOP);  
        }

        if(rx_bytes_to_receive > 0)
        {
            twi_master->TASKS_RESUME = 1;
        }
    }
    if(twi_master->EVENTS_ERROR)
    {
        twi_master->EVENTS_ERROR = 0;
        active_config->twi_operation_complete = true;
        active_config->twi_ack_received = false;
    }
}

void SPI0_TWI0_IRQHandler()
{
	 twi_interrupt_internal(NRF_TWI0);
}

void SPI1_TWI1_IRQHandler()
{
	 twi_interrupt_internal(NRF_TWI1);
}

bool twi_master_write(uint8_t address, uint8_t *tx_data, uint8_t tx_data_length,twi_config_t *cfg)
{
    if (tx_data_length == 0)
    {    
        return false;
    }
		active_config =cfg;

		sd_ppi_channel_enable_clr(1 << cfg->twi_ppi_ch);

    
    cfg->twi->ADDRESS = address;
    tx_data_ptr = tx_data;
    tx_bytes_to_send = tx_data_length - 1;
    rx_bytes_to_receive = 0;
    cfg->twi->TXD = *tx_data_ptr++;
    cfg->twi->TASKS_STARTTX = 1;
    cfg->twi_operation_complete = false;
    
    while(cfg->twi_operation_complete == false);
		active_config =0;
    return cfg->twi_ack_received;

}

bool twi_master_write_read(uint8_t address, uint8_t *tx_data, uint8_t tx_data_length, uint8_t *rx_data, uint8_t rx_data_length,twi_config_t *cfg)
{
    if (tx_data_length == 0 || rx_data_length == 0)
    {    
        return false;
    }
		active_config =cfg;

    cfg->twi->ADDRESS = address;
    tx_data_ptr = tx_data;
    tx_bytes_to_send = tx_data_length - 1;
    rx_data_ptr = rx_data;
    rx_bytes_to_receive = rx_data_length;
       
    if (rx_bytes_to_receive == 1)
    {
        sd_ppi_channel_assign(cfg->twi_ppi_ch, &cfg->twi->EVENTS_BB, &cfg->twi->TASKS_STOP);
    }
    else
    {
        sd_ppi_channel_assign(cfg->twi_ppi_ch, &cfg->twi->EVENTS_BB, &cfg->twi->TASKS_SUSPEND);
    }
    sd_ppi_channel_enable_clr(1 << cfg->twi_ppi_ch);        
    
    cfg->twi->TXD = *tx_data_ptr++;
    cfg->twi->TASKS_STARTTX = 1;
    cfg->twi_operation_complete = false;
		while(cfg->twi_operation_complete == false);
		active_config =0;
    return cfg->twi_ack_received;
}

bool twi_master_read(uint8_t address, uint8_t *rx_data, uint8_t rx_data_length,twi_config_t *cfg)
{
    if(rx_data_length == 0)
    {
        return false;
    }
		active_config =cfg;

    cfg->twi->ADDRESS = address;
    rx_data_ptr = rx_data;
    rx_bytes_to_receive = rx_data_length;
    
    if (rx_bytes_to_receive == 1)
    {
        sd_ppi_channel_assign(cfg->twi_ppi_ch, &cfg->twi->EVENTS_BB, &cfg->twi->TASKS_STOP);
    }
    else
    {
        sd_ppi_channel_assign(cfg->twi_ppi_ch, &cfg->twi->EVENTS_BB, &cfg->twi->TASKS_SUSPEND);
    }
    sd_ppi_channel_enable_set(1 << cfg->twi_ppi_ch);
    
    cfg->twi->TASKS_STARTRX = 1;
    cfg->twi_operation_complete = false;

    while(cfg->twi_operation_complete == false);
		active_config =0;
    return cfg->twi_ack_received;
}

/**
 * Detects stuck slaves (SDA = 0 and SCL = 1) and tries to clear the bus.
 *
 * @return
 * @retval false Bus is stuck.
 * @retval true Bus is clear.
 */
static bool twi_master_clear_bus(twi_config_t *cfg)
{
    bool bus_clear;

    TWI_SDA_HIGH();
    TWI_SCL_HIGH();
    TWI_DELAY();

    if (TWI_SDA_READ() == 1 && TWI_SCL_READ() == 1)
    {
        bus_clear = true;
    }
    else
    {
        uint_fast8_t i;
        bus_clear = false;

        // Clock max 18 pulses worst case scenario(9 for master to send the rest of command and 9 for slave to respond) to SCL line and wait for SDA come high
        for (i=18; i--;)
        {
            TWI_SCL_LOW();
            TWI_DELAY();
            TWI_SCL_HIGH();
            TWI_DELAY();

            if (TWI_SDA_READ() == 1)
            {
                bus_clear = true;
                break;
            }
        }
    }
    return bus_clear;
}

bool twi_master_init(twi_config_t *cfg)
{
    cfg->twi_operation_complete = true;
    cfg->twi_ack_received = true;

          
    /* To secure correct signal levels on the pins used by the TWI
       master when the system is in OFF mode, and when the TWI master is 
       disabled, these pins must be configured in the GPIO peripheral.
    */ 
    NRF_GPIO->PIN_CNF[cfg->twi_pinselect_scl] = 
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
      | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos)
      | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)
      | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
      | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);    

    NRF_GPIO->PIN_CNF[cfg->twi_pinselect_sda] = 
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
      | (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos)
      | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)
      | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
      | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos); 
    
    cfg->twi->EVENTS_RXDREADY = 0;
    cfg->twi->EVENTS_TXDSENT = 0;
    cfg->twi->PSELSCL = cfg->twi_pinselect_scl;
    cfg->twi->PSELSDA = cfg->twi_pinselect_sda;
    
    switch(cfg->frequency)
    {
        case TWI_FREQ_100KHZ:
            cfg->twi->FREQUENCY = TWI_FREQUENCY_FREQUENCY_K100 << TWI_FREQUENCY_FREQUENCY_Pos;
            break;
        case TWI_FREQ_400KHZ:
            cfg->twi->FREQUENCY = TWI_FREQUENCY_FREQUENCY_K400 << TWI_FREQUENCY_FREQUENCY_Pos;
            break;
    }
    sd_ppi_channel_assign(cfg->twi_ppi_ch, &cfg->twi->EVENTS_BB, &cfg->twi->TASKS_SUSPEND);
    sd_ppi_channel_enable_clr(1 << cfg->twi_ppi_ch);
    sd_nvic_SetPriority(cfg->twi_interrupt_no, TWI_IRQ_PRIORITY_SD);
    sd_nvic_EnableIRQ(cfg->twi_interrupt_no);
    cfg->twi->INTENSET = TWI_INTENSET_TXDSENT_Msk | TWI_INTENSET_STOPPED_Msk | TWI_INTENSET_ERROR_Msk | TWI_INTENSET_RXDREADY_Msk;
    cfg->twi->ENABLE = TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos;
    return twi_master_clear_bus(cfg);
}
