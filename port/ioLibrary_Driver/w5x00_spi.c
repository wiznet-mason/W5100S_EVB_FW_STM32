/**
 * Copyright (c) 2021 WIZnet Co.,Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
  * ----------------------------------------------------------------------------------------------------
  * Includes
  * ----------------------------------------------------------------------------------------------------
  */
#include <stdio.h>

#include "port_common.h"

#include "wizchip_conf.h"
#include "w5x00_spi.h"

/**
  * ----------------------------------------------------------------------------------------------------
  * Variables
  * ----------------------------------------------------------------------------------------------------
  */

extern SPI_HandleTypeDef hspi1;

#ifdef USE_SPI_DMA
static uint dma_tx;
static uint dma_rx;
static dma_channel_config dma_channel_config_tx;
static dma_channel_config dma_channel_config_rx;
#endif

/**
  * ----------------------------------------------------------------------------------------------------
  * Functions
  * ----------------------------------------------------------------------------------------------------
  */
static inline void wizchip_select(void)
{
    HAL_GPIO_WritePin(W5100S_SPI1_CS_GPIO_Port, W5100S_SPI1_CS_Pin, GPIO_PIN_RESET);
}

static inline void wizchip_deselect(void)
{
    HAL_GPIO_WritePin(W5100S_SPI1_CS_GPIO_Port, W5100S_SPI1_CS_Pin, GPIO_PIN_SET);
}

void wizchip_reset()
{

}

static uint8_t wizchip_read(void)
{
    uint8_t rx_data = 0;
    uint8_t tx_data = 0xFF;

		while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
    HAL_SPI_TransmitReceive(&hspi1, &tx_data, &rx_data, 1, 10);

    return rx_data;
}

static void wizchip_write(uint8_t tx_data)
{
		uint8_t rtnByte;

    while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
    HAL_SPI_TransmitReceive(&hspi1, &tx_data, &rtnByte, 1, 10);

}

#ifdef USE_SPI_DMA
static void wizchip_read_burst(uint8_t *pBuf, uint16_t len)
{

}

static void wizchip_write_burst(uint8_t *pBuf, uint16_t len)
{

}
#endif

static void wizchip_critical_section_lock(void)
{
    __disable_irq();
}

static void wizchip_critical_section_unlock(void)
{
    __enable_irq();
}

void wizchip_spi_initialize(void)
{

}

void wizchip_cris_initialize(void)
{
	reg_wizchip_cris_cbfunc(wizchip_critical_section_lock, wizchip_critical_section_unlock);
}

void wizchip_initialize(void)
{
    /* Deselect the FLASH : chip select high */
    wizchip_deselect();

    /* CS function register */
    reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);

    /* SPI function register */
    reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write);
#ifdef USE_SPI_DMA
    reg_wizchip_spiburst_cbfunc(wizchip_read_burst, wizchip_write_burst);
#endif

    /* W5x00 initialize */
    uint8_t temp;
    uint8_t memsize[2][4] = {{2, 2, 2, 2}, {2, 2, 2, 2}};

    if (ctlwizchip(CW_INIT_WIZCHIP, (void *)memsize) == -1)
    {
        printf(" W5x00 initialized fail\n");

        return;
    }

    /* Check PHY link status */
    do
    {
        if (ctlwizchip(CW_GET_PHYLINK, (void *)&temp) == -1)
        {
            printf(" Unknown PHY link status\n");

            return;
        }
    } while (temp == PHY_LINK_OFF);
}

void wizchip_check(void)
{
    /* Read version register */
    if (getVER() != 0x51) // W5100S
    {
        printf(" ACCESS ERR : VERSIONR != 0x51, read value = 0x%02x\n", getVER());

        while (1)
            ;
    }
}

/* Network */
void network_initialize(wiz_NetInfo net_info)
{
    ctlnetwork(CN_SET_NETINFO, (void *)&net_info);
}

void print_network_information(wiz_NetInfo net_info)
{
    uint8_t tmp_str[8] = {
        0,
    };

    ctlnetwork(CN_GET_NETINFO, (void *)&net_info);
    ctlwizchip(CW_GET_ID, (void *)tmp_str);

    if (net_info.dhcp == NETINFO_DHCP)
    {
        printf("====================================================================================================\n");
        printf(" %s network configuration : DHCP\n\n", (char *)tmp_str);
    }
    else
    {
        printf("====================================================================================================\n");
        printf(" %s network configuration : static\n\n", (char *)tmp_str);
    }

    printf(" MAC         : %02X:%02X:%02X:%02X:%02X:%02X\n", net_info.mac[0], net_info.mac[1], net_info.mac[2], net_info.mac[3], net_info.mac[4], net_info.mac[5]);
    printf(" IP          : %d.%d.%d.%d\n", net_info.ip[0], net_info.ip[1], net_info.ip[2], net_info.ip[3]);
    printf(" Subnet Mask : %d.%d.%d.%d\n", net_info.sn[0], net_info.sn[1], net_info.sn[2], net_info.sn[3]);
    printf(" Gateway     : %d.%d.%d.%d\n", net_info.gw[0], net_info.gw[1], net_info.gw[2], net_info.gw[3]);
    printf(" DNS         : %d.%d.%d.%d\n", net_info.dns[0], net_info.dns[1], net_info.dns[2], net_info.dns[3]);
    printf("====================================================================================================\n\n");
}
