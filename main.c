/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the RutDevKit-PSoC62_QSPI_PSRAM_XIP
*              Application for ModusToolbox.
*
* Related Document: See README.md
*
*
*  Created on: 2022-10-28
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Address: Jonavos g. 30, Kaunas 44262, Lithuania
*  Author: GDR
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*
* Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cycfg_qspi_memslot.h"

/***************************************************************************
* Global constants
***************************************************************************/
#define SMIF_PRIORITY           (1u)     /* SMIF interrupt priority */
#define PACKET_SIZE             (1024u)  /* The memory Read/Write packet */
#define NUM_BYTES_PER_LINE		(16u)	 /* Used when array of data is printed on the console */
#define TIMEOUT_1_MS            (1000ul) /* 1 ms timeout for all blocking functions */
#define QSPI_FREQ            	(80000000ul)
#define USER_DATA               (0xA0)   /* User data for memory write and read */

/*LED indication control*/
#define LED_IND_OFF          (0x03)
#define LED_IND_RED          (0x01)
#define LED_IND_GREEN        (0x02)

/***************************************************************************
* Global variables and functions
***************************************************************************/
void handle_error(void);
void Isr_SMIF(void);
void Init_SMIF(void);
void status_led (uint32_t status);

/*QSPI PSRAM object*/
cyhal_qspi_t qspi_psram_obj;

/* SMIF configuration parameters */
cy_stc_smif_config_t SMIFConfig =
{
    /* .mode           */ CY_SMIF_NORMAL,      /* Mode of operation */
    /* .deselectDelay  */ 2U,      			/* Minimum duration of SPI deselection */
    /* .rxClockSel     */ CY_SMIF_SEL_INVERTED_INTERNAL_CLK,     /* Clock source for the receiver clock */
    /* .blockEvent     */ CY_SMIF_BUS_ERROR    /* What happens when there is a read
                                                * to an empty RX FIFO or write to a full TX FIFO
                                                */
};

/*QSPI pins configuration structure*/
cyhal_qspi_slave_pin_config_t qspi_pins =
{
		.io[0] = QSPI_IO0,
		.io[1] = QSPI_IO1,
		.io[2] = QSPI_IO2,
		.io[3] = QSPI_IO3,
		.io[4] = NC,
		.io[5] = NC,
		.io[6] = NC,
		.io[7] = NC,
		.ssel = PSRAM_SSEL
};


int main(void)
{
    cy_rslt_t result;
    cy_en_smif_status_t result_smif;
    uint8_t fram_buffer[PACKET_SIZE];    /* Buffer to read memory in burst mode */
    uint8_t pttrn=USER_DATA;
    uint32_t index;
    uint8_t  volatile *Xip_Buffer_8;     /* Xip  buffer for 8-bit data */

    /* Initilaize the Xip buffer for memory mapped accesses */
    Xip_Buffer_8 =(uint8_t *)APS6404L_3SQR_ZR_SlaveSlot_0.baseAddress; /* Assign the SMIF base address to Xip buffer*/

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    /*Enable debug output via KitProg UART*/
    result = cy_retarget_io_init( KITPROG_TX, KITPROG_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }
    printf("\x1b[2J\x1b[;H");
    printf("RDK2 QSPI PSRAM XIP Example.\r\n");

    /*Initialize LEDs*/
    result = cyhal_gpio_init( LED1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    result = cyhal_gpio_init( LED2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

    /*Initialize QSPI PSRAM Interface*/
    result = cyhal_qspi_init(&qspi_psram_obj, QSPI_CLK, &qspi_pins, QSPI_FREQ, 0, NULL);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

    /*Initializes SMIF block*/
    Init_SMIF();

	/*Indicate the write operation*/
	status_led (LED_IND_GREEN);

	/*Write the data to the PSRAM*/
	memset(fram_buffer, pttrn, PACKET_SIZE);
	result_smif = Cy_SMIF_MemWrite( qspi_psram_obj.base, &APS6404L_3SQR_ZR_SlaveSlot_0, 0x00, fram_buffer, PACKET_SIZE, &qspi_psram_obj.context );
    if (result_smif != CY_SMIF_SUCCESS)
    {
    	printf("Write error. Address: 0x%08X\r\n", (unsigned int)(APS6404L_3SQR_ZR_SlaveSlot_0.baseAddress));
    }
    printf("All the data has been written to the PSRAM memory.\r\n");

	/*Indicate the write operation*/
	status_led (LED_IND_OFF);

    /*Read the data from the PSRAM*/
    memset(fram_buffer, 0x00, PACKET_SIZE);
    result_smif = Cy_SMIF_MemRead( qspi_psram_obj.base, &APS6404L_3SQR_ZR_SlaveSlot_0, 0x0000, fram_buffer, PACKET_SIZE, &qspi_psram_obj.context );
    if (result_smif != CY_SMIF_SUCCESS)
    {
    	printf("Read error. Address: 0x%08X \r\n", (unsigned int)(APS6404L_3SQR_ZR_SlaveSlot_0.baseAddress));
    }
    printf("All the data has been read from the PSRAM memory.\r\n");

    /*Check the data consistency*/
    for(index=0; index < PACKET_SIZE; index++)
    {
    	if( fram_buffer[index] != pttrn)
    	{
    		printf("Read error. Address: 0x%08X\r\n", (unsigned int)&fram_buffer[index]);
    		handle_error();
    	}
    }
    printf("Memory consistency check: PASS.\r\n");

    /*Enter XIP mode*/
    printf("Set the SMIF memory mapped (XIP) mode... \r\n");
    Cy_SMIF_SetMode(qspi_psram_obj.base, CY_SMIF_MEMORY);
    printf("In memory mapped (XIP) mode from now.\r\n");

    for (;;)
    {
    	/*Indicate the write operation*/
    	status_led (LED_IND_GREEN);

    	/*Print the pattern to be written*/
    	printf("Writing whole PSRAM with the data: 0x%02X\r\n", pttrn);
    	for(index = 0; index < APS6404L_3SQR_ZR_SlaveSlot_0.deviceCfg->memSize; index++)
    	{
    		Xip_Buffer_8[index] = pttrn;
    	}

    	/*Indicate the write operation*/
    	status_led (LED_IND_OFF);

    	/*Read the memory, compare the pattern*/
    	printf("Reading and comparing whole PSRAM data...\r\n");
    	for(index = 0; index < APS6404L_3SQR_ZR_SlaveSlot_0.deviceCfg->memSize; index++)
    	{
    		if(Xip_Buffer_8[index] != pttrn)
    		{
    			printf("Read error. Address: 0x%08X\r\n", (unsigned int)&Xip_Buffer_8[index]);
    		}
    	}

    	pttrn++;
    }
}

/*******************************************************************************
* Function Name: Isr_SMIF
********************************************************************************
*
* The ISR for the SMIF interrupt. All Read/Write transfers to/from the external
* memory are processed inside the SMIF ISR.
*
*******************************************************************************/
void Isr_SMIF(void)
{
    Cy_SMIF_Interrupt(qspi_psram_obj.base, &qspi_psram_obj.context);
}

/*******************************************************************************
* Function Name: Initialize_SMIF
********************************************************************************
*
* This function initializes the SMIF block
*
*******************************************************************************/
void Init_SMIF(void)
{
	/* Initialize SMIF interrupt */

	      cy_stc_sysint_t smifIntConfig =
		  {
		   .intrSrc = 2u,
		   .intrPriority = SMIF_PRIORITY
		  };

	      cy_en_sysint_status_t intrStatus = Cy_SysInt_Init(&smifIntConfig, Isr_SMIF);

	      if(0u != intrStatus)
		    {
		    	printf("================================================================================\r\n");
		        printf("SMIF interrupt initialization failed.\r\n");
		        printf("Error Code: 0x%08uX\r\n", (unsigned int) intrStatus);
		        printf("================================================================================\r\n");
		        status_led (LED_IND_RED);

		        for(;;)
		          {
		             /*Waits forever when SMIF initialization error occurs*/
		          }
		    }

		  printf("SMIF interrupt is initialized.\r\n");

		  /* Initialize SMIF */
		  cy_en_smif_status_t smifStatus;
		  smifStatus = Cy_SMIF_Init(qspi_psram_obj.base, &SMIFConfig, TIMEOUT_1_MS, &qspi_psram_obj.context);

		  if(0u != smifStatus)
		    {
		    	printf("================================================================================\r\n");
		        printf("SMIF interrupt is initialized.\r\n");
		        printf("Error Code: 0x%08uX\r\n", (unsigned int) smifStatus);
		        printf("================================================================================\r\n");
		        status_led (LED_IND_RED);

		   for(;;)
		         {
		             /*Waits forever when SMIF initialization error occurs*/
		         }
		     }
	     /* Configure slave select and data select. These settings depend on the pins
	      * selected in the Device and QSPI configurators.
	      */
		  Cy_SMIF_SetDataSelect(qspi_psram_obj.base, qspi_psram_obj.slave_select, APS6404L_3SQR_ZR_SlaveSlot_0.dataSelect);
	       Cy_SMIF_Enable(qspi_psram_obj.base, &qspi_psram_obj.context);
	       Cy_SMIF_Memslot_Init(qspi_psram_obj.base, (cy_stc_smif_block_config_t *)&smifBlockConfig, &qspi_psram_obj.context);
	       Cy_SMIF_SetMode(qspi_psram_obj.base, CY_SMIF_NORMAL);
	       NVIC_EnableIRQ(smifIntConfig.intrSrc); /* Enable the SMIF interrupt */

		   printf("SMIF hardware block is initialized.\r\n");
}

/*******************************************************************************
* Function Name: status_led
********************************************************************************
*
* This function drives the red/green status of RGB
* *
* \param status - The led status (RGB_GLOW_OFF, RGB_GLOW_RED, or RGB_GLOW_GREEN)
*
*******************************************************************************/
void status_led (uint32_t status)
{
	if (status == LED_IND_RED)
	{
		cyhal_gpio_write(LED2, CYBSP_LED_STATE_ON);
		cyhal_gpio_write(LED1, CYBSP_LED_STATE_OFF);
	}
	else if (status == LED_IND_GREEN)
	{
		cyhal_gpio_write(LED2, CYBSP_LED_STATE_OFF);
		cyhal_gpio_write(LED1, CYBSP_LED_STATE_ON);
	}
	else
	{
		cyhal_gpio_write(LED2, CYBSP_LED_STATE_OFF);
		cyhal_gpio_write(LED1, CYBSP_LED_STATE_OFF);
	}
}

void handle_error(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}

/* [] END OF FILE */
