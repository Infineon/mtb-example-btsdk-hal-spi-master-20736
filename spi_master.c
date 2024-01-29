/*
 * Copyright 2016-2024, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
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
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/******************************************************
 * This application demonstrates initializing an SPI
 * interface to communicate with a peer device as a SPI
 * master.
 ******************************************************/

/** @file */

#include "spar_utils.h"
#include "bleprofile.h"
#include "bleapp.h"
#include "gpiodriver.h"
#include "string.h"
#include "stdio.h"
#include "platform.h"
#include "spiffydriver.h"
#include "bleappconfig.h"
#include "devicelpm.h"
#include "hidddriversconfig.h"
#include "sparcommon.h"

/******************************************************
 *                      Constants
 ******************************************************/

// Comment out following definition to have Client run transmissions
#define SPI_MASTER_TRANSMIT_ON_TIMEOUT

// Use 1M speed
#define SPEED                           1000000

// CS is active low
#define CS_ASSERT                       0
#define CS_DEASSERT                     1

// use GPIO P14 for output flow control
#define SPIFFY2_OUTPUT_FLOW_CTRL_PIN    14
#define SPIFFY2_OUTPUT_FLOW_CTRL_PORT   0

// use GPIO P2 for input flow control
#define SPIFFY2_INPUT_FLOW_CTRL_PIN     2
#define SPIFFY2_INPUT_FLOW_CTRL_PORT    0

// input flow control also defined as CS
#define CS_PORT                         SPIFFY2_INPUT_FLOW_CTRL_PORT
#define CS_PIN                          SPIFFY2_INPUT_FLOW_CTRL_PIN

// Max transaction size
#define SPIFFY2_MAX_NUMBER_OF_BYTES_PER_TRANSACTION 15
#define SPI_TRANSACTION_BUFFER_SIZE                 16

#define DEBUG_PORT                                  1
#define DEBUG_PIN                                   12

#define SPI_COMM_CHECK_BYTE                         0xA0

#define SPI_TRANSFER_STATE_IDLE                     0
#define SPI_TRANSFER_STATE_MASTER                   1
#define SPI_TRANSFER_STATE_SLAVE                    2
#define SPI_TRANSFER_STATE_ABORT                    3

#define SPI_TRANSFER_SUBSTATE_NONE                  0

#define SPI_TRANSFER_SUBSTATE_START                 1
#define SPI_TRANSFER_SUBSTATE_END                   2


/******************************************************
 *               Function Prototypes
 ******************************************************/

void application_gpio_interrupt_handler(void* parameter, UINT8 u8);
static void   spiffy2_master_initialize(void);

static void application_spiffy2_init_in_master_mode(void);
static UINT8 ringBufferAdd(UINT8* buffer, UINT8 length);
static UINT8 application_spiffy2_send_bytes(void);
static void spi_comm_master_create(void);
static void spi_comm_master_timeout(UINT32 arg);
static void spi_comm_master_fine_timeout(UINT32 arg);

UINT8       spiProcessingCheck(void);
UINT32      device_lpm_queriable(LowPowerModePollType type, UINT32 context);

/******************************************************
 *               Variables Definitions
 ******************************************************/
UINT32  timer_count         = 0;
UINT8   count               = 0;
UINT8   spiTransferState    = SPI_TRANSFER_STATE_IDLE;
UINT8   spiTransferSubState = SPI_TRANSFER_SUBSTATE_NONE;


UINT32  spiTxhwfifoHead = 0;
UINT32  spiTxhwfifoTail = 0;
UINT32  spiTxnumEmptyEntries = SPI_TRANSACTION_BUFFER_SIZE;

// The bytes we want to transmit as a slave.
UINT8 spi_master_bytes_to_tx[SPI_TRANSACTION_BUFFER_SIZE][SPIFFY2_MAX_NUMBER_OF_BYTES_PER_TRANSACTION];


// Following structure defines UART configuration
const BLE_PROFILE_PUART_CFG spi_comm_master_puart_cfg =
{
    /*.baudrate   =*/ 115200,
    /*.txpin      =*/ PUARTENABLE | GPIO_PIN_UART_TX,
    /*.rxpin      =*/ PUARTENABLE | GPIO_PIN_UART_RX,
};

// Following structure defines GPIO configuration used by the application
const BLE_PROFILE_GPIO_CFG spi_comm_master_gpio_cfg =
{
    /*.gpio_pin =*/
    {
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 // GPIOs are not used
    },
    /*.gpio_flag =*/
    {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    }
};

UINT16  spi_comm_master_data_client_configuration    = 0;
UINT16  spi_comm_master_control_client_configuration = 0;
UINT32  spi_comm_master_timer_count                  = 0;
UINT32  spi_comm_master_timer_timeout                = 0;

/******************************************************
 *               Function Definitions
 ******************************************************/

// Application initialization
APPLICATION_INIT()
{
    bleapp_set_cfg(NULL,
                   0,
                   NULL,
                   (void *)&spi_comm_master_puart_cfg,
                   (void *)&spi_comm_master_gpio_cfg,
                   spi_comm_master_create);

    // BLE_APP_DISABLE_TRACING();     ////// Uncomment to disable all tracing
    BLE_APP_ENABLE_TRACING_ON_PUART();
}

// Create spi comm master
void spi_comm_master_create(void)
{
    ble_trace0("spi_comm_master_create()\n");
    bleprofile_GPIOInit(bleprofile_gpio_p_cfg);

    // Initialization for SPI2 interface
    application_spiffy2_init_in_master_mode();

    bleprofile_regTimerCb(spi_comm_master_fine_timeout, spi_comm_master_timeout);
    bleprofile_StartTimer();

    devlpm_init(); //disable sleep
    devlpm_registerForLowPowerQueries(device_lpm_queriable, 0);
}

void spi_comm_master_timeout(UINT32 arg)
{
     //ble_trace0("\rMaster timeout");
}

void spi_comm_master_fine_timeout(UINT32 arg)
{
    //ble_trace0("Master fine timeout\n");
    UINT8 result;
#ifdef SPI_MASTER_TRANSMIT_ON_TIMEOUT
    UINT8 i;
    UINT8 buffer[SPIFFY2_MAX_NUMBER_OF_BYTES_PER_TRANSACTION-1];

    for (i=0; i<SPIFFY2_MAX_NUMBER_OF_BYTES_PER_TRANSACTION-1; i++)
    {
        buffer[i] = i + 0x0F; // differentiate master & slave data
    }
    timer_count++;

    if (timer_count == 10)
    {
        timer_count = 5;


        while (spiTxnumEmptyEntries > 0)
        {
            buffer[0] = count++;

            ringBufferAdd(buffer, SPIFFY2_MAX_NUMBER_OF_BYTES_PER_TRANSACTION-1);
            ble_trace1("adding buffer, num left: %d\n", spiTxnumEmptyEntries);

        }

        result = application_spiffy2_send_bytes();
        if (result)
            ble_trace1("master transmit to slave : %d\n", result);

    }
#else
    if (spiTxnumEmptyEntries != SPI_TRANSACTION_BUFFER_SIZE)
    {
        result = application_spiffy2_send_bytes();
        if (result)
            ble_trace1("master transmit: %d\n", result);
    }
#endif
    spi_comm_master_timer_count++;

    spiProcessingCheck();
}

// sends data to the slave
void masterToSlaveSend(void)
{

    UINT8 length = 0;
    UINT8 result[1];
    ble_trace0("\rMasterToSlave tranmitt");

    if ( (spiTxnumEmptyEntries != SPI_TRANSACTION_BUFFER_SIZE) &&
         (spiTransferState == SPI_TRANSFER_STATE_MASTER) )
    {
        ble_trace0("\rStarting M->S transaction\n");

        // CS is asserted, so this is a M->S transaction. We can transmit data to slave.
        ble_trace1("Transmitted Length: 0x%02X\nBytes:\n", spi_master_bytes_to_tx[spiTxhwfifoTail][0]);
        //ble_tracen(&spi_master_bytes_to_tx[spiTxhwfifoTail][1], spi_master_bytes_to_tx[spiTxhwfifoTail][0]);
        length = spi_master_bytes_to_tx[spiTxhwfifoTail][0]+1;

        spi_master_bytes_to_tx[spiTxhwfifoTail][0] |= SPI_COMM_CHECK_BYTE;

        spiffyd_txData(SPIFFYD_2, length, &spi_master_bytes_to_tx[spiTxhwfifoTail][0]);

        // update ring buffer variables
        spiTxnumEmptyEntries++;
        spiTxhwfifoTail++;
        if (spiTxhwfifoTail == SPI_TRANSACTION_BUFFER_SIZE)
        {
            spiTxhwfifoTail = 0;
        }

        // deassert CSB  to end the command
        gpio_setPinOutput(CS_PORT, CS_PIN, CS_DEASSERT);   // Deassert chipselect

        spi_comm_master_timer_timeout = spi_comm_master_timer_count + 2;
        spiTransferSubState = SPI_TRANSFER_SUBSTATE_END;
    }
}

// Start Slave to Master transaction
void slaveToMasterStart(void)
{
    // pull CSB low to start the command
    gpio_setPinOutput(CS_PORT, CS_PIN, CS_ASSERT);     // Assert chipselect
    spiTransferState = SPI_TRANSFER_STATE_SLAVE;
    spiTransferSubState = SPI_TRANSFER_SUBSTATE_START;

    spi_comm_master_timer_timeout = spi_comm_master_timer_count + 2;

    ble_trace0("Starting S->M transaction\n");
}

// Start Master to Slave transaction
void masterToSlaveStart(void)
{
    //ble_trace0("\rmaster to slave transmit");
    // Assert CS to indicate that we want to transmit something to the slave.
    gpio_setPinOutput(CS_PORT, CS_PIN, CS_ASSERT);     // Assert chipselect

    spiTransferState = SPI_TRANSFER_STATE_MASTER;
    spiTransferSubState = SPI_TRANSFER_SUBSTATE_START;

    spi_comm_master_timer_timeout = spi_comm_master_timer_count + 2;

}

// Abort Master transcation
void masterToAbort(void)
{
    gpio_setPinOutput(CS_PORT, CS_PIN, CS_DEASSERT);     // Deassert chipselect
    spiTransferState = SPI_TRANSFER_STATE_ABORT;
    spiTransferSubState = SPI_TRANSFER_SUBSTATE_NONE;
    spi_comm_master_timer_timeout = spi_comm_master_timer_count + 1;
}

// End Master transcation
void masterToIdle(void)
{
    gpio_setPinOutput(CS_PORT, CS_PIN, CS_DEASSERT);     // Deassert chipselect
    spiTransferState = SPI_TRANSFER_STATE_IDLE;
    spiTransferSubState = SPI_TRANSFER_SUBSTATE_NONE;
}

// Abort Slave transcation
void slaveToAbort(void)
{
    gpio_setPinOutput(CS_PORT, CS_PIN, CS_DEASSERT);     // Deassert chipselect
    spiTransferState = SPI_TRANSFER_STATE_ABORT;
    spiTransferSubState = SPI_TRANSFER_SUBSTATE_NONE;
    spi_comm_master_timer_timeout = spi_comm_master_timer_count + 1;
}

// End Slave transcation
void slaveToIdle(void)
{
    gpio_setPinOutput(CS_PORT, CS_PIN, CS_DEASSERT);     // Deassert chipselect
    spiTransferState = SPI_TRANSFER_STATE_IDLE;
    spiTransferSubState = SPI_TRANSFER_SUBSTATE_NONE;
}

// Perform Slave to Master transaction
void slaveToMasterReceive(void)
{
    // try to receive 15 bytes at a time at most
    UINT8 buffer[SPIFFY2_MAX_NUMBER_OF_BYTES_PER_TRANSACTION];
    UINT8 length = 0;

    memset(buffer, 0x00, sizeof(buffer));

    // Get the first byte and check the length
    spiffyd_rxData(SPIFFYD_2, 1, &length);

    //ble_trace1("Received Length: 0x%0x Bytes\n", length);

    if ((length & 0xF0) == SPI_COMM_CHECK_BYTE)
    {
        length = length & 0x0F;

        ble_trace1("SPI master received 0x%0x Bytes from slave \n", length);

        if (length < SPIFFY2_MAX_NUMBER_OF_BYTES_PER_TRANSACTION)
        {
            // M->S transaction falling edge transition.
            // First byte is the bytes in this transaction.
            if (length > 0)
            {
                spiffyd_rxData(SPIFFYD_2, length, &buffer[1]);
            }

             ble_tracen((char *)&buffer[1], length);
        }
    }
    else
    {
        ble_trace1("First byte failed comm check : 0x%0x \n", length);
    }

    // deassert CSB  to end the command
    gpio_setPinOutput(CS_PORT, CS_PIN, CS_DEASSERT);   // Deassert chipselect

    spi_comm_master_timer_timeout = spi_comm_master_timer_count + 2;
    spiTransferSubState = SPI_TRANSFER_SUBSTATE_END;
}

// Based on states and FC, perform next step in the transaction
UINT8 spiProcessingCheck(void)
{

    UINT8 result;
    UINT8 retVal = 0;

    if (spiTransferState == SPI_TRANSFER_STATE_SLAVE)
    {
        if (spiTransferSubState == SPI_TRANSFER_SUBSTATE_START)
        {
            // Check FCO is low indicating request for S->M transaction
            if (!gpio_getPinInput(SPIFFY2_OUTPUT_FLOW_CTRL_PORT, SPIFFY2_OUTPUT_FLOW_CTRL_PIN))
            {
                slaveToMasterReceive();
                retVal = TRUE;
            }
            else if (spi_comm_master_timer_count >= spi_comm_master_timer_timeout)
            {
                slaveToAbort();
            }
        }

        if (spiTransferSubState == SPI_TRANSFER_SUBSTATE_END)
        {
            // If slave requests again, that is ok as well
            if (gpio_getPinInput(SPIFFY2_OUTPUT_FLOW_CTRL_PORT, SPIFFY2_OUTPUT_FLOW_CTRL_PIN))
            {
                slaveToMasterStart();
                retVal = TRUE;
            }
            // No indication so just need to timeout
            else if (spi_comm_master_timer_count >= spi_comm_master_timer_timeout)
            {
                slaveToIdle();
            }
        }
    }

    if (spiTransferState == SPI_TRANSFER_STATE_MASTER)
    {
        if (spiTransferSubState == SPI_TRANSFER_SUBSTATE_START)
        {
            // Check FCO is high indicating ack for M->S transaction
            if (gpio_getPinInput(SPIFFY2_OUTPUT_FLOW_CTRL_PORT, SPIFFY2_OUTPUT_FLOW_CTRL_PIN))
            {
                masterToSlaveSend();
                retVal = TRUE;
            }
            else if (spi_comm_master_timer_count >= spi_comm_master_timer_timeout)
            {
                // assume race condition and slave was trying to send
                slaveToMasterReceive();
            }
        }

        if (spiTransferSubState == SPI_TRANSFER_SUBSTATE_END)
        {
            // Check for end of M->S transaction.
            if (!gpio_getPinInput(SPIFFY2_OUTPUT_FLOW_CTRL_PORT, SPIFFY2_OUTPUT_FLOW_CTRL_PIN))
            {
                masterToIdle();
                retVal = TRUE;
            }
            else if (spi_comm_master_timer_count >= spi_comm_master_timer_timeout)
            {
                masterToAbort();
            }
        }
    }


    if (spiTransferState == SPI_TRANSFER_STATE_ABORT)
    {
        if (spi_comm_master_timer_count >= spi_comm_master_timer_timeout)
        {
            masterToIdle();
        }
    }

    if (spiTransferState == SPI_TRANSFER_STATE_IDLE)
    {
        if (gpio_getPinInput(SPIFFY2_OUTPUT_FLOW_CTRL_PORT, SPIFFY2_OUTPUT_FLOW_CTRL_PIN))
        {
            slaveToMasterStart();

            // If slave asserted FCO while we are processing the data, set retVal so we can process again
            retVal = TRUE;
        }
    }

    return(retVal);
}

// Thread context interrupt handler.
void application_gpio_interrupt_handler(void* parameter, UINT8 u8)
{
    UINT8 result = 1;
    // ble_trace0("\rgpio interrupt handler\n");
    while (result)
    {
        result = spiProcessingCheck();
    }

    while ((spiTxnumEmptyEntries != SPI_TRANSACTION_BUFFER_SIZE) && (spiTransferState == SPI_TRANSFER_STATE_IDLE))
    if ((spiTxnumEmptyEntries != SPI_TRANSACTION_BUFFER_SIZE) && (spiTransferState == SPI_TRANSFER_STATE_IDLE))
    {
        result = application_spiffy2_send_bytes();
        if (result)
        {
            ble_trace1("master transmit: %d\n", result);
        }
    }

    // Pending interrupt on this GPIO will automatically be cleared by the driver.
}

// initialize spiffy2 as SPI master
void application_spiffy2_init_in_master_mode(void)
{
    // We will use an interrupt for the first rx byte and then start a state machine from then on.
    UINT16 interrupt_handler_mask[3] = {0, 0, 0};

    // Use SPIFFY2 interface as master
    spi2PortConfig.masterOrSlave = MASTER2_CONFIG;

    // pull for MISO for master, MOSI/CLOCK/CS if slave mode
    spi2PortConfig.pinPullConfig = INPUT_PIN_PULL_UP;

    // Use P3 for CLK, P4 for MOSI and P1 for MISO
    spi2PortConfig.spiGpioConfig = MASTER2_P03_CLK_P04_MOSI_P01_MISO;

    // Initialize SPIFFY2 instance
    spiffyd_init(SPIFFYD_2);

    // Configure the SPIFFY2 HW block
    spiffyd_configure(SPIFFYD_2, SPEED, SPI_MSB_FIRST, SPI_SS_ACTIVE_LOW, SPI_MODE_3);

    gpio_configurePin(CS_PORT, CS_PIN, GPIO_OUTPUT_ENABLE | GPIO_INPUT_DISABLE, CS_DEASSERT);

    // configure GPIO used for output flow control (FCO) (input for the app)
    gpio_configurePin(SPIFFY2_OUTPUT_FLOW_CTRL_PORT, SPIFFY2_OUTPUT_FLOW_CTRL_PIN,
        GPIO_INPUT_ENABLE|GPIO_PULL_DOWN|GPIO_EN_INT_BOTH_EDGE,GPIO_PIN_OUTPUT_LOW);

    interrupt_handler_mask[SPIFFY2_OUTPUT_FLOW_CTRL_PORT] |= (1 << SPIFFY2_OUTPUT_FLOW_CTRL_PIN);

    // Now register the interrupt handler.
    gpio_registerForInterrupt(interrupt_handler_mask, application_gpio_interrupt_handler, NULL);

    // Clear out any spurious interrupt status.
    gpio_clearPinInterruptStatus(SPIFFY2_OUTPUT_FLOW_CTRL_PORT, SPIFFY2_OUTPUT_FLOW_CTRL_PIN);

}

// Adds buffer to the ring buffer
UINT8 ringBufferAdd(UINT8* buffer, UINT8 length)
{

    if (spiTxnumEmptyEntries == 0)
    {
        return(1);
    }

    // Copy over the data to a ring buffer so we can send it when the master is ready.
    spi_master_bytes_to_tx[spiTxhwfifoHead][0] = length;
    memcpy(&spi_master_bytes_to_tx[spiTxhwfifoHead][1], buffer, length);

    // update ring buffer variables
    spiTxnumEmptyEntries--;
    spiTxhwfifoHead++;
    if (spiTxhwfifoHead == SPI_TRANSACTION_BUFFER_SIZE)
    {
        spiTxhwfifoHead = 0;
    }

    return(0);
}
// Sends some bytes to the SPI slave. Bytes will be sent asynchronously.
UINT8 application_spiffy2_send_bytes(void)
{
    // check to see if we are not already doing a SPI transfer
    if (spiTransferState != SPI_TRANSFER_STATE_IDLE)
    {
        spiProcessingCheck();
        ble_trace1("spiTransferState %d\n", spiTransferState);
        return 1;
    }

    // read the input of CS to make sure it is not asserted so we are not already in another transaction
    if (gpio_getPinOutput(SPIFFY2_INPUT_FLOW_CTRL_PORT, SPIFFY2_INPUT_FLOW_CTRL_PIN) == CS_ASSERT)
    {
        spiProcessingCheck();
        return 2;
    }

    // If FCO is already high, either we are either transmitting or in the start of a receive already
    if (gpio_getPinInput(SPIFFY2_OUTPUT_FLOW_CTRL_PORT, SPIFFY2_OUTPUT_FLOW_CTRL_PIN))
    {
        spiProcessingCheck();
        return 3;
    }

    masterToSlaveStart();

    // If FCO is already high, may be race condition so abort
    if (gpio_getPinInput(SPIFFY2_OUTPUT_FLOW_CTRL_PORT, SPIFFY2_OUTPUT_FLOW_CTRL_PIN))
    {
        masterToAbort();
        return 4;
    }
    return 0;
}


// Callback called by the FW when ready to sleep/deep-sleep. Disable both by returning 0.
UINT32 device_lpm_queriable(LowPowerModePollType type, UINT32 context)
{
    // Disable sleep.
    return 0;
}
