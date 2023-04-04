/*
 * Lab-Project-coreMQTT-Agent 201215
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"

#include "fsl_phy.h"
/* FreeRTOS+TCP Includes */
#include "FreeRTOS_IP.h"

#include "enet_ethernetif.h"

#include "fsl_phyksz8081.h"
#include "fsl_enet_mdio.h"
#include "fsl_gpio.h"
#include "fsl_iomuxc.h"

#include "ksdk_mbedtls.h"

#include "nxLog_App.h"
#include "mflash_drv.h"

#include "ex_sss_boot.h"

#include "mflash_file.h"
#include "kvstore.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#ifndef EXAMPLE_NETIF_INIT_FN
/*! @brief Network interface initialization function. */
#define EXAMPLE_NETIF_INIT_FN    ethernetif0_init
#endif /* EXAMPLE_NETIF_INIT_FN */

/* Address of PHY interface. */
#define EXAMPLE_PHY_ADDRESS    BOARD_ENET0_PHY_ADDRESS

/* MDIO operations. */
#define EXAMPLE_MDIO_OPS       enet_ops

/* PHY operations. */
#define EXAMPLE_PHY_OPS        phyksz8081_ops

/* ENET clock frequency. */
#define EXAMPLE_CLOCK_FREQ     CLOCK_GetFreq( kCLOCK_IpgClk )


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void Board_InitNetwork( void );

int app_main( void );

/*******************************************************************************
 * Variables
 ******************************************************************************/
const uint8_t ucIPAddress[ ipIP_ADDRESS_LENGTH_BYTES ] = { 192, 168, 1, 3 };
const uint8_t ucNetMask[ ipIP_ADDRESS_LENGTH_BYTES ] = { 0xFF, 0xFF, 0xFF, 0x00 };
const uint8_t ucGatewayAddress[ ipIP_ADDRESS_LENGTH_BYTES ] = { 192, 168, 1, 1 };
const uint8_t ucDNSServerAddress[ ipIP_ADDRESS_LENGTH_BYTES ] = { 208, 67, 222, 222 };
/* MAC address configuration. */
const uint8_t ucMACAddress[ ipMAC_ADDRESS_LENGTH_BYTES ] = { 0x02, 0x12, 0x13, 0x10, 0x15, 0x25 };

//static mdio_handle_t mdioHandle = { .ops = &EXAMPLE_MDIO_OPS };
//static phy_handle_t phyHandle = { .phyAddr = EXAMPLE_PHY_ADDRESS, .mdioHandle = &mdioHandle, .ops = &EXAMPLE_PHY_OPS };

struct netif ethernet_netif;

/*******************************************************************************
 * Secure element contexts
 ******************************************************************************/
static ex_sss_boot_ctx_t gex_sss_demo_boot_ctx;
ex_sss_boot_ctx_t * pex_sss_demo_boot_ctx = &gex_sss_demo_boot_ctx;

static ex_sss_cloud_ctx_t gex_sss_demo_tls_ctx;
ex_sss_cloud_ctx_t * pex_sss_demo_tls_ctx = &gex_sss_demo_tls_ctx;

const char * g_port_name = NULL;

static mflash_file_t dir_template[] =
{
    {
        .path = KVSTORE_FILE_PATH,
        .max_size = ( MFLASH_SECTOR_SIZE * 2U )
    },
    {}
};

/*******************************************************************************
 * Code
 ******************************************************************************/
void Board_InitNetwork( void )
{
	BaseType_t xResult;

	xResult = FreeRTOS_IPInit( ucIPAddress,
	                           ucNetMask,
	                           ucGatewayAddress,
	                           ucDNSServerAddress,
							   ucMACAddress );

	assert( xResult == pdPASS );
}

void BOARD_InitModuleClock( void )
{
    const clock_enet_pll_config_t config = { .enableClkOutput = true, .enableClkOutput25M = false, .loopDivider = 1 };

    CLOCK_InitEnetPll( &config );
}

void delay( void )
{
    volatile uint32_t i = 0;

    for( i = 0; i < 1000000; ++i )
    {
        __asm( "NOP" ); /* delay */
    }
}


/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Application entry point.
 */
int main( void )
{
    gpio_pin_config_t gpio_config = { kGPIO_DigitalOutput, 0, kGPIO_NoIntmode };

    /* Init board hardware. */
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
    BOARD_InitModuleClock();
    SCB_DisableDCache();

    IOMUXC_EnableMode( IOMUXC_GPR, kIOMUXC_GPR_ENET1TxClkOutputDir, true );

    GPIO_PinInit( GPIO1, 9, &gpio_config );
    GPIO_PinInit( GPIO1, 10, &gpio_config );
    /* pull up the ENET_INT before RESET. */
    GPIO_WritePinOutput( GPIO1, 10, 1 );
    GPIO_WritePinOutput( GPIO1, 9, 0 );
    delay();
    GPIO_WritePinOutput( GPIO1, 9, 1 );

    if( mflash_drv_init() != 0 )
    {
        PRINTF( ( "\r\nFailed to initialize flash driver.\r\n" ) );

        while( 1 )
        {
            /* Empty while. */
        }
    }

    vTaskStartScheduler();

    /* Should not reach here. */
    for( ; ; )
    {
    }
}

void vApplicationDaemonTaskStartupHook( void )
{
    /* Initialize file system. */
    if( mflash_init( dir_template, false ) != kStatus_Success )
    {
        PRINTF( "\r\nFailed to initialize file system.\r\n" );

        for( ; ; )
        {
            __asm( "NOP" );
        }
    }

    /* Initialize network. */
    Board_InitNetwork();

    /* Initialize Logging locks */
    if( nLog_Init() != 0 )
    {
        PRINTF( "\r\nLogging initialization failed.\r\n" );

        for( ; ; )
        {
            __asm( "NOP" );
        }
    }

    if( app_main() != pdPASS )
    {
        PRINTF( "\r\nApp main initialization failed.\r\n" );

        for( ; ; )
        {
            __asm( "NOP" );
        }
    }
}

/**
 * @brief Loop forever if stack overflow is detected.
 *
 * If configCHECK_FOR_STACK_OVERFLOW is set to 1,
 * this hook provides a location for applications to
 * define a response to a stack overflow.
 *
 * Use this hook to help identify that a stack overflow
 * has occurred.
 *
 */
void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    char * pcTaskName )
{
    PRINTF( "ERROR: stack overflow on task %s.\r\n", pcTaskName );

    portDISABLE_INTERRUPTS();

    /* Unused Parameters */
    ( void ) xTask;
    ( void ) pcTaskName;

    /* Loop forever */
    for( ; ; )
    {
        __asm( "NOP" );
    }
}

/**
 * @brief Warn user if pvPortMalloc fails.
 *
 * Called if a call to pvPortMalloc() fails because there is insufficient
 * free memory available in the FreeRTOS heap.  pvPortMalloc() is called
 * internally by FreeRTOS API functions that create tasks, queues, software
 * timers, and semaphores.  The size of the FreeRTOS heap is set by the
 * configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h.
 *
 */
void vApplicationMallocFailedHook()
{
    PRINTF( "ERROR: Malloc failed to allocate memory\r\n" );
    taskDISABLE_INTERRUPTS();

    /* Loop forever */
    for( ; ; )
    {
    }
}

/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
 * implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
 * used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t ** ppxIdleTaskTCBBuffer,
                                    StackType_t ** ppxIdleTaskStackBuffer,
                                    uint32_t * pulIdleTaskStackSize )
{
    /* If the buffers to be provided to the Idle task are declared inside this
     * function then they must be declared static - otherwise they will be allocated on
     * the stack and so not exists after this function exits. */
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle
     * task's state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/*-----------------------------------------------------------*/

BaseType_t xApplicationDNSQueryHook( const char * pcName )
{
	return 0;
}

BaseType_t xApplicationGetRandomNumber( uint32_t * pulNumber )
{
	*pulNumber = 0x1234ABCD;
	return pdTRUE;
}

void SendARP_task( void * arg )
#if 0
{
	uint32_t ipADDR_DST;
	FreeRTOS_inet_pton( FREERTOS_AF_INET, "192.168.1.2", &ipADDR_DST );
	static uint16_t i = 0;
	while( 1 )
	{
		FreeRTOS_OutputARPRequest( ipADDR_DST );

		FreeRTOS_SendPingRequest( ipADDR_DST, 20, 20 );
		configPRINTF(("Sent %u", i++ ));

		//vTaskDelay( pdMS_TO_TICKS( 500 ) );
	}
}
#else
{
	static const TickType_t xReceiveTimeOut = portMAX_DELAY;
	static struct freertos_sockaddr xClient, xBindAddress;
	static Socket_t serverSocket, xConnectedSocket;
	static uint8_t recvBuffer[200];
	static socklen_t xSize;
	BaseType_t xBacklog = 1;

	while( 1 )
	{
		serverSocket = FreeRTOS_socket( FREERTOS_AF_INET,
										FREERTOS_SOCK_STREAM,
										FREERTOS_SOCK_DEPENDENT_PROTO );

		configASSERT( serverSocket != FREERTOS_INVALID_SOCKET );

		/* Set a time out so accept() will just wait for a connection. */
		FreeRTOS_setsockopt( serverSocket,
							 0,
							 FREERTOS_SO_RCVTIMEO,
							 &xReceiveTimeOut,
							 sizeof( xReceiveTimeOut ) );

		/* Set a time out so accept() will just wait for a connection. */
		FreeRTOS_setsockopt( serverSocket,
							 0,
							 FREERTOS_SO_REUSE_LISTEN_SOCKET,
							 NULL,
							 0 );

		/* Set the listening port to 10000. */
		xBindAddress.sin_port = ( uint16_t ) 10000;
		xBindAddress.sin_port = FreeRTOS_htons( xBindAddress.sin_port );

		/* Bind the socket to the port that the client RTOS task will send to. */
		FreeRTOS_bind( serverSocket, &xBindAddress, sizeof( xBindAddress ) );

		/* Set the socket into a listening state so it can accept connections.
		The maximum number of simultaneous connections is limited to 20. */
		FreeRTOS_listen( serverSocket, xBacklog );

		/* Wait for incoming connections. */
		xConnectedSocket = FreeRTOS_accept( serverSocket, &xClient, &xSize );

		if( xConnectedSocket != FREERTOS_INVALID_SOCKET )
		{

		while( 1 )
		{
			memset( recvBuffer, 0, 200 );
			if( FreeRTOS_recv( serverSocket, recvBuffer, 200, 0 ) > 0 )
			{
                static uint8_t display = 0;
                char out;
                switch( display )
                {
                    case 0:
                    	out = '\\';
                    	break;
                    case 1:
                        out = '|';
                        break;
                    case 2:
                        out = '/';
                        break;
                    case 3:
                        out = '-';
                        break;
                }
                display++;
                if(display > 3)
                	display = 0;

                configPRINTF( ( "\b%c", out ) );

				FreeRTOS_send( serverSocket, recvBuffer, strlen( recvBuffer ), 0 );
			}
			else
			{
				configPRINTF( ( "ERROR Receiving!" ) );
				FreeRTOS_shutdown( serverSocket, 0U );
				FreeRTOS_closesocket( serverSocket );
				break;
			}
		}
		}
	}

	vTaskDelete( NULL );
}
#endif
#if ( ipconfigUSE_NETWORK_EVENT_HOOK == 1 )
    /* This function shall be defined by the application. */
    void vApplicationIPNetworkEventHook( eIPCallbackEvent_t eNetworkEvent )
    {
    	uint32_t ulIPAddress, ulNetMask, ulGatewayAddress, ulDNSServerAddress;
    	char cBuffer[ 16 ];
    	static uint8_t flag = pdFALSE;

    	if( eNetworkEvent == eNetworkUp )
    	{
			FreeRTOS_GetAddressConfiguration( &ulIPAddress, &ulNetMask, &ulGatewayAddress, &ulDNSServerAddress );
			FreeRTOS_inet_ntoa( ulIPAddress, cBuffer );
			FreeRTOS_printf( ( "\r\n\r\nIP Address: %s\r\n", cBuffer ) );

			FreeRTOS_inet_ntoa( ulNetMask, cBuffer );
			FreeRTOS_printf( ( "Subnet Mask: %s\r\n", cBuffer ) );

			FreeRTOS_inet_ntoa( ulGatewayAddress, cBuffer );
			FreeRTOS_printf( ( "Gateway Address: %s\r\n", cBuffer ) );

			FreeRTOS_inet_ntoa( ulDNSServerAddress, cBuffer );
			FreeRTOS_printf( ( "DNS Server Address: %s\r\n\r\n\r\n", cBuffer ) );

			if( flag == pdFALSE )
			{
				xTaskCreate( SendARP_task, "SendTask", configMINIMAL_STACK_SIZE*5,
							 NULL,
							 1,
							 NULL );
				flag = pdTRUE;
			}
    	}
    }
#endif
#if ( ipconfigSUPPORT_OUTGOING_PINGS == 1 )
    void vApplicationPingReplyHook( ePingReplyStatus_t eStatus,
                                    uint16_t usIdentifier )
    {
    	if( eStatus == eSuccess )
    	{
    	    configPRINTF( ( "Got ping reply. ID %u", usIdentifier ) );
    	}
    }
#endif

uint32_t ulApplicationGetNextSequenceNumber( uint32_t ulSourceAddress,
                                             uint16_t usSourcePort,
                                             uint32_t ulDestinationAddress,
                                             uint16_t usDestinationPort )
{
	return 0xABCD1234;
}

/**
 * @brief This is to provide the memory that is used by the RTOS daemon/time task.
 *
 * If configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
 * implementation of vApplicationGetTimerTaskMemory() to provide the memory that is
 * used by the RTOS daemon/time task.
 */
void vApplicationGetTimerTaskMemory( StaticTask_t ** ppxTimerTaskTCBBuffer,
                                     StackType_t ** ppxTimerTaskStackBuffer,
                                     uint32_t * pulTimerTaskStackSize )
{
    /* If the buffers to be provided to the Timer task are declared inside this
     * function then they must be declared static - otherwise they will be allocated on
     * the stack and so not exists after this function exits. */
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle
     * task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
/*-----------------------------------------------------------*/
