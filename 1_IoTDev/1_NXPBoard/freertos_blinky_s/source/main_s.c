/*
 * FreeRTOS Pre-Release V1.0.0
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
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
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */

/* Board specific includes. */

/* Trustzone config. */
#include "resource_config.h"

/* FreeRTOS includes. */
#include "secure_port_macros.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_power.h"
#include "fsl_rtc.h"
#include "fsl_usart.h"
#include "fsl_debug_console.h"
#include "fsl_ctimer.h"
#include "mbedtls/ecdsa.h"
#include "mbedtls/ecdh.h"
#include "mbedtls/sha256.h"
#include "mbedtls/pk.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/entropy.h"

#include <stdbool.h>
#include <math.h>

#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#include "hacl-c/Hacl_Ed25519.h"
#else
#include MBEDTLS_CONFIG_FILE
#include "ksdk_mbedtls.h"
#endif
#include <stdio.h>
#include <time.h>

#include "nsc_functions.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/**
 * @brief Start address of non-secure application.
 */
#define mainNONSECURE_APP_START_ADDRESS DEMO_CODE_START_NS

/**
 * @brief LED port and pins.
 */
#define LED_PORT      BOARD_LED_BLUE_GPIO_PORT
#define GREEN_LED_PIN BOARD_LED_GREEN_GPIO_PIN
#define BLUE_LED_PIN  BOARD_LED_BLUE_GPIO_PIN

/**
 * @brief CTimer configuration.
 */
#define CTIMER          	CTIMER2         /* Timer 2 */
#define CTIMER_MAT_ATT_OUT 	kCTIMER_Match_1 /* Match output */
#define CTIMER_MAT_GEN_OUT 	kCTIMER_Match_2 /* Match output */
#define CTIMER_CLK_FREQ 	CLOCK_GetCTimerClkFreq(2U)

/**
 * @brief typedef for non-secure Reset Handler.
 */
#if defined(__IAR_SYSTEMS_ICC__)
typedef __cmse_nonsecure_call void (*NonSecureResetHandler_t)(void);
#else
typedef void (*NonSecureResetHandler_t)(void) __attribute__((cmse_nonsecure_call));
#endif

#define WIFI_USART          	USART4
#define WIFI_USART_CLK_SRC  	kCLOCK_Flexcomm4
#define WIFI_USART_IRQn         FLEXCOMM4_IRQn
#define WIFI_USART_IRQHandler   FLEXCOMM4_IRQHandler
#define WIFI_USART_CLK_FREQ 	CLOCK_GetFlexCommClkFreq(0U)

#define BUF_SIZE			(256)
#define SIG_SIZE			(256)
#define NONCE_SIZE			(12)
#define TIME_SIZE			(4)
#define ID_SIZE				(4)
#define HASH_SIZE			(32)
#define ATT_SIZE			(1)
#define TIME_PREV			(1681506039)
//#define PRV_DEV_KEY_PEM		"-----BEGIN EC PRIVATE KEY-----\r\nMHcCAQEEIF3U39mcfT5CzujDNem0gk4x1bzPodlveTZZhKbJdtFToAoGCCqGSM49\r\nAwEHoUQDQgAEuQnbuq0OifGY0Fb9TlVw+Y8wXX28TiW+Yq38CIx5sVghlTjBmuFh\r\nm0yBJr5L88OHBd9ymb3S5idXq0EStfbv3Q==\r\n-----END EC PRIVATE KEY-----"
#define PRV_DEV_KEY_PEM			"-----BEGIN EC PRIVATE KEY-----\r\nMHcCAQEEICpv1EKjbxgba94GKHDi96MnOIe7QD84tnycCDHR4irOoAoGCCqGSM49\r\nAwEHoUQDQgAE8yB1KwQQFgklJZD7L2rXr41DKBMgVYm5XDj93duhkxqNUi6f4Z7M\r\nHPFPktKrSkwyZaKbXoNc6OcQVNreR1RTqA==\r\n-----END EC PRIVATE KEY-----"
#define PUB_M_SRV_KEY_PEM 		"-----BEGIN PUBLIC KEY-----\r\nMFkwEwYHKoZIzj0CAQYIKoZIzj0DAQcDQgAEp5WVs1qXLCPdYresNZkyJ192FxXA\r\nTxFzfZHwtWX+xs50yc4x4ax7sNrzWyAe3F87ZZ8MpK+e60gEJumTrp6mzA==\r\n-----END PUBLIC KEY-----"
#define M_SRV_URL				"bitaaccdda"
#define MSG_END_CHAR		"MSGEND"
#define ACK_END_CHAR		"ACKEND"
#define BRD_END_CHAR		"BRDEND"

#define T_ATT				(300)
#define T_GEN				(1)

#define CONVERT_MS_TO_S		(1000)

/*-----------------------------------------------------------*/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/**
 * @brief Application-specific implementation of the SystemInitHook().
 */
void SystemInitHook(void);

/**
 * @brief Boots into the non-secure code.
 *
 * @param[in] ulNonSecureStartAddress Start address of the non-secure application.
 */
void BootNonSecure(uint32_t ulNonSecureStartAddress);

/**
 * @brief Broadcast the message with signature.
 */
void response();

/**
 * @brief Attest the memory.
 */
uint8_t attest();

/*-----------------------------------------------------------*/


void ctimer_match_callback(uint32_t flags);

void ctimer_match_start(uint8_t ctimer_match);

/* Array of function pointers for callback for each channel */
ctimer_callback_t ctimer_callback_table[] = {
    NULL, ctimer_match_callback, ctimer_match_callback, NULL, NULL, NULL, NULL, NULL};

/*******************************************************************************
 * Global Variables
 ******************************************************************************/

mbedtls_pk_context private_key = {0, };
mbedtls_ctr_drbg_context ctr_drbg = {0, };
static volatile uint32_t s_MsCount = 0U;

static uint8_t attest_result[ATT_SIZE] = {0};

static uint8_t req_msg[BUF_SIZE] = {0, };
static size_t req_msg_len = 0;

static uint8_t num_req = 0U;
static uint8_t n_list[BUF_SIZE*NONCE_SIZE] = {0, };
static uint8_t tmp_num_req = 0U;
static uint8_t tmp_n_list[BUF_SIZE] = {0, };
static uint8_t r_flag = 0U;
#ifdef PERFORMANCE_EVALUATION
static uint8_t num_req_for_print = 0U;
#endif

static uint32_t t_attest = 0U;

/*-----------------------------------------------------------*/


/*******************************************************************************
 * Code
 ******************************************************************************/

void SystemInitHook(void)
{
    /* The TrustZone should be configured as early as possible after RESET.
     * Therefore it is called from SystemInit() during startup. The
     * SystemInitHook() weak function overloading is used for this purpose.
     */
    BOARD_InitTrustZone();
}
/*-----------------------------------------------------------*/

void BootNonSecure(uint32_t ulNonSecureStartAddress)
{
    NonSecureResetHandler_t pxNonSecureResetHandler;

    /* Main Stack Pointer value for the non-secure side is the first entry in
     * the non-secure vector table. Read the first entry and assign the same to
     * the non-secure main stack pointer(MSP_NS). */
    secureportSET_MSP_NS(*((uint32_t *)(ulNonSecureStartAddress)));

    /* Reset handler for the non-secure side is the second entry in the
     * non-secure vector table. */
    pxNonSecureResetHandler = (NonSecureResetHandler_t)(*((uint32_t *)((ulNonSecureStartAddress) + 4U)));

    /* Start non-secure state software application by jumping to the non-secure
     * reset handler. */
    pxNonSecureResetHandler();
}


/*-----------------------------------------------------------*/

/*!
 * @brief Milliseconds counter since last POR/reset.
 */
void SysTick_Handler(void)
{
    s_MsCount++;
}

void ctimer_match_callback(uint32_t flags)
{
#ifdef PERFORMANCE_EVALUATION
	uint32_t before_cycles = DWT->CYCCNT;
	uint32_t after_cycles = DWT->CYCCNT;
	uint32_t time1, time2;
#endif

	volatile unsigned long val = CTIMER_GetTimerCountValue(CTIMER);

    if (flags == kCTIMER_Match1Flag)
	{
    	DWT->CYCCNT = 0;
#ifdef PERFORMANCE_EVALUATION
		before_cycles = 0;
#endif

		*attest_result = attest();

#ifdef PERFORMANCE_EVALUATION
		after_cycles = DWT->CYCCNT;

		PRINTF("[Attestation] Cycle consumed: %u cycles\n\r", after_cycles - before_cycles);
#endif

    	CTIMER_ClearStatusFlags(CTIMER, kCTIMER_Match1Flag);
    	ctimer_match_start(CTIMER_MAT_ATT_OUT);
//    	CTIMER_EnableInterrupts(CTIMER, kCTIMER_Match1InterruptEnable);
	}

    if (flags == kCTIMER_Match2Flag)
	{
    	CTIMER_DisableInterrupts(CTIMER, kCTIMER_Match1InterruptEnable);
    	CTIMER_DisableInterrupts(CTIMER, kCTIMER_Match2InterruptEnable);
    	if (num_req != 0) {
    		r_flag = 1;
#ifdef PERFORMANCE_EVALUATION
    		time1 = CTIMER_GetTimerCountValue(CTIMER);
#endif

    		response();
#ifdef PERFORMANCE_EVALUATION
    		time2 = CTIMER_GetTimerCountValue(CTIMER);
    		PRINTF("Time1: %u, Time2: %u\r\n", time1, time2);
#endif
    		r_flag = 0;
    	}
    	CTIMER_EnableInterrupts(CTIMER, kCTIMER_Match1InterruptEnable);

#ifdef PERFORMANCE_EVALUATION
		after_cycles = DWT->CYCCNT;

		PRINTF("[Response] Cycle consumed: %u cycles, nonces: %u\n\r", after_cycles - before_cycles, num_req_for_print);
#endif

		CTIMER_ClearStatusFlags(CTIMER, kCTIMER_Match2Flag);

	}

}


void WIFI_USART_IRQHandler(void)
{
//	uint8_t msg[BUF_SIZE] = {0, };
//	size_t msg_len = 0;

	/* If new data arrived. */
	while ((kUSART_RxFifoNotEmptyFlag | kUSART_RxError) & USART_GetStatusFlags(WIFI_USART))
    {
		req_msg[req_msg_len++] = USART_ReadByte(WIFI_USART);
        if (req_msg_len >= BUF_SIZE)
        {
            // error handling
        	PRINTF("Request message is too long\n");
        }
    }

	// It receives the message until getting MSG_END_CHAR
	if (req_msg_len < strlen(MSG_END_CHAR) || strcmp((char *)req_msg + req_msg_len - strlen(MSG_END_CHAR), MSG_END_CHAR) != 0)
	{
		USART_ClearStatusFlags(WIFI_USART, kUSART_RxError);
		SDK_ISR_EXIT_BARRIER;
		return;
	}

	// if response message is being generated
	if (r_flag == 1) {
		memcpy(tmp_n_list + tmp_num_req * NONCE_SIZE, req_msg, NONCE_SIZE);
		tmp_num_req += 1;
	}
	else {
		// Lazy-response technique
		// If this is the first request within T_Gen
		if (num_req == 0)
		{
			// Start the timer ringing in T_Gen
			ctimer_match_start(CTIMER_MAT_GEN_OUT);
//			CTIMER_EnableInterrupts(CTIMER, kCTIMER_Match2InterruptEnable);
		}
		else {
			// Otherwise, just copy a nonce to the response message
			memcpy(n_list + num_req * NONCE_SIZE, req_msg, NONCE_SIZE);
		}
		num_req += 1;
	}

	req_msg_len = 0;
	USART_ClearStatusFlags(WIFI_USART, kUSART_RxError);

    SDK_ISR_EXIT_BARRIER;
}

int getEntropyItfFunction(void* userData,uint8_t* buffer,size_t bytes)
{
	int i;
	for(i = 0; i < bytes ; i++)
	{
		buffer[i] = i;
	}

	return 0;
}

void __sha256(const char *msg, size_t msg_len, char *digest)
{
	mbedtls_sha256_context sha256 = {0, };

	mbedtls_sha256_init(&sha256);
	mbedtls_sha256_starts(&sha256, 0);
	mbedtls_sha256_update(&sha256, msg, msg_len);
	mbedtls_sha256_finish(&sha256, digest);
}


/* Memory location to attest. */
#define MEM_SIZE 		0x10000				/* Length of FLASH 1 with NS-User privilege */
static unsigned long saddr = 0x40000;	/* Start of FLASH 1 with NS-User privilege */

// Due to the limitation of mbedTLS SHA256 library implemented on NXP,
// it couldn't hash data larger than 64KB in Secure World even with 192MB secure RAM,
// so it's inevitable to divide data into 4KB chucks.
#define MEM_SIZE_4K		0x1000



uint8_t attest()
{
	// It is assumed that the hash of expected result would be stored in the secure flash memory
	// Given digest is hashed value of address 0x40000 with the size of 0x30000
//	uint8_t expected_digest[HASH_SIZE] = { 0x4B, 0x95, 0x99, 0x39, 0xC0, 0xD7, 0xF5, 0x0A,
//			0x34, 0xF2, 0xA5, 0xDB, 0x50, 0x66, 0x24, 0x22,
//			0x75, 0x74, 0x60, 0x5C, 0x09, 0xB8, 0xE1, 0x3E,
//			0x37, 0x5D, 0xBE, 0x73, 0xBF, 0xCB, 0xE1, 0xFD,
//	};
	uint8_t expected_digest[HASH_SIZE] = { 0x97, 0x8E, 0xB9, 0x04, 0x92, 0x36, 0xF4, 0x91,
			0x2F, 0x03, 0x93, 0x59, 0x85, 0xCA, 0x21, 0x45,
			0x72, 0x73, 0xE9, 0x8F, 0xD5, 0x86, 0x0B, 0xB2,
			0x28, 0xCF, 0xF5, 0x8F, 0x23, 0x8D, 0xAF, 0x35,
		};
	uint8_t digest[HASH_SIZE] = {0, };
	uint8_t digest_hash_chain[HASH_SIZE*MEM_SIZE/MEM_SIZE_4K];

	for (int i=0; i<MEM_SIZE/MEM_SIZE_4K; i++) {
		__sha256((uint8_t *)saddr+i*MEM_SIZE_4K, MEM_SIZE_4K, digest_hash_chain+i*HASH_SIZE);
	}
	__sha256(digest_hash_chain, sizeof(digest_hash_chain), digest);

//	t_attest = s_MsCount;
	t_attest = CTIMER_GetTimerCountValue(CTIMER);

	if (memcmp(expected_digest, digest, HASH_SIZE) != 0)
		return 1;

	return 0;
}

// Currently, the left message of non-secure applications get erased if announcement takes place.
// This can be readily modified to keep the left message if USART method is converted to non-block transmit/receive.
void broadcast_msg_w_priority(const uint8_t *msg, size_t msg_len)
{
	USART_Type *base = WIFI_USART;

	// if the USART is occupied by other non-secure applications
	if ( (0U == (base->STAT & USART_STAT_TXIDLE_MASK)) ){
		// USART disable
		base->CFG &= ~(USART_CFG_ENABLE_MASK);
		// wait until USART being disabled
		while (base->CFG & USART_CFG_ENABLE_MASK == USART_CFG_ENABLE_MASK);
		// USART enable
		base->CFG |= USART_CFG_ENABLE_MASK;
	}

	USART_WriteBlocking(WIFI_USART, msg, msg_len);
}


void response()
{
	uint8_t msg[BUF_SIZE] = {0, };
	size_t msg_len = 0;
	uint8_t digest[HASH_SIZE] = {0, };
	uint8_t signature[SIG_SIZE] = {0, };
	uint8_t n_dev[NONCE_SIZE] = {0, };
	mbedtls_sha256_context sha256 = {0, };
	int ret = 0;
	uint8_t m_srv_url_len = strlen(M_SRV_URL);

	uint32_t attest_time;
	uint32_t cycles_before, cycles_after = 0;


	// Response procedure (TS would be constant value here)

	// msg: ["DP-RES" (6) || n_dev(12) || num_of_n_usr(1) || n_usr_list(12*num_of_n_usr) || M_SRV_URL(10) || attest_result(1) || attest_time(4) || signature(variable)]

	ret = mbedtls_ctr_drbg_random(&ctr_drbg, n_dev, NONCE_SIZE);

	memcpy(msg+msg_len, n_dev, NONCE_SIZE);
	msg_len += NONCE_SIZE;

	memcpy(msg+msg_len, &num_req, sizeof(num_req));
	msg_len += sizeof(num_req);

	for (int i=0; i<num_req; i++) {
		memcpy(msg+msg_len, n_list + i*NONCE_SIZE, NONCE_SIZE);
		msg_len += NONCE_SIZE;
	}

	memcpy(msg+msg_len, M_SRV_URL, strlen(M_SRV_URL));
	msg_len += strlen(M_SRV_URL);

	memcpy(msg+msg_len, attest_result, ATT_SIZE);
	msg_len += ATT_SIZE;

	attest_time = CTIMER_GetTimerCountValue(CTIMER) - t_attest;

	memcpy(msg+msg_len, &attest_time, sizeof(attest_time));
	msg_len += sizeof(attest_time);

	__sha256(msg, msg_len, digest);

	size_t sig_len = 0;
#ifdef PERFORMANCE_EVALUATION
	cycles_before = DWT->CYCCNT;
#endif
	ret = mbedtls_pk_sign (&private_key, MBEDTLS_MD_SHA256, digest, HASH_SIZE, signature,
			&sig_len, mbedtls_ctr_drbg_random, &ctr_drbg);
#ifdef PERFORMANCE_EVALUATION
	cycles_after = DWT->CYCCNT;
	PRINTF("[Signing] Cycle consumed: %u cycles\n\r", cycles_after - cycles_before);
#endif
	if(ret != 0){while(1);}

	memcpy(msg+msg_len, signature, sig_len);
	msg_len += sig_len;

	memcpy(msg+msg_len, BRD_END_CHAR, strlen(BRD_END_CHAR));
	msg_len += strlen(BRD_END_CHAR);

	broadcast_msg_w_priority(msg, msg_len);

	// assign temporary nonce list to main nonce list, and reset temporary num_req to 0
#ifdef PERFORMANCE_EVALUATION
	num_req_for_print = num_req;
#endif
	num_req = tmp_num_req;
	memcpy(n_list, tmp_n_list, NONCE_SIZE*tmp_num_req);
	tmp_num_req = 0;
}


/* Secure main(). */
/*!
 * @brief Main function
 */

// TODO: Currently, if failure takes place, then just stops working. We might need to keep working normally, but without broadcasting
#define RX_RING_BUFFER_SIZE 100U

uint8_t g_rxRingBuffer[RX_RING_BUFFER_SIZE] = {0}; /* RX ring buffer. */
uint8_t g_resp_bufferfer[RX_RING_BUFFER_SIZE] = {0}; /* Buffer for receive data to echo. */


uint8_t ringBuffer[RX_RING_BUFFER_SIZE];
volatile uint16_t rxIndex = 0; /* Index of the memory to save new arrived data. */

void ctimer_match_start(uint8_t ctimer_match)
{
	ctimer_match_config_t matchConfig;
	unsigned long matchValue;

	/* Configuration 0 */
	matchConfig.enableCounterReset = false;
	matchConfig.enableCounterStop  = false;
	if (ctimer_match == CTIMER_MAT_ATT_OUT) {
		matchValue = T_ATT*1000;
	} else {
		matchValue = T_GEN*1000;
	}
	matchValue += CTIMER_GetTimerCountValue(CTIMER);
	matchConfig.matchValue         = matchValue;
	matchConfig.outControl         = kCTIMER_Output_Toggle;
	matchConfig.outPinInitState    = false;
	matchConfig.enableInterrupt    = true;

	CTIMER_StopTimer(CTIMER);
	CTIMER_SetupMatch(CTIMER, ctimer_match, &matchConfig);
	CTIMER_StartTimer(CTIMER);
}

void ctimer_init()
{
	ctimer_config_t config;
	ctimer_match_config_t matchConfigAtt, matchConfigGen;

	CTIMER_GetDefaultConfig(&config);
	config.prescale = CTIMER_CLK_FREQ/1000 - 1;
	CTIMER_Init(CTIMER, &config);

	/* Configuration 0 */
	matchConfigAtt.enableCounterReset = false;
	matchConfigAtt.enableCounterStop  = false;
	matchConfigAtt.matchValue         = T_ATT*1000;
	matchConfigAtt.outControl         = kCTIMER_Output_Toggle;
	matchConfigAtt.outPinInitState    = false;
	matchConfigAtt.enableInterrupt    = true;

	/* Configuration 1 */
	matchConfigGen.enableCounterReset = false;
	matchConfigGen.enableCounterStop  = false;
	matchConfigGen.matchValue         = T_GEN*1000;
	matchConfigGen.outControl         = kCTIMER_Output_Toggle;
	matchConfigGen.outPinInitState    = false;
	matchConfigGen.enableInterrupt    = true;


	CTIMER_RegisterCallBack(CTIMER, ctimer_callback_table, kCTIMER_MultipleCallback);
	CTIMER_SetupMatch(CTIMER, CTIMER_MAT_ATT_OUT, &matchConfigAtt);
	CTIMER_SetupMatch(CTIMER, CTIMER_MAT_GEN_OUT, &matchConfigGen);
	CTIMER_StartTimer(CTIMER);
	CTIMER_DisableInterrupts(CTIMER, kCTIMER_Match2InterruptEnable);
//	CTIMER_EnableInterrupts(CTIMER, kCTIMER_Match1InterruptEnable);

	NVIC_SetPriority(CTIMER2_IRQn, 1);
	NVIC_SetPriority(FLEXCOMM4_IRQn, 0);
}



int main(void)
{
	usart_config_t config;
	uint8_t req_buffer[BUF_SIZE] = {0, };
	uint8_t resp_buffer[BUF_SIZE] = {0, };
	int ret = 0;
	uint32_t cycle_records = 0;
	mbedtls_entropy_context entropy = {0, };


	/* Init DWT at the beginning of main function*/
	DWT->CTRL |= (1 << DWT_CTRL_CYCCNTENA_Pos);
	DWT->CYCCNT = 0;

    /* Init board hardware. */
    /* set BOD VBAT level to 1.65V */
    POWER_SetBodVbatLevel(kPOWER_BodVbatLevel1650mv, kPOWER_BodHystLevel50mv, false);
    gpio_pin_config_t xLedConfig = {.pinDirection = kGPIO_DigitalOutput, .outputLogic = 1};

    /* Initialize GPIO for LEDs. */
    GPIO_PortInit(GPIO, LED_PORT);
    GPIO_PinInit(GPIO, LED_PORT, GREEN_LED_PIN, &(xLedConfig));
    GPIO_PinInit(GPIO, LED_PORT, BLUE_LED_PIN, &(xLedConfig));

    /* Set non-secure vector table */
    SCB_NS->VTOR = mainNONSECURE_APP_START_ADDRESS;

    /* attach main clock divide to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    /* Use FRO_HF clock as input clock source. */
    CLOCK_AttachClk(kFRO_HF_to_CTIMER2);

    /* enable clock for GPIO*/
	CLOCK_EnableClock(kCLOCK_Gpio0);
	CLOCK_EnableClock(kCLOCK_Gpio1);

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

	if( CRYPTO_InitHardware() != kStatus_Success )
	{
		PRINTF( "Initialization of crypto HW failed\n\r" );
		while(1);
	}

	/* Init SysTick module */
	/* call CMSIS SysTick function. It enables the SysTick interrupt at low priority */
	SysTick_Config(CLOCK_GetCoreSysClkFreq() / CONVERT_MS_TO_S); /* 1 ms period */

    USART_GetDefaultConfig(&config);
	config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
	config.enableTx     = true;
	config.enableRx     = true;

	USART_Init(WIFI_USART, &config, WIFI_USART_CLK_FREQ);

    /* Init RTC */
   	RTC_Init(RTC);

   	mbedtls_pk_init(&private_key);

	ret = mbedtls_pk_parse_key(&private_key, PRV_DEV_KEY_PEM, strlen(PRV_DEV_KEY_PEM)+1, NULL, 0);
	if(ret != 0){while(1);}
	mbedtls_entropy_init(&entropy);
	mbedtls_ctr_drbg_init( &ctr_drbg );
	ret = mbedtls_ctr_drbg_seed(&ctr_drbg,
								mbedtls_entropy_func,
								&entropy,
								(const unsigned char *) "RANDOM_GEN",
								10);
	if(ret != 0){while(1);}

   	/* init CTimer */
   	ctimer_init();


	/* Enable RX interrupt. */
	USART_EnableInterrupts(WIFI_USART, kUSART_RxLevelInterruptEnable | kUSART_RxErrorInterruptEnable);
	EnableIRQ(WIFI_USART_IRQn);

   	PRINTF("Finish booting process\n\r");

#ifdef PERFORMANCE_EVALUATION
   	cycle_records = DWT->CYCCNT;

   	PRINTF("Pure boot\n\r");
   	PRINTF("%u\n\r", cycle_records);
#endif

   	/* Boot the non-secure code. */
	BootNonSecure(mainNONSECURE_APP_START_ADDRESS);

    /* Non-secure software does not return, this code is not executed. */
    for (;;)
    {
    }

    // clean up
	mbedtls_ctr_drbg_free(&ctr_drbg);
	mbedtls_entropy_free(&entropy);
}
/*-----------------------------------------------------------*/

void vGetRegistersFromStack(uint32_t *pulFaultStackAddress)
{
    /* These are volatile to try and prevent the compiler/linker optimising them
     * away as the variables never actually get used.  If the debugger won't show the
     * values of the variables, make them global my moving their declaration outside
     * of this function. */
    volatile uint32_t r0;
    volatile uint32_t r1;
    volatile uint32_t r2;
    volatile uint32_t r3;
    volatile uint32_t r12;
    volatile uint32_t lr;  /* Link register. */
    volatile uint32_t pc;  /* Program counter. */
    volatile uint32_t psr; /* Program status register. */
    volatile uint32_t _CFSR;
    volatile uint32_t _HFSR;
    volatile uint32_t _DFSR;
    volatile uint32_t _AFSR;
    volatile uint32_t _SFSR;
    volatile uint32_t _BFAR;
    volatile uint32_t _MMAR;
    volatile uint32_t _SFAR;

    r0 = pulFaultStackAddress[0];
    r1 = pulFaultStackAddress[1];
    r2 = pulFaultStackAddress[2];
    r3 = pulFaultStackAddress[3];

    r12 = pulFaultStackAddress[4];
    lr  = pulFaultStackAddress[5];
    pc  = pulFaultStackAddress[6];
    psr = pulFaultStackAddress[7];

    /* Configurable Fault Status Register. Consists of MMSR, BFSR and UFSR. */
    _CFSR = (*((volatile unsigned long *)(0xE000ED28)));

    /* Hard Fault Status Register. */
    _HFSR = (*((volatile unsigned long *)(0xE000ED2C)));

    /* Debug Fault Status Register. */
    _DFSR = (*((volatile unsigned long *)(0xE000ED30)));

    /* Auxiliary Fault Status Register. */
    _AFSR = (*((volatile unsigned long *)(0xE000ED3C)));

    /* Secure Fault Status Register. */
    _SFSR = (*((volatile unsigned long *)(0xE000EDE4)));

    /* Read the Fault Address Registers. Note that these may not contain valid
     * values. Check BFARVALID/MMARVALID to see if they are valid values. */
    /* MemManage Fault Address Register. */
    _MMAR = (*((volatile unsigned long *)(0xE000ED34)));

    /* Bus Fault Address Register. */
    _BFAR = (*((volatile unsigned long *)(0xE000ED38)));

    /* Secure Fault Address Register. */
    _SFAR = (*((volatile unsigned long *)(0xE000EDE8)));

    /* Remove compiler warnings about the variables not being used. */
    (void)r0;
    (void)r1;
    (void)r2;
    (void)r3;
    (void)r12;
    (void)lr;  /* Link register. */
    (void)pc;  /* Program counter. */
    (void)psr; /* Program status register. */
    (void)_CFSR;
    (void)_HFSR;
    (void)_DFSR;
    (void)_AFSR;
    (void)_SFSR;
    (void)_MMAR;
    (void)_BFAR;
    (void)_SFAR;

    /* When the following line is hit, the variables contain the register values. */
    for (;;)
    {
    }
}
/*-----------------------------------------------------------*/
