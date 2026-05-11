/*
 * sim7600g.h
 *
 *  Created on: Apr 16, 2026
 *      Author: test1
 */

#ifndef INC_SIM7600G_H_
#define INC_SIM7600G_H_


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>


//#define SIM_PWR_PORT      	GPIOA
//#define SIM_PWR_PIN       	GPIO_PIN_8

//#define SIM_AT_TIMEOUT    	5000
#define SIM_APN        			"airtelgprs.com"
#define SIM_RX_BUF				512


typedef enum {
    SIM_OK = 0,
    SIM_ERROR,
    SIM_TIMEOUT,
    SIM_NO_CARRIER
} SIM_Status;





/**
 * @brief  One-time certificate provisioning into SIM7600G filesystem.
 *
 *         Call this ONCE from a factory/provisioning sketch, not on every boot.
 *         Requires the certificate PEM content as null-terminated strings.
 *
 *         AT+CCERTDOWN uploads a file; the modem then prompts with
 *         ">" and waits for exactly <len> bytes followed by Ctrl-Z (0x1A).
 *
 * @param  ca_pem      AWS root CA (AmazonRootCA1.pem)
 * @param  cert_pem    Device certificate
 * @param  key_pem     Device private key
 */
void SIM_Provision(const char *ca_pem,
        				const char *cert_pem,
						const char *key_pem);

/**
 * @brief  Non-blocking line reader from USART1 DMA ring buffer.
 *
 *         Copies bytes from the circular DMA buffer into buf until '\n' is
 *         found or buf is full. Returns true when a complete line is ready.
 *         Leaves partial lines in an internal accumulator across calls.
 *
 * @param  buf  Destination buffer
 * @param  len  Buffer size (bytes)
 * @return true if a '\n'-terminated line was placed in buf
 */
SIM_Status SIM_ReadLine(char *buf, size_t len);

SIM_Status SIM_CheckIP(void);

const char *SIM_GetIP(void);

//static void _ClearBuf(void);

void SIM_Init(void);

void SIM_GetSignal(void);

SIM_Status SIM_4GLTENetwork(void);

void SIM_Tx(const char *cmd);

SIM_Status SIM_Wait(const char *expected, uint32_t timeout_ms);

SIM_Status SIM_ReadLine(char *buf, size_t len);

extern UART_HandleTypeDef huart1;

//static char rx_buf[SIM_RX_BUF_SIZE];
//static volatile uint16_t rx_idx = 0;

static uint8_t  s_dma_buf[SIM_RX_BUF];
static uint16_t s_rd_head = 0;
static char s_ip_addr[32] = {0};


#endif /* INC_SIM7600G_H_ */
