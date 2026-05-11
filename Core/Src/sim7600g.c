/*
 * sim7600g.c
 *
 *  Created on: Apr 16, 2026
 *      Author: test1
 */


#include "main.h"
#include "sim7600g.h"
#include "mqtt.h"


void SIM_Provision(const char *ca_pem,
                         const char *cert_pem,
                         const char *key_pem)
{
    char cmd[128];
    const struct { const char *name; const char *data; } certs[] = {
        { "AmazonRootCA1.pem",  ca_pem   },
        { "TSGP-001.cert.pem",  cert_pem },
        { "TSGP-001.private.key",   key_pem  },
    };

    for (int i = 0; i < 3; i++)
    {
        size_t dlen = strlen(certs[i].data);

        /* Delete if already exists */
        snprintf(cmd, sizeof(cmd), "AT+CCERTDELE=\"%s\"\r\n", certs[i].name);
        SIM_Tx(cmd);
        HAL_Delay(500);

        /* Upload: AT+CCERTDOWN="filename",<len> */
        snprintf(cmd, sizeof(cmd),
                 "AT+CCERTDOWN=\"%s\",%zu\r\n", certs[i].name, dlen);
        SIM_Tx(cmd);

        SIM_Wait(">", 3000);    /* modem ready to receive */

        /* Send raw PEM bytes */
        HAL_UART_Transmit(&huart1,
                          (const uint8_t *)certs[i].data, dlen, 10000);

        /* Ctrl-Z to commit */
        const uint8_t ctrlz = 0x1A;
        HAL_UART_Transmit(&huart1, &ctrlz, 1, 1000);

        SIM_Wait("OK", 5000);
    }
}

SIM_Status SIM_ReadLine(char *buf, size_t len)
{
    static char  s_accum[512];
    static size_t s_accum_len = 0;

    /* DMA write pointer: buffer fills from index 0, CNDTR counts DOWN */
    uint16_t wr = SIM_RX_BUF - (uint16_t)huart1.hdmarx->Instance->CNDTR;

    while (s_rd_head != wr)
    {
        char c = (char)s_dma_buf[s_rd_head];
        s_rd_head = (s_rd_head + 1) % SIM_RX_BUF;

        if (c == '\r') continue;    /* ignore CR */

        if (c == '\n')
        {
            s_accum[s_accum_len] = '\0';
            if (s_accum_len > 0)
            {
                size_t copy = (s_accum_len < len - 1) ? s_accum_len : len - 1;
                memcpy(buf, s_accum, copy);
                buf[copy] = '\0';
                s_accum_len = 0;
                return SIM_OK;
            }
            /* empty line — keep going */
            continue;
        }

        if (s_accum_len < sizeof(s_accum) - 1)
            s_accum[s_accum_len++] = c;
    }
    return SIM_ERROR;
}

//static void _ClearBuf(void) {
//    memset(rx_buf, 0, SIM_RX_BUF_SIZE);
//    rx_idx = 0;
//}


///* ── Power-on sequence ───────────────────────────────────────────────────── */
//void SIM_PowerOn(void) {
//    // Pulse PWR_KEY LOW for 500ms
//    HAL_GPIO_WritePin(SIM_PWR_PORT, SIM_PWR_PIN, GPIO_PIN_RESET);
//    HAL_Delay(500);
//    HAL_GPIO_WritePin(SIM_PWR_PORT, SIM_PWR_PIN, GPIO_PIN_SET);
//    HAL_Delay(3000);  // boot time
//}


void SIM_Init(void) {

	/* Start USART1 DMA circular receive */
	HAL_UART_Receive_DMA(&huart1, s_dma_buf, SIM_RX_BUF);

	HAL_Delay(3000);            /* allow modem to boot */

    SIM_4GLTENetwork();

}

void SIM_GetSignal(void)
{
    extern SensorData_t g_sensor;
    char line[64];

    HAL_UART_Transmit(&huart1, "AT+CSQ\r\n", strlen("AT+CSQ\r\n"), 2000);

    uint32_t t = HAL_GetTick();
    while ((HAL_GetTick() - t) < 2000)
    {
        if (SIM_ReadLine(line, sizeof(line)))
        {
            /* +CSQ: <rssi>,<ber>   rssi 0-31, 99=unknown
               dBm = (rssi * 2) - 113                          */
            int rssi = 99, ber = 0;
            if (sscanf(line, "+CSQ: %d,%d", &rssi, &ber) == 2 && rssi != 99)
                g_sensor.signal_dbm = (int8_t)((rssi * 2) - 113);
            return;
        }
    }
}



SIM_Status SIM_4GLTENetwork(void)
{

	// Check modem alive
	SIM_Tx("AT\r\n");
	SIM_Wait("OK", 2000);

	//Check SIM status
	SIM_Tx("AT+CPIN?\r\n");
	SIM_Wait("OK", 3000);

	SIM_GetSignal();

	// Check registration (wait up to 30s)
    for (uint8_t retry = 0; retry < 30; retry++)
    {
        SIM_Tx("AT+CREG?\r\n");
        char line[128];
        uint32_t t = HAL_GetTick();
        while ((HAL_GetTick() - t) < 1500)
        {
            if (SIM_ReadLine(line, sizeof(line)))
            {
                if (!strstr(line, "+CREG: 0,1") &&
                    !strstr(line, "+CREG: 0,5"))
                    return SIM_ERROR;
            }
        }
        HAL_Delay(1000);
    }

    /* PDP context — set APN */
	SIM_Tx("AT+CGDCONT=1,\"IP\",\"" SIM_APN "\"\r\n");
	SIM_Wait("OK", 2000);

	/* Activate PDP */
	SIM_Tx("AT+CGACT=1,1\r\n");
	SIM_Wait("OK", 10000);

	SIM_CheckIP();

	return SIM_OK;
}



void SIM_Tx(const char *cmd)
{
    HAL_UART_Transmit(&huart1,
                      (const uint8_t *)cmd, strlen(cmd),
                      2000);
}


SIM_Status SIM_Wait(const char *expected, uint32_t timeout_ms)
{
    char line[256];
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout_ms)
    {
        if (SIM_ReadLine(line, sizeof(line)))
        {
            if (strstr(line, expected))
            {
                return SIM_OK;
            }
            if(strstr(line, "ERROR"))
            {
            	return SIM_ERROR;
            }

        }
    }
    return SIM_TIMEOUT;
}

SIM_Status SIM_CheckIP(void)
{
    char line[128];

    s_ip_addr[0] = '\0';
    SIM_Tx("AT+CIFSR\r\n");

    uint32_t t = HAL_GetTick();
    while ((HAL_GetTick() - t) < 5000)
    {
        if (!SIM_ReadLine(line, sizeof(line)))
            continue;

        /* Match either response variant */
        char *tag = strstr(line, "+CIFSR:");
        if (!tag) continue;

        /* Find quoted IP: first '"' after the colon */
        char *q1 = strchr(tag, '"');
        if (!q1) continue;
        q1++;                               /* skip opening quote          */

        /* Skip label field — find second quoted section (the IP) */
        char *q2 = strchr(q1, '"');         /* closing quote of label      */
        if (!q2) continue;
        q2++;                               /* skip comma after label      */
        if (*q2 == ',') q2++;
        if (*q2 == '"') q2++;               /* opening quote of IP         */

        char *q3 = strchr(q2, '"');         /* closing quote of IP         */
        if (!q3) continue;

        size_t ip_len = (size_t)(q3 - q2);
        if (ip_len == 0 || ip_len >= sizeof(s_ip_addr)) continue;

        memcpy(s_ip_addr, q2, ip_len);
        s_ip_addr[ip_len] = '\0';

        /* Reject 0.0.0.0 */
        if (strcmp(s_ip_addr, "0.0.0.0") == 0)
        {
            s_ip_addr[0] = '\0';
            continue;
        }

        return SIM_OK;    /* valid IP obtained */
    }

    return SIM_ERROR;       /* timed out or no valid IP */
}

const char *SIM_GetIP(void)
{
    return s_ip_addr;
}
