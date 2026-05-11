/*
 * mqtt.c
 *
 *  Created on: Apr 18, 2026
 *      Author: test1
 */

#include "main.h"
#include "sim7600g.h"
#include "mqtt.h"



void MQTT_Init(void)
{
    char cmd[AT_CMD_BUF];

    /* 1. Start the MQTT service */
    sim_cmd("AT+CMQTTSTART\r\n", 5000);

    /* 2. Acquire client (index 0, client-id = DEVICE_ID) */
    snprintf(cmd, sizeof(cmd),
             "AT+CMQTTACCQ=0,\"%s\",1\r\n", DEVICE_ID);
    sim_cmd(cmd, 3000);

    /* 3. Configure TLS — SSL context 0, verify server cert */
    snprintf(cmd, sizeof(cmd),
             "AT+CMQTTSSLCFG=0,0,\"%s\",\"%s\",\"%s\",1,1\r\n",
             TLS_CA_CERT, TLS_CLIENT_CERT, TLS_CLIENT_KEY);
    sim_cmd(cmd, 3000);

    /* 4. Connect to AWS IoT broker */
    snprintf(cmd, sizeof(cmd),
             "AT+CMQTTCONNECT=0,\"tcp://%s:%d\",60,1\r\n",
             AWS_ENDPOINT, AWS_PORT);
    sim_wait_response("+CMQTTCONNECT: 0,0", 15000);   /* 0,0 = success */

    /* 5. Subscribe to MCCB command topic (QoS 1) */
    snprintf(cmd, sizeof(cmd),
             "AT+CMQTTSUB=0,\"%s\",%zu,1\r\n",
             TOPIC_CMD_MCCB, strlen(TOPIC_CMD_MCCB));
    sim_cmd(cmd, 5000);

    /* 6. Publish initial online status */
    char payload[128];
    build_status(payload, sizeof(payload), true);
    sim_publish(TOPIC_STATUS, payload);

    s_last_telemetry_ms = HAL_GetTick();
    s_last_status_ms    = HAL_GetTick();
}


void MQTT_Process(void)
{
    uint32_t now = HAL_GetTick();
    char payload[AT_PAYLOAD_BUF];

    /* Telemetry every 5 s */
    if ((now - s_last_telemetry_ms) >= TELEMETRY_INTERVAL_MS)
    {
        s_last_telemetry_ms = now;
        build_telemetry(payload, sizeof(payload));
        sim_publish(TOPIC_TELEMETRY, payload);
    }

    /* Heartbeat every 60 s */
    if ((now - s_last_status_ms) >= STATUS_INTERVAL_MS)
    {
        s_last_status_ms = now;
        build_status(payload, sizeof(payload), true);
        sim_publish(TOPIC_STATUS, payload);
    }

    /* Poll USART1 for incoming MQTT messages */
    MQTT_PollIncoming();
}


void MQTT_PublishFault(uint8_t fault_code, const char *description)
{
    char payload[AT_PAYLOAD_BUF];
    build_fault(payload, sizeof(payload), fault_code, description);
    sim_publish(TOPIC_FAULT, payload);
}


void MQTT_PollIncoming(void)
{
    char line[AT_PAYLOAD_BUF];

    if (!SIM7600G_ReadLine(line, sizeof(line)))
        return;

    /* ── Detect start of incoming message ───────────────────────────── */
    if (strncmp(line, "+CMQTTRXSTART:", 14) == 0)
    {
        s_rx_state = RX_WAITING_PAYLOAD;
        s_rx_payload[0] = '\0';
        return;
    }

    /* ── Capture payload line ────────────────────────────────────────── */
    if (s_rx_state == RX_WAITING_PAYLOAD &&
        strncmp(line, "+CMQTTRXPAYLOAD:", 16) == 0)
    {
        /* Format: +CMQTTRXPAYLOAD: 0,<json> */
        char *comma = strchr(line, ',');
        if (comma)
        {
            strncpy(s_rx_payload, comma + 1, sizeof(s_rx_payload) - 1);
            s_rx_payload[sizeof(s_rx_payload) - 1] = '\0';
            /* Strip trailing \r\n */
            size_t l = strlen(s_rx_payload);
            while (l && (s_rx_payload[l-1] == '\r' || s_rx_payload[l-1] == '\n'))
                s_rx_payload[--l] = '\0';
        }
        return;
    }

    /* ── End of message — dispatch ───────────────────────────────────── */
    if (s_rx_state == RX_WAITING_PAYLOAD &&
        strncmp(line, "+CMQTTRXEND:", 12) == 0)
    {
        s_rx_state = RX_IDLE;
        if (s_rx_payload[0] != '\0')
            MQTT_HandleCommand(s_rx_payload);
        return;
    }
}


void MQTT_HandleCommand(const char *json_str)
{
    char command[16]     = {0};
    char auth_token[64]  = {0};
    char issued_by[32]   = {0};
    char timestamp[32]   = {0};

    /* Lightweight inline JSON field extractor (no heap, no malloc) */
    #define JSON_STR(key, dst, dsz) \
        do { \
            const char *_k = strstr(json_str, "\"" key "\""); \
            if (_k) { \
                _k = strchr(_k, ':'); \
                if (_k) { _k++; while (*_k == ' ') _k++; \
                    if (*_k == '"') { _k++; size_t _i = 0; \
                        while (*_k && *_k != '"' && _i < (dsz)-1) \
                            dst[_i++] = *_k++; \
                        dst[_i] = '\0'; } } } \
        } while (0)

    JSON_STR("command",    command,    sizeof(command));
    JSON_STR("auth_token", auth_token, sizeof(auth_token));
    JSON_STR("issued_by",  issued_by,  sizeof(issued_by));
    JSON_STR("timestamp",  timestamp,  sizeof(timestamp));

    char ack[256];

    /* Validate command type */
    if (strcmp(command, "RESTORE") != 0)
        return;

    /* ── Validate auth_token present and non-empty (spec section 6.3) ─ */
    if (auth_token[0] == '\0')
    {
        build_ack(ack, sizeof(ack), "REJECTED_AUTH", timestamp);
        sim_publish(TOPIC_ACK, ack);
        /* Log: timestamp, user role, result */
        FaultEngine_LogCommand(timestamp, issued_by, "REJECTED_AUTH");
        return;
    }

    /* ── Safety check: sensor readings within safe limits ────────────── */
    if (!mccb_safe_to_restore())
    {
        build_ack(ack, sizeof(ack), "REJECTED_UNSAFE", timestamp);
        sim_publish(TOPIC_ACK, ack);
        FaultEngine_LogCommand(timestamp, issued_by, "REJECTED_UNSAFE");
        return;
    }

    /* ── Execute RESTORE: pulse PB0 HIGH 500 ms (same as fault trip hw) */
    mccb_pulse();

    /* ── ACK within 10 seconds (spec requirement) ────────────────────── */
    build_ack(ack, sizeof(ack), "SUCCESS", timestamp);
    sim_publish(TOPIC_ACK, ack);
    FaultEngine_LogCommand(timestamp, issued_by, "SUCCESS");

    /* Clear MCCB trip state in device state */
    extern SemaphoreHandle_t xDevStateMutex;
    extern DeviceState_t g_dev_state;
    if (xSemaphoreTake(xDevStateMutex, pdMS_TO_TICKS(20)) == pdTRUE)
    {
        g_dev_state.mccb_tripped = false;
        xSemaphoreGive(xDevStateMutex);
    }
}


static void sim_publish(const char *topic, const char *payload)
{
    char cmd[AT_CMD_BUF];
    size_t topic_len   = strlen(topic);
    size_t payload_len = strlen(payload);

    /* Step 1 — set topic */
    snprintf(cmd, sizeof(cmd), "AT+CMQTTTOPIC=0,%zu\r\n", topic_len);
    sim_cmd(cmd, 2000);
    /* Wait for ">" prompt then send topic */
    sim_wait_response(">", 2000);
    HAL_UART_Transmit(&huart1, (const uint8_t *)topic, topic_len, 2000);

    /* Step 2 — set payload */
    snprintf(cmd, sizeof(cmd), "AT+CMQTTPAYLOAD=0,%zu\r\n", payload_len);
    sim_cmd(cmd, 2000);
    sim_wait_response(">", 2000);
    HAL_UART_Transmit(&huart1, (const uint8_t *)payload, payload_len, 3000);

    /* Step 3 — publish (QoS 1, retain 0, timeout 60 s) */
    sim_cmd("AT+CMQTTPUB=0,1,60,0\r\n", 5000);
}


static void sim_cmd(const char *cmd, uint32_t timeout_ms)
{
    HAL_UART_Transmit(&huart1,
                      (const uint8_t *)cmd, strlen(cmd),
                      timeout_ms);
}


static bool sim_wait_response(const char *expected, uint32_t timeout_ms)
{
    char line[AT_PAYLOAD_BUF];
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout_ms)
    {
        if (SIM7600G_ReadLine(line, sizeof(line)))
        {
            if (strstr(line, expected))
                return true;
        }
    }
    return false;
}


static void build_telemetry(char *buf, size_t len)
{
    snprintf(buf, len,
        "{"
        "\"device_id\":\"%s\","
        "\"timestamp\":\"%s\","
        "\"voltage\":{\"phase_R\":%.1f,\"phase_Y\":%.1f,\"phase_B\":%.1f},"
        "\"current\":{\"phase_R\":%.1f,\"phase_Y\":%.1f,\"phase_B\":%.1f},"
        "\"temperature_oil\":%.1f,"
        "\"vibration_status\":\"%s\","
        "\"mccb_status\":\"%s\","
        "\"fault\":{\"active\":%s,\"type\":null,\"code\":null},"
        "\"gps\":{\"lat\":%.4f,\"lng\":%.4f,\"valid\":%s},"
        "\"signal_strength_dbm\":%d,"
        "\"battery_backup_active\":%s,"
        "\"firmware_version\":\"%s\""
        "}",
        DEVICE_ID,
        g_sensor.timestamp,
        g_sensor.voltage_R, g_sensor.voltage_Y, g_sensor.voltage_B,
        g_sensor.current_R, g_sensor.current_Y, g_sensor.current_B,
        g_sensor.temperature_oil,
        g_sensor.vibration_normal ? "normal" : "abnormal",
        g_sensor.mccb_on         ? "ON"     : "OFF",
        g_sensor.fault_active    ? "true"   : "false",
        g_sensor.gps_lat, g_sensor.gps_lng,
        g_sensor.gps_valid ? "true" : "false",
        g_sensor.signal_dbm,
        g_sensor.battery_backup  ? "true"   : "false",
        FIRMWARE_VERSION
    );
}


static void build_fault(char *buf, size_t len,
                         uint8_t code, const char *desc)
{
    snprintf(buf, len,
        "{"
        "\"device_id\":\"%s\","
        "\"fault_code\":%u,"
        "\"description\":\"%s\","
        "\"voltage_R\":%.1f,"
        "\"current_R\":%.1f,"
        "\"temperature_oil\":%.1f"
        "}",
        DEVICE_ID, code, desc,
        g_sensor.voltage_R,
        g_sensor.current_R,
        g_sensor.temperature_oil
    );
}

static void build_status(char *buf, size_t len, bool online)
{
    snprintf(buf, len,
        "{\"device_id\":\"%s\",\"status\":\"%s\"}",
        DEVICE_ID,
        online ? "online" : "offline"
    );
}

static void build_ack(char *buf, size_t len,
                       const char *result, const char *timestamp)
{
    snprintf(buf, len,
        "{"
        "\"device_id\":\"%s\","
        "\"command\":\"RESTORE\","
        "\"result\":\"%s\","
        "\"timestamp\":\"%s\""
        "}",
        DEVICE_ID, result, timestamp
    );
}


static bool mccb_safe_to_restore(void)
{
    /*
     * Safety thresholds mirror spec section 5 fault table.
     * Restore is only allowed when all readings are within normal range.
     */

    /* Voltage: 180–260 V per phase (F01 overvoltage / F02 undervoltage) */
    if (g_sensor.voltage_R < 180.0f || g_sensor.voltage_R > 260.0f) return false;
    if (g_sensor.voltage_Y < 180.0f || g_sensor.voltage_Y > 260.0f) return false;
    if (g_sensor.voltage_B < 180.0f || g_sensor.voltage_B > 260.0f) return false;

    /* Current: < 90 A per phase (F03 overload threshold) */
    if (g_sensor.current_R > 90.0f) return false;
    if (g_sensor.current_Y > 90.0f) return false;
    if (g_sensor.current_B > 90.0f) return false;

    /* Oil temperature: < 85 °C (F06 threshold) */
    if (g_sensor.temperature_oil > 85.0f) return false;

    /* No active fault of any kind */
    if (g_sensor.fault_active) return false;

    return true;
}


static void mccb_pulse(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);     /* HIGH = activate */
    HAL_Delay(MCCB_PULSE_MS);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);   /* release         */
}
