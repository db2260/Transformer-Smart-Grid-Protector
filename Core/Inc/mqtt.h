/*
 * mqtt.h
 *
 *  Created on: Apr 18, 2026
 *      Author: test1
 */

#ifndef INC_MQTT_H_
#define INC_MQTT_H_


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>


#define DEVICE_ID    		"TSGP1-001"
#define FIRMWARE_VERSION    "1.0.0"

#define AWS_ENDPOINT        "xxxxxxx.iot.ap-south-1.amazonaws.com"
#define AWS_PORT            8883

#define TLS_CA_CERT         "AmazonRootCA1.pem"
#define TLS_CLIENT_CERT     "TSGP-001.cert.pem"
#define TLS_CLIENT_KEY      "TSGP-001.private.key"

#define TOPIC_TELEMETRY     "tsgp1/TSGP1-001/telemetry"
#define TOPIC_FAULT         "tsgp1/TSGP1-001/fault"
#define TOPIC_STATUS        "tsgp1/TSGP1-001/status"
#define TOPIC_CMD_MCCB      "tsgp1/TSGP1-001/command/mccb"
#define TOPIC_ACK           "tsgp1/TSGP1-001/ack"

#define TELEMETRY_INTERVAL_MS   5000
#define STATUS_INTERVAL_MS      60000

#define MCCB_PULSE_MS           200

#define AT_CMD_BUF          256
#define AT_PAYLOAD_BUF      640

static uint32_t s_last_telemetry_ms = 0;
static uint32_t s_last_status_ms    = 0;

/* Receive state machine for +CMQTTRXSTART / +CMQTTRXPAYLOAD */
static enum {
    RX_IDLE,
    RX_WAITING_PAYLOAD,
} s_rx_state = RX_IDLE;

static char s_rx_payload[AT_PAYLOAD_BUF];


typedef struct {
    char    timestamp[32];      /* ISO-8601, e.g. "2026-04-01T10:30:00Z"   */

    float   voltage_R;          /* Phase voltages (V)                       */
    float   voltage_Y;
    float   voltage_B;

    float   current_R;          /* Phase currents (A)                       */
    float   current_Y;
    float   current_B;

    float   temperature_oil;    /* Oil temperature (°C)                     */

    bool    vibration_normal;
    bool    mccb_on;
    bool    fault_active;

    float   gps_lat;
    float   gps_lng;
    bool    gps_valid;          /* false = last known coords, no current fix */

    int8_t  signal_dbm;         /* Populated from AT+CSQ                    */
    bool    battery_backup;
} SensorData_t;

extern SensorData_t g_sensor;

/**
 * @brief  Initialise SIM7600G MQTT stack and connect to AWS IoT Core.
 *
 *         Call once after USART1 DMA is running and the SIM7600G has
 *         registered on the network (AT+CREG? = 0,1 or 0,5).
 *
 *         Certificate files must already be present in the modem filesystem.
 *         Upload them once with AT+CCERTDOWN (see sim7600g_provision.c).
 */
void MQTT_Init(void);


/**
 * @brief  Non-blocking periodic handler. Call every main loop iteration.
 *         Manages timed telemetry / status publishes and polls for commands.
 */
void MQTT_Process(void);


/**
 * @brief  Immediately publish a fault event with a sensor snapshot.
 */
void MQTT_PublishFault(uint8_t fault_code, const char *description);


/**
 * @brief  Poll USART1 DMA ring buffer for +CMQTTRXSTART / +CMQTTRXPAYLOAD.
 *
 *         SIM7600G delivers an incoming MQTT message as three URC lines:
 *           +CMQTTRXSTART: 0,<topic_len>,<payload_len>
 *           +CMQTTRXPAYLOAD: 0,<json_payload>
 *           +CMQTTRXEND: 0
 */
void MQTT_PollIncoming(void);


/**
 * @brief  Parse an MCCB RESTORE command JSON and execute if safe.
 *
 *         Validates:
 *           1. "command" == "RESTORE"
 *           2. "auth_token" present and non-empty
 *           3. All sensor readings within safe limits
 *
 *         Publishes:
 *           SUCCESS          — relay pulsed, power restored
 *           REJECTED_AUTH    — missing / invalid auth_token
 *           REJECTED_UNSAFE  — sensor readings out of safe range
 */
void MQTT_HandleCommand(const char *json_str);



/* ─── SIM7600G publish helper ────────────────────────────────────────────── */

/**
 * @brief  Publish a payload using the AT+CMQTTTOPIC / AT+CMQTTPAYLOAD /
 *         AT+CMQTTPUB three-step sequence required by the SIM7600G.
 *
 *         Step 1:  AT+CMQTTTOPIC=0,<topic_len>   → modem prompts ">"
 *         Step 2:  send raw topic bytes
 *         Step 3:  AT+CMQTTPAYLOAD=0,<payload_len> → modem prompts ">"
 *         Step 4:  send raw payload bytes
 *         Step 5:  AT+CMQTTPUB=0,1,60,0           → QoS1, retain=0
 */
static void sim_publish(const char *topic, const char *payload);


/* ─── Low-level AT transport ─────────────────────────────────────────────── */

static void sim_cmd(const char *cmd, uint32_t timeout_ms);


/**
 * @brief  Spin-poll USART1 lines until the expected token appears or timeout.
 *         Returns true on match, false on timeout.
 */
static bool sim_wait_response(const char *expected, uint32_t timeout_ms);


/* ─── JSON payload builders ──────────────────────────────────────────────── */

static void build_telemetry(char *buf, size_t len);


/*
 * NOTE: build_telemetry() constructs real JSON (no escaped quotes).
 * sim_publish() sends raw bytes — no extra escaping needed, unlike the
 * previous ESP8266 version which embedded JSON inside an AT string argument.
 */

static void build_fault(char *buf, size_t len,
                         uint8_t code, const char *desc);



static void build_status(char *buf, size_t len, bool online);



static void build_ack(char *buf, size_t len,
                       const char *result, const char *timestamp);


/* ─── MCCB safety and control ────────────────────────────────────────────── */

static bool mccb_safe_to_restore(void);


/**
 * @brief  Pulse PB0 HIGH for 500 ms — restores MCCB via shunt trip coil.
 *         Spec section 5 MCCB Trip Sequence:
 *           PB0 HIGH → PC817 optocoupler → BC547 → 12V relay → MCCB shunt coil.
 *         Used for remote RESTORE command. Same hardware path as fault trip.
 */
static void mccb_pulse(void);






#endif /* INC_MQTT_H_ */
