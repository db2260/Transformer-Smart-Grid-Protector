/*
 * tasks.h
 *
 *  Created on: Apr 25, 2026
 *      Author: test1
 */

#ifndef INC_TASKS_H_
#define INC_TASKS_H_



#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <math.h>



typedef struct {
    bool    active;
    uint8_t code;
    bool    do_trip;    /* true = fault requires MCCB hardware trip */
} FaultState_t;


typedef struct {
    bool    gps_fix;
    bool    network_ok;
    bool    battery_mode;
    bool    mccb_tripped;
    bool    comm_loss;  /* F10: no MQTT ACK > 5 min — triggers LED blink */
} DeviceState_t;



extern SensorData_t     g_sensor;
extern FaultState_t     g_fault;
extern DeviceState_t    g_dev_state;

extern SemaphoreHandle_t xSensorMutex;
extern SemaphoreHandle_t xFaultMutex;
extern SemaphoreHandle_t xDevStateMutex;

extern TaskHandle_t hSensorRead;
extern TaskHandle_t hFaultEngine;
extern TaskHandle_t hMQTT_Publish;
extern TaskHandle_t hGPS_Update;
extern TaskHandle_t hMQTT_Subscribe;
extern TaskHandle_t hLED_Status;
extern TaskHandle_t hBatteryMonitor;
extern TaskHandle_t hWatchdog;



#define STACK_SENSOR        256
#define STACK_FAULT         256
#define STACK_MQTT_PUB      512
#define STACK_GPS           384
#define STACK_MQTT_SUB      384
#define STACK_LED           128
#define STACK_BATTERY       192
#define STACK_WATCHDOG      128



#define PERIOD_SENSOR_MS        100
#define PERIOD_FAULT_MS         200
#define PERIOD_MQTT_PUB_MS      5000
#define PERIOD_GPS_MS           30000
#define PERIOD_LED_MS           500
#define PERIOD_BATTERY_MS       10000
#define PERIOD_WATCHDOG_MS      1000


/* If a task doesn't check in within this many watchdog ticks, force reboot  */
#define WDG_HANG_TICKS          30      /* 30 × 1 s = 30 seconds            */



/* ─── Watchdog checkin table ─────────────────────────────────────────────── */
/*
 * Each task writes its index into this array every cycle.
 * Task_Watchdog checks that no entry has been stale for > WDG_HANG_TICKS.
 */
typedef struct {
    const char *name;
    uint32_t    last_checkin;   /* tick count at last task checkin          */
    uint32_t    max_period_ms;  /* expected maximum period                  */
} WdgEntry_t;

#define WDG_IDX_SENSOR      0
#define WDG_IDX_FAULT       1
#define WDG_IDX_MQTT_PUB    2
#define WDG_IDX_GPS         3
#define WDG_IDX_MQTT_SUB    4
#define WDG_IDX_LED         5
#define WDG_IDX_BATTERY     6
#define WDG_NUM_TASKS       7

static WdgEntry_t s_wdg_table[WDG_NUM_TASKS] = {
    [WDG_IDX_SENSOR]   = { "SensorRead",     0, PERIOD_SENSOR_MS   * 3 },
    [WDG_IDX_FAULT]    = { "FaultEngine",    0, PERIOD_FAULT_MS    * 3 },
    [WDG_IDX_MQTT_PUB] = { "MQTT_Publish",   0, PERIOD_MQTT_PUB_MS + 2000 },
    [WDG_IDX_GPS]      = { "GPS_Update",     0, PERIOD_GPS_MS      + 5000 },
    [WDG_IDX_MQTT_SUB] = { "MQTT_Subscribe", 0, 3000  },
    [WDG_IDX_LED]      = { "LED_Status",     0, PERIOD_LED_MS      * 3 },
    [WDG_IDX_BATTERY]  = { "BatteryMonitor", 0, PERIOD_BATTERY_MS  + 2000 },
};



void Task_SensorRead    (void *arg);
void Task_FaultEngine   (void *arg);
void Task_MQTT_Publish  (void *arg);
void Task_GPS_Update    (void *arg);
void Task_MQTT_Subscribe(void *arg);
void Task_LED_Status    (void *arg);
void Task_BatteryMonitor(void *arg);
void Task_Watchdog      (void *arg);

/** Create all mutexes and tasks, then start the FreeRTOS scheduler. */
void RTOS_Init(void);

/* ─── FaultEngine utilities ──────────────────────────────────────────────── */

/**
 * Log a MCCB RESTORE command attempt (timestamp, issued_by role, result).
 * Called by MQTT_HandleCommand() after every RESTORE attempt.
 * Writes to circular log buffer + debug USART2.
 */
void FaultEngine_LogCommand(const char *timestamp,
                             const char *issued_by,
                             const char *result);

/**
 * Reset the F10 communication-loss timer.
 * Call after every successful MQTT publish ACK.
 */
void FaultEngine_MQTTAckReceived(void);

/* ─── Fault description helper (implement in sensors.c or fault_codes.c) ── */
const char *Fault_GetDescription(uint8_t code);


extern ADC_SensorData_t a2dsensor;


/* ═══════════════════════════════════════════════════════════════════════════
 * Task_FaultEngine — Priority HIGH, 200 ms
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * Fault table (from spec section 5):
 *
 *  Code  Name                Trigger                         Action
 *  F01   Overvoltage         Any phase > 260 V               Trip MCCB + alert
 *  F02   Undervoltage        Any phase < 180 V               Trip MCCB + alert
 *  F03   Overload Current    Any phase current > 90 A        Trip MCCB + alert
 *  F04   Short Circuit       Any phase current > 150 A       INSTANT Trip + alert
 *  F05   Phase Imbalance     Imbalance > 15%                 Trip MCCB + alert
 *  F06   Oil Overtemperature Oil temp > 85 °C                Trip MCCB + alert
 *  F07   Vibration/Tamper    MPU6050 shock/tilt detected     Alert only (no trip)
 *  F08   Unexpected Trip     MCCB feedback mismatch          Alert only
 *  F09   Power Failure       Main power loss detected        Switch battery + alert
 *  F10   Communication Loss  No MQTT ACK for > 5 min         Log + LED blink
 *
 * MCCB Trip Sequence (hardware, section 5 spec):
 *  1. Fault confirmed by firmware rule engine
 *  2. Set PB0 (TRIP_OUT) HIGH for 500 ms pulse      ← HIGH, not LOW
 *  3. PC817 optocoupler → BC547 transistor drives relay coil
 *  4. 12V relay closes → shunt trip coil → MCCB opens
 *  5. Set PB0 LOW after 500 ms
 *  6. Set LED TRIP ON, log fault with timestamp
 *  7. Publish fault payload to AWS IoT Core immediately
 *
 * F04 Short Circuit is INSTANT — no 200 ms debounce, executes in-line.
 * F07, F08, F09, F10 do NOT trip the MCCB.
 *
 * Phase imbalance calculation (F05):
 *   avg  = (I_R + I_Y + I_B) / 3
 *   max_dev = max(|I_R-avg|, |I_Y-avg|, |I_B-avg|)
 *   imbalance% = (max_dev / avg) * 100  →  trip if > 15%
 *
 * MCCB feedback (F08):
 *   MCCB_FB_PIN (PA7) reads the auxiliary contact.
 *   If mccb_on (expected ON) but FB=LOW (actually OFF) → unexpected trip.
 *
 * Command log (section 6.3):
 *   Every RESTORE attempt is logged to USART2 with timestamp, role, result.
 */

/* ─── Fault code definitions (spec section 5) ───────────────────────────── */

#define FAULT_F01_OVERVOLTAGE       0x01U
#define FAULT_F02_UNDERVOLTAGE      0x02U
#define FAULT_F03_OVERLOAD          0x03U
#define FAULT_F04_SHORT_CIRCUIT     0x04U
#define FAULT_F05_PHASE_IMBALANCE   0x05U
#define FAULT_F06_OIL_OVERTEMP      0x06U
#define FAULT_F07_VIBRATION         0x07U
#define FAULT_F08_UNEXPECTED_TRIP   0x08U
#define FAULT_F09_POWER_FAILURE     0x09U
#define FAULT_F10_COMM_LOSS         0x0AU

/* ─── Thresholds (exact from spec) ──────────────────────────────────────── */

#define THR_V_OVER_V            260.0f   /* F01: any phase > 260 V          */
#define THR_V_UNDER_V           180.0f   /* F02: any phase < 180 V          */
#define THR_I_OVERLOAD_A         90.0f   /* F03: any phase current > 90 A   */
#define THR_I_SHORT_A           150.0f   /* F04: any phase current > 150 A  */
#define THR_PHASE_IMBAL_PCT      15.0f   /* F05: imbalance > 15%            */
#define THR_OIL_TEMP_C           85.0f   /* F06: oil temp > 85 °C           */
#define THR_COMM_LOSS_MS     300000UL    /* F10: no MQTT ACK for > 5 min    */

/* ─── Hardware pins (from spec hardware section) ─────────────────────────── */

#define TRIP_OUT_PORT       GPIOB
#define TRIP_OUT_PIN        GPIO_PIN_0   /* PB0: TRIP_OUT, active HIGH       */
#define MCCB_FB_PORT        GPIOA
#define MCCB_FB_PIN         GPIO_PIN_7   /* PA7: MCCB auxiliary contact      */
#define TRIP_PULSE_MS       500U         /* spec: 500 ms pulse               */

/* ─── Command attempt log ────────────────────────────────────────────────── */
/*
 * Circular log of the last CMDLOG_SIZE RESTORE attempts.
 * Written by MQTT_HandleCommand(), read by diagnostic tools.
 */
#define CMDLOG_SIZE     8U

typedef struct {
    char    timestamp[32];
    char    issued_by[32];
    char    result[20];     /* "SUCCESS" / "REJECTED_AUTH" / "REJECTED_UNSAFE" */
} CmdLogEntry_t;

static CmdLogEntry_t s_cmd_log[CMDLOG_SIZE];
static uint8_t       s_cmd_log_idx = 0;



#endif /* INC_TASKS_H_ */
