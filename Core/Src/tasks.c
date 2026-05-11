/*
 * tasks.c
 *
 *  Created on: Apr 25, 2026
 *      Author: test1
 */




#include "main.h"
#include "tasks.h"



SensorData_t    g_sensor    = {0};
FaultState_t    g_fault     = {0};
DeviceState_t   g_dev_state = {0};


SemaphoreHandle_t xSensorMutex   = NULL;
SemaphoreHandle_t xFaultMutex    = NULL;
SemaphoreHandle_t xDevStateMutex = NULL;


TaskHandle_t hSensorRead     = NULL;
TaskHandle_t hFaultEngine    = NULL;
TaskHandle_t hMQTT_Publish   = NULL;
TaskHandle_t hGPS_Update     = NULL;
TaskHandle_t hMQTT_Subscribe = NULL;
TaskHandle_t hLED_Status     = NULL;
TaskHandle_t hBatteryMonitor = NULL;
TaskHandle_t hWatchdog       = NULL;

ADC_SensorData_t a2dsensor = NULL;



static inline void wdg_checkin(uint8_t idx)
{
    s_wdg_table[idx].last_checkin = xTaskGetTickCount();
}


/* ═══════════════════════════════════════════════════════════════════════════
 * Task_SensorRead — Priority HIGH, 100 ms
 * ═══════════════════════════════════════════════════════════════════════════ */

void Task_SensorRead(void *arg)
{
    (void)arg;
    TickType_t xLastWake = xTaskGetTickCount();

    for (;;)
    {
    	ADC_ProcessAll(&a2dsensor);

        /* ── Write to shared struct under mutex ──────────────────────── */
        if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            g_sensor.voltage_R = a2dsensor.voltage_R;
            g_sensor.voltage_Y = a2dsensor.voltage_Y;
            g_sensor.voltage_B = a2dsensor.voltage_B;
            g_sensor.current_R = a2dsensor.current_R;
            g_sensor.current_Y = a2dsensor.current_Y;
            g_sensor.current_B = a2dsensor.current_B;
            g_sensor.temperature_oil = a2dsensor.temperature_oil;
            /* timestamp populated by RTC/SNTP — set in your RTC task or here */
            xSemaphoreGive(xSensorMutex);
        }

        wdg_checkin(WDG_IDX_SENSOR);
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(PERIOD_SENSOR_MS));
    }
}


void FaultEngine_LogCommand(const char *timestamp,
                             const char *issued_by,
                             const char *result)
{
    CmdLogEntry_t *e = &s_cmd_log[s_cmd_log_idx % CMDLOG_SIZE];
    strncpy(e->timestamp, timestamp, sizeof(e->timestamp) - 1);
    strncpy(e->issued_by, issued_by,  sizeof(e->issued_by)  - 1);
    strncpy(e->result,    result,     sizeof(e->result)     - 1);
    s_cmd_log_idx++;

    /* Also emit to debug UART (USART2) */
    char msg[96];
    snprintf(msg, sizeof(msg),
             "[CMD] ts=%s by=%s result=%s\r\n",
             timestamp, issued_by, result);
    HAL_UART_Transmit(&huart2, (const uint8_t *)msg, strlen(msg), 50);
}

/* ─── MCCB trip sequence (spec hardware section) ────────────────────────── */
/*
 * Called ONLY for faults that require a trip (F01–F06).
 * NOT called for F07, F08, F09, F10.
 *
 * Runs in Task_FaultEngine context. The 500 ms vTaskDelay is acceptable here
 * because Task_FaultEngine is HIGH priority — lower-priority tasks run during
 * the delay; higher-priority tasks (Watchdog) still preempt normally.
 */
static void mccb_trip_sequence(uint8_t fault_code)
{
    /* Step 2: Set PB0 HIGH — pulse TRIP_OUT */
    HAL_GPIO_WritePin(TRIP_OUT_PORT, TRIP_OUT_PIN, GPIO_PIN_SET);

    /* Step 3–4: hardware handles optocoupler → transistor → relay → MCCB */

    /* Step 5: hold for 500 ms then release */
    vTaskDelay(pdMS_TO_TICKS(TRIP_PULSE_MS));
    HAL_GPIO_WritePin(TRIP_OUT_PORT, TRIP_OUT_PIN, GPIO_PIN_RESET);

    /* Step 6: set TRIP LED and update device state */
    HAL_GPIO_WritePin(LED_TRIP_PORT, LED_TRIP_PIN, GPIO_PIN_SET);

    if (xSemaphoreTake(xDevStateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        g_dev_state.mccb_tripped = true;
        xSemaphoreGive(xDevStateMutex);
    }

    /* Step 7: notify MQTT_Publish task — bit 0 = fault event */
    xTaskNotify(hMQTT_Publish, 0x01UL, eSetBits);

    /* Log to debug UART */
    char msg[64];
    snprintf(msg, sizeof(msg),
             "[FAULT] MCCB TRIPPED F%02X ts=%lu\r\n",
             fault_code, (unsigned long)HAL_GetTick());
    HAL_UART_Transmit(&huart2, (const uint8_t *)msg, strlen(msg), 50);
}

/* ─── Phase imbalance helper ─────────────────────────────────────────────── */

static float phase_imbalance_pct(float i_R, float i_Y, float i_B)
{
    float avg = (i_R + i_Y + i_B) / 3.0f;
    if (avg < 0.5f) return 0.0f;    /* avoid div/0 at near-zero current    */

    float d_R = fabsf(i_R - avg);
    float d_Y = fabsf(i_Y - avg);
    float d_B = fabsf(i_B - avg);

    float max_dev = d_R;
    if (d_Y > max_dev) max_dev = d_Y;
    if (d_B > max_dev) max_dev = d_B;

    return (max_dev / avg) * 100.0f;
}

/* ─── Fault description strings ──────────────────────────────────────────── */

const char *Fault_GetDescription(uint8_t code)
{
    switch (code)
    {
        case FAULT_F01_OVERVOLTAGE:     return "F01 Overvoltage >260V";
        case FAULT_F02_UNDERVOLTAGE:    return "F02 Undervoltage <180V";
        case FAULT_F03_OVERLOAD:        return "F03 Overload Current >90A";
        case FAULT_F04_SHORT_CIRCUIT:   return "F04 Short Circuit >150A";
        case FAULT_F05_PHASE_IMBALANCE: return "F05 Phase Imbalance >15%";
        case FAULT_F06_OIL_OVERTEMP:    return "F06 Oil Overtemperature >85C";
        case FAULT_F07_VIBRATION:       return "F07 Vibration/Tamper";
        case FAULT_F08_UNEXPECTED_TRIP: return "F08 Unexpected MCCB Trip";
        case FAULT_F09_POWER_FAILURE:   return "F09 Power Failure";
        case FAULT_F10_COMM_LOSS:       return "F10 Communication Loss";
        default:                        return "Unknown Fault";
    }
}

/* ─── Communication loss tracker (F10) ──────────────────────────────────── */
/*
 * Task_MQTT_Publish calls FaultEngine_MQTTAckReceived() each time a
 * successful publish completes. FaultEngine checks elapsed time each cycle.
 */
static uint32_t s_last_mqtt_ack_ms = 0;

void FaultEngine_MQTTAckReceived(void)
{
    s_last_mqtt_ack_ms = HAL_GetTick();
}

/* ═══════════════════════════════════════════════════════════════════════════ */

void Task_FaultEngine(void *arg)
{
    (void)arg;
    TickType_t xLastWake = xTaskGetTickCount();

    /* Initialise MQTT ACK timer so F10 doesn't fire at boot */
    s_last_mqtt_ack_ms = HAL_GetTick();

    for (;;)
    {
        /* ── Snapshot sensor data ────────────────────────────────────── */
        SensorData_t snap;
        if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            snap = g_sensor;
            xSemaphoreGive(xSensorMutex);
        }
        else
        {
            wdg_checkin(WDG_IDX_FAULT);
            vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(PERIOD_FAULT_MS));
            continue;
        }

        uint8_t new_code   = 0;
        bool    new_active = false;
        bool    do_trip    = false;   /* true = hardware MCCB trip required  */

        /* ══════════════════════════════════════════════════════════════
         * F04 — Short Circuit  (> 150 A): INSTANT TRIP, highest priority
         * Check this first so it cannot be masked by F03.
         * ══════════════════════════════════════════════════════════════ */
        if (snap.current_R > THR_I_SHORT_A ||
            snap.current_Y > THR_I_SHORT_A ||
            snap.current_B > THR_I_SHORT_A)
        {
            new_code   = FAULT_F04_SHORT_CIRCUIT;
            new_active = true;
            do_trip    = true;
            goto fault_commit;      /* skip remaining checks — act immediately */
        }

        /* ══════════════════════════════════════════════════════════════
         * F01 — Overvoltage  (any phase > 260 V)
         * ══════════════════════════════════════════════════════════════ */
        if (snap.voltage_R > THR_V_OVER_V ||
            snap.voltage_Y > THR_V_OVER_V ||
            snap.voltage_B > THR_V_OVER_V)
        {
            new_code = FAULT_F01_OVERVOLTAGE; new_active = true; do_trip = true;
            goto fault_commit;
        }

        /* ══════════════════════════════════════════════════════════════
         * F02 — Undervoltage  (any phase < 180 V)
         * ══════════════════════════════════════════════════════════════ */
        if (snap.voltage_R < THR_V_UNDER_V ||
            snap.voltage_Y < THR_V_UNDER_V ||
            snap.voltage_B < THR_V_UNDER_V)
        {
            new_code = FAULT_F02_UNDERVOLTAGE; new_active = true; do_trip = true;
            goto fault_commit;
        }

        /* ══════════════════════════════════════════════════════════════
         * F03 — Overload Current  (any phase > 90 A)
         * ══════════════════════════════════════════════════════════════ */
        if (snap.current_R > THR_I_OVERLOAD_A ||
            snap.current_Y > THR_I_OVERLOAD_A ||
            snap.current_B > THR_I_OVERLOAD_A)
        {
            new_code = FAULT_F03_OVERLOAD; new_active = true; do_trip = true;
            goto fault_commit;
        }

        /* ══════════════════════════════════════════════════════════════
         * F05 — Phase Imbalance  (imbalance > 15%)
         * ══════════════════════════════════════════════════════════════ */
        {
            float imbal = phase_imbalance_pct(snap.current_R,
                                               snap.current_Y,
                                               snap.current_B);
            if (imbal > THR_PHASE_IMBAL_PCT)
            {
                new_code = FAULT_F05_PHASE_IMBALANCE;
                new_active = true; do_trip = true;
                goto fault_commit;
            }
        }

        /* ══════════════════════════════════════════════════════════════
         * F06 — Oil Overtemperature  (oil temp > 85 °C)
         * ══════════════════════════════════════════════════════════════ */
        if (snap.temperature_oil > THR_OIL_TEMP_C)
        {
            new_code = FAULT_F06_OIL_OVERTEMP; new_active = true; do_trip = true;
            goto fault_commit;
        }

        /* ══════════════════════════════════════════════════════════════
         * F07 — Vibration / Tamper  (MPU6050 shock/tilt) — ALERT ONLY
         * vibration_normal is written false by MPU6050 read in SensorRead task.
         * ══════════════════════════════════════════════════════════════ */
        if (!snap.vibration_normal)
        {
            new_code = FAULT_F07_VIBRATION; new_active = true; do_trip = false;
            goto fault_commit;
        }

        /* ══════════════════════════════════════════════════════════════
         * F08 — Unexpected MCCB Trip  (feedback mismatch) — ALERT ONLY
         * MCCB_FB_PIN LOW while mccb_on=true means MCCB opened unexpectedly.
         * ══════════════════════════════════════════════════════════════ */
        if (snap.mccb_on &&
            HAL_GPIO_ReadPin(MCCB_FB_PORT, MCCB_FB_PIN) == GPIO_PIN_RESET)
        {
            new_code = FAULT_F08_UNEXPECTED_TRIP; new_active = true; do_trip = false;
            goto fault_commit;
        }

        /* ══════════════════════════════════════════════════════════════
         * F10 — Communication Loss  (no MQTT ACK for > 5 min) — LOG + LED
         * ══════════════════════════════════════════════════════════════ */
        if ((HAL_GetTick() - s_last_mqtt_ack_ms) > THR_COMM_LOSS_MS)
        {
            new_code = FAULT_F10_COMM_LOSS; new_active = true; do_trip = false;
            goto fault_commit;
        }

        /* No fault detected this cycle */
        new_active = false;
        new_code   = 0;

fault_commit:
        /* ── Update shared fault state ───────────────────────────────── */
        {
            bool newly_active = false;

            if (xSemaphoreTake(xFaultMutex, pdMS_TO_TICKS(10)) == pdTRUE)
            {
                /* Rising edge detection — only act on NEW faults */
                if (new_active && !g_fault.active)
                    newly_active = true;

                g_fault.active   = new_active;
                g_fault.code     = new_code;
                g_fault.do_trip  = do_trip;
                xSemaphoreGive(xFaultMutex);
            }

            /* Mirror fault flag into sensor struct for telemetry payload */
            if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(10)) == pdTRUE)
            {
                g_sensor.fault_active = new_active;
                xSemaphoreGive(xSensorMutex);
            }

            /* ── Execute trip sequence on rising edge (trip faults only) */
            if (newly_active && do_trip)
            {
                mccb_trip_sequence(new_code);
                /* mccb_trip_sequence() already notifies MQTT_Publish */
            }
            else if (newly_active && !do_trip)
            {
                /*
                 * Alert-only faults (F07, F08, F10):
                 * Notify MQTT_Publish for immediate cloud alert.
                 * No relay pulse.
                 */
                xTaskNotify(hMQTT_Publish, 0x01UL, eSetBits);

                /* F10 specific: blink network LED (handled in LED task via
                   g_dev_state.comm_loss flag) */
                if (new_code == FAULT_F10_COMM_LOSS)
                {
                    if (xSemaphoreTake(xDevStateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
                    {
                        g_dev_state.comm_loss = true;
                        xSemaphoreGive(xDevStateMutex);
                    }
                }
            }

            /* Clear comm_loss flag once connectivity restores */
            if (!new_active || new_code != FAULT_F10_COMM_LOSS)
            {
                if (xSemaphoreTake(xDevStateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
                {
                    g_dev_state.comm_loss = false;
                    xSemaphoreGive(xDevStateMutex);
                }
            }
        }

        wdg_checkin(WDG_IDX_FAULT);
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(PERIOD_FAULT_MS));
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Task_MQTT_Publish — Priority MEDIUM, 5 s (+ immediate on fault notify)
 * ═══════════════════════════════════════════════════════════════════════════ */

void Task_MQTT_Publish(void *arg)
{
    (void)arg;
    TickType_t xLastWake = xTaskGetTickCount();

    for (;;)
    {
        /* Wait up to 5 s OR wake immediately on fault notification */
        uint32_t notif = 0;
        xTaskNotifyWait(0, 0xFFFFFFFF, &notif,
                        pdMS_TO_TICKS(PERIOD_MQTT_PUB_MS));

        bool is_fault_event = (notif & 0x01) != 0;

        /* Take sensor snapshot */
        SensorData_t snap;
        FaultState_t fsnap;

        if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(50)) == pdTRUE)
        { snap = g_sensor; xSemaphoreGive(xSensorMutex); }

        if (xSemaphoreTake(xFaultMutex, pdMS_TO_TICKS(50)) == pdTRUE)
        { fsnap = g_fault; xSemaphoreGive(xFaultMutex); }

        if (is_fault_event && fsnap.active)
        {
            /* Publish to fault topic immediately */
            MQTT_PublishFault(fsnap.code, Fault_GetDescription(fsnap.code));
        }
        else
        {
            /* Regular 5 s telemetry */
            MQTT_Process();
        }

        /* Update network LED */
        if (xSemaphoreTake(xDevStateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            g_dev_state.network_ok = (SIM7600G_GetIP()[0] != '\0');
            xSemaphoreGive(xDevStateMutex);
        }

        wdg_checkin(WDG_IDX_MQTT_PUB);
        xLastWake = xTaskGetTickCount();    /* re-anchor after notify wake */
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Task_GPS_Update — Priority LOW, 30 s
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * SIM7600G AT+CGPSINFO exact response format (from spec):
 *   +CGPSINFO: 1619.174,N,8106.748,E,010426,103000.0,50,1.5,180
 *              ^lat      ^NS ^lng    ^EW ^date  ^time  ^alt ^spd ^crs
 *
 * Fields (all comma-separated, no spaces after colon):
 *   [0] raw_lat  — ddmm.mmm   e.g. 1619.174  → 16°19.174' → 16.3196°
 *   [1] NS       — 'N' or 'S'
 *   [2] raw_lng  — dddmm.mmm  e.g. 8106.748  → 81°06.748' → 81.1125°
 *   [3] EW       — 'E' or 'W'
 *   [4] date     — DDMMYY
 *   [5] time     — HHMMSS.s UTC
 *   [6] altitude — metres
 *   [7] speed    — km/h
 *   [8] course   — degrees
 *
 * No-fix response: +CGPSINFO: ,,,,,,,,
 *   All fields are empty — detect by checking raw_lat == 0.
 *
 * Policy (from spec):
 *   - On fix:    update lat/lng/alt/speed, set gps_valid = true
 *   - No fix:    keep LAST KNOWN lat/lng, set gps_valid = false
 *   - Telemetry: always include lat/lng + gps_valid flag
 */

/**
 * @brief  Parse one +CGPSINFO response line into GPS fields.
 *
 * @param  line      Raw line from SIM7600G (after "+CGPSINFO: ")
 * @param  lat       Output decimal degrees latitude
 * @param  lng       Output decimal degrees longitude
 * @param  alt_m     Output altitude in metres
 * @param  speed_kmh Output speed in km/h
 * @return true if a valid fix was parsed, false if no-fix or parse error
 */
static bool parse_cgpsinfo(const char *line,
                            float *lat, float *lng,
                            float *alt_m, float *speed_kmh)
{
    /*
     * Find the data after "+CGPSINFO:" — skip the tag and any leading space.
     * The SIM7600G always includes the colon; firmware spec shows no space
     * but we handle both "+CGPSINFO: data" and "+CGPSINFO:data".
     */
    const char *tag = strstr(line, "+CGPSINFO:");
    if (!tag) return false;

    const char *p = tag + 10;           /* skip "+CGPSINFO:" (10 chars)    */
    while (*p == ' ') p++;              /* skip optional space              */

    /* No-fix: first char is a comma → all fields empty */
    if (*p == ',' || *p == '\0') return false;

    /*
     * Parse 9 comma-separated fields.
     * sscanf stops at commas for %f/%c, so we advance manually field by field.
     */
    float raw_lat = 0.0f, raw_lng = 0.0f;
    char  ns = 'N', ew = 'E';
    char  date[8] = {0}, time_s[12] = {0};
    float alt = 0.0f, spd = 0.0f, crs = 0.0f;

    /* Field 0: raw latitude (ddmm.mmm) */
    if (sscanf(p, "%f", &raw_lat) != 1 || raw_lat == 0.0f) return false;
    p = strchr(p, ','); if (!p) return false; p++;

    /* Field 1: N/S */
    ns = *p;
    p = strchr(p, ','); if (!p) return false; p++;

    /* Field 2: raw longitude (dddmm.mmm) */
    if (sscanf(p, "%f", &raw_lng) != 1) return false;
    p = strchr(p, ','); if (!p) return false; p++;

    /* Field 3: E/W */
    ew = *p;
    p = strchr(p, ','); if (!p) return false; p++;

    /* Field 4: date (DDMMYY) — store as string, skip for now */
    sscanf(p, "%7s", date);
    p = strchr(p, ','); if (!p) return false; p++;

    /* Field 5: UTC time (HHMMSS.s) */
    sscanf(p, "%11s", time_s);
    p = strchr(p, ','); if (!p) return false; p++;

    /* Field 6: altitude (metres) */
    sscanf(p, "%f", &alt);
    p = strchr(p, ','); if (!p) return false; p++;

    /* Field 7: speed (km/h) */
    sscanf(p, "%f", &spd);
    p = strchr(p, ','); if (!p) return false; p++;

    /* Field 8: course (degrees) */
    sscanf(p, "%f", &crs);

    /* ── Convert ddmm.mmm → decimal degrees ──────────────────────────── */
    /*
     * Example from spec: 1619.174
     *   degrees = (int)(1619.174 / 100) = 16
     *   minutes = 1619.174 - 1600       = 19.174
     *   decimal = 16 + 19.174/60        = 16.31957°
     *
     * Example: 8106.748
     *   degrees = 81
     *   minutes = 6.748
     *   decimal = 81 + 6.748/60         = 81.11247°
     */
    int   lat_deg = (int)(raw_lat / 100.0f);
    float lat_min = raw_lat - (float)(lat_deg * 100);
    *lat = (float)lat_deg + lat_min / 60.0f;
    if (ns == 'S') *lat = -(*lat);

    int   lng_deg = (int)(raw_lng / 100.0f);
    float lng_min = raw_lng - (float)(lng_deg * 100);
    *lng = (float)lng_deg + lng_min / 60.0f;
    if (ew == 'W') *lng = -(*lng);

    *alt_m     = alt;
    *speed_kmh = spd;

    return true;
}

void Task_GPS_Update(void *arg)
{
    (void)arg;
    TickType_t xLastWake = xTaskGetTickCount();

    char line[128];

    /*
     * AT+CGPS=1,1 — start GPS engine in standalone mode.
     * Do this ONCE here before the loop.
     * If GPS was already running (e.g. warm boot), the modem returns ERROR
     * which we ignore — parse_cgpsinfo() will still work fine.
     */
    HAL_UART_Transmit(&huart1,
                      (const uint8_t *)"AT+CGPS=1,1\r\n", 13, 1000);
    vTaskDelay(pdMS_TO_TICKS(2000));    /* give GPS engine time to start    */

    for (;;)
    {
        float lat = 0.0f, lng = 0.0f, alt = 0.0f, spd = 0.0f;
        bool  got_fix = false;

        /* ── Send AT+CGPSINFO ────────────────────────────────────────── */
        HAL_UART_Transmit(&huart1,
                          (const uint8_t *)"AT+CGPSINFO\r\n", 13, 1000);

        /*
         * Wait up to 3 s for the +CGPSINFO: response line.
         * The modem replies immediately — 3 s is generous; adjust down if
         * you need tighter 30 s timing.
         */
        uint32_t t = HAL_GetTick();
        while ((HAL_GetTick() - t) < 3000 && !got_fix)
        {
            if (!SIM7600G_ReadLine(line, sizeof(line))) continue;

            got_fix = parse_cgpsinfo(line, &lat, &lng, &alt, &spd);
        }

        /* ── Update shared state ─────────────────────────────────────── */
        if (got_fix)
        {
            /*
             * Valid fix: update lat, lng, alt, speed and mark gps_valid=true.
             * These become the new "last known" coordinates.
             */
            if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(20)) == pdTRUE)
            {
                g_sensor.gps_lat   = lat;
                g_sensor.gps_lng   = lng;
                g_sensor.gps_valid = true;
                xSemaphoreGive(xSensorMutex);
            }
            if (xSemaphoreTake(xDevStateMutex, pdMS_TO_TICKS(20)) == pdTRUE)
            {
                g_dev_state.gps_fix = true;
                xSemaphoreGive(xDevStateMutex);
            }
        }
        else
        {
            /*
             * No fix — per spec: keep LAST KNOWN lat/lng as-is in g_sensor.
             * Only flip gps_valid = false so telemetry includes the flag.
             * Do NOT zero out lat/lng.
             */
            if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(20)) == pdTRUE)
            {
                g_sensor.gps_valid = false;
                /* g_sensor.gps_lat and gps_lng intentionally unchanged */
                xSemaphoreGive(xSensorMutex);
            }
            if (xSemaphoreTake(xDevStateMutex, pdMS_TO_TICKS(20)) == pdTRUE)
            {
                g_dev_state.gps_fix = false;
                xSemaphoreGive(xDevStateMutex);
            }
        }

        wdg_checkin(WDG_IDX_GPS);
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(PERIOD_GPS_MS));
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Task_MQTT_Subscribe — Priority MEDIUM, continuous
 * ═══════════════════════════════════════════════════════════════════════════ */

void Task_MQTT_Subscribe(void *arg)
{
    (void)arg;

    for (;;)
    {
        /*
         * MQTT_PollIncoming() is non-blocking — it drains whatever is in
         * the SIM7600G DMA ring buffer right now and returns immediately.
         * We call it in a tight loop with a short yield so other tasks
         * of equal priority get CPU time.
         */
        MQTT_PollIncoming();

        wdg_checkin(WDG_IDX_MQTT_SUB);
        vTaskDelay(pdMS_TO_TICKS(50));  /* yield 50 ms — ample for MQTT URCs */
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Task_LED_Status — Priority LOW, 500 ms
 * ═══════════════════════════════════════════════════════════════════════════ */

/* GPIO pin mapping — adjust to your board */
#define LED_POWER_PORT      GPIOC
#define LED_POWER_PIN       GPIO_PIN_13   /* onboard LED, active LOW on Blue Pill */
#define LED_FAULT_PORT      GPIOB
#define LED_FAULT_PIN       GPIO_PIN_1
#define LED_TRIP_PORT       GPIOB
#define LED_TRIP_PIN        GPIO_PIN_10
#define LED_NETWORK_PORT    GPIOB
#define LED_NETWORK_PIN     GPIO_PIN_11

static inline void led_set(GPIO_TypeDef *port, uint16_t pin, bool on)
{
    HAL_GPIO_WritePin(port, pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Task_LED_Status(void *arg)
{
    (void)arg;
    TickType_t xLastWake = xTaskGetTickCount();
    bool blink_toggle = false;

    for (;;)
    {
        DeviceState_t dsnap;
        FaultState_t  fsnap;

        if (xSemaphoreTake(xDevStateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        { dsnap = g_dev_state; xSemaphoreGive(xDevStateMutex); }

        if (xSemaphoreTake(xFaultMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        { fsnap = g_fault; xSemaphoreGive(xFaultMutex); }

        blink_toggle = !blink_toggle;

        /* Power LED: solid ON when main power present, blink on battery */
        led_set(LED_POWER_PORT, LED_POWER_PIN,
                dsnap.battery_mode ? blink_toggle : true);

        /* Fault LED: solid ON when active fault */
        led_set(LED_FAULT_PORT, LED_FAULT_PIN, fsnap.active);

        /* Trip LED: solid ON when MCCB has been tripped */
        led_set(LED_TRIP_PORT, LED_TRIP_PIN, dsnap.mccb_tripped);

        /* Network LED: blink while connecting, fast-blink on comm loss, solid when ok */
        if (dsnap.comm_loss)
            led_set(LED_NETWORK_PORT, LED_NETWORK_PIN, blink_toggle); /* F10 blink */
        else if (!dsnap.network_ok)
            led_set(LED_NETWORK_PORT, LED_NETWORK_PIN, blink_toggle);
        else
            led_set(LED_NETWORK_PORT, LED_NETWORK_PIN, true);

        wdg_checkin(WDG_IDX_LED);
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(PERIOD_LED_MS));
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Task_BatteryMonitor — Priority LOW, 10 s
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Main power sense pin — GPIO input pulled HIGH, goes LOW on power loss */
#define MAIN_POWER_PORT     GPIOA
#define MAIN_POWER_PIN      GPIO_PIN_8

void Task_BatteryMonitor(void *arg)
{
    (void)arg;
    TickType_t xLastWake = xTaskGetTickCount();
    bool was_on_battery  = false;

    for (;;)
    {
        bool main_power_present =
            (HAL_GPIO_ReadPin(MAIN_POWER_PORT, MAIN_POWER_PIN) == GPIO_PIN_SET);

        bool now_on_battery = !main_power_present;

        /* ── Rising edge: just lost mains power ──────────────────────── */
        if (now_on_battery && !was_on_battery)
        {
            if (xSemaphoreTake(xDevStateMutex, pdMS_TO_TICKS(20)) == pdTRUE)
            {
                g_dev_state.battery_mode = true;
                xSemaphoreGive(xDevStateMutex);
            }
            if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(20)) == pdTRUE)
            {
                g_sensor.battery_backup = true;
                xSemaphoreGive(xSensorMutex);
            }

            /* Alert cloud immediately via MQTT fault topic */
            MQTT_PublishFault(FAULT_COMM_ERROR, "Main power loss — battery backup active");
        }

        /* ── Falling edge: mains restored ────────────────────────────── */
        if (!now_on_battery && was_on_battery)
        {
            if (xSemaphoreTake(xDevStateMutex, pdMS_TO_TICKS(20)) == pdTRUE)
            {
                g_dev_state.battery_mode = false;
                xSemaphoreGive(xDevStateMutex);
            }
            if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(20)) == pdTRUE)
            {
                g_sensor.battery_backup = false;
                xSemaphoreGive(xSensorMutex);
            }
        }

        was_on_battery = now_on_battery;

        wdg_checkin(WDG_IDX_BATTERY);
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(PERIOD_BATTERY_MS));
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Task_Watchdog — Priority HIGHEST, 1 s
 * ═══════════════════════════════════════════════════════════════════════════ */

void Task_Watchdog(void *arg)
{
    (void)arg;
    TickType_t xLastWake = xTaskGetTickCount();

    /* Initialise checkin timestamps so we don't false-trip at boot */
    uint32_t now = xTaskGetTickCount();
    for (int i = 0; i < WDG_NUM_TASKS; i++)
        s_wdg_table[i].last_checkin = now;

    for (;;)
    {
        now = xTaskGetTickCount();

        /* Check each monitored task */
        for (int i = 0; i < WDG_NUM_TASKS; i++)
        {
            uint32_t elapsed_ms =
                (now - s_wdg_table[i].last_checkin) * portTICK_PERIOD_MS;

            if (elapsed_ms > (WDG_HANG_TICKS * 1000U))
            {
                /*
                 * Task has hung for > 30 s.
                 * We do NOT feed the IWDG — let it expire and reboot.
                 * Optionally log the hanging task name via USART before dying.
                 */
                char msg[80];
                snprintf(msg, sizeof(msg),
                         "[WDG] HANG: %s silent for %lu ms — rebooting\r\n",
                         s_wdg_table[i].name, (unsigned long)elapsed_ms);
                HAL_UART_Transmit(&huart2,
                                  (const uint8_t *)msg, strlen(msg), 100);

                /* Stop feeding — IWDG will fire within ~32 s */
                for (;;) { /* spin — do not feed */ }
            }
        }

        /* All tasks healthy — feed hardware watchdog */
        HAL_IWDG_Refresh(&hiwdg);

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(PERIOD_WATCHDOG_MS));
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * RTOS_Init — create mutexes and launch all tasks
 * ═══════════════════════════════════════════════════════════════════════════ */

void RTOS_Init(void)
{
    /* Create mutexes */
    xSensorMutex   = xSemaphoreCreateMutex();
    xFaultMutex    = xSemaphoreCreateMutex();
    xDevStateMutex = xSemaphoreCreateMutex();

    configASSERT(xSensorMutex);
    configASSERT(xFaultMutex);
    configASSERT(xDevStateMutex);

    /* Create tasks
       Priorities: Highest=5, High=4, Medium=3, Low=2
       (FreeRTOS priority 0 is idle; keep tskIDLE_PRIORITY = 0)         */

    xTaskCreate(Task_SensorRead,     "SensorRead",  STACK_SENSOR,
                NULL, 4, &hSensorRead);

    xTaskCreate(Task_FaultEngine,    "FaultEngine", STACK_FAULT,
                NULL, 4, &hFaultEngine);

    xTaskCreate(Task_MQTT_Publish,   "MQTT_Pub",    STACK_MQTT_PUB,
                NULL, 3, &hMQTT_Publish);

    xTaskCreate(Task_GPS_Update,     "GPS_Update",  STACK_GPS,
                NULL, 2, &hGPS_Update);

    xTaskCreate(Task_MQTT_Subscribe, "MQTT_Sub",    STACK_MQTT_SUB,
                NULL, 3, &hMQTT_Subscribe);

    xTaskCreate(Task_LED_Status,     "LED_Status",  STACK_LED,
                NULL, 2, &hLED_Status);

    xTaskCreate(Task_BatteryMonitor, "BattMon",     STACK_BATTERY,
                NULL, 2, &hBatteryMonitor);

    xTaskCreate(Task_Watchdog,       "Watchdog",    STACK_WATCHDOG,
                NULL, 5, &hWatchdog);

    /* Start scheduler — does not return */
    vTaskStartScheduler();

    /* Should never reach here */
    for (;;) {}
}
