/*
 * adc.c
 *
 *  Created on: Apr 10, 2026
 *      Author: test1
 */


#include "main.h"
#include "adc.h"


HAL_StatusTypeDef ADC_Init(ADC_HandleTypeDef *hadc)
{
    if (hadc == NULL) {
        return HAL_ERROR;
    }

    s_hadc = hadc;

    /* Channel configuration table: rank, channel, GPIO (for reference) */
    static const struct {
        uint32_t rank;
        uint32_t channel;
    } ch_cfg[ADC_NUM_CHANNELS] = {
        { 1U, ADC_CHANNEL_0 },   /* PA0 — ADC_TEMP */
        { 2U, ADC_CHANNEL_1 },   /* PA1 — ADC_CT1  */
        { 3U, ADC_CHANNEL_2 },   /* PA2 — ADC_CT2  */
        { 4U, ADC_CHANNEL_3 },   /* PA3 — ADC_CT3  */
        { 5U, ADC_CHANNEL_4 },   /* PA4 — ADC_PT1  */
        { 6U, ADC_CHANNEL_5 },   /* PA5 — ADC_PT2  */
        { 7U, ADC_CHANNEL_6 },   /* PA6 — ADC_PT3  */
    };

    ADC_ChannelConfTypeDef sConfig = {0};

    /* Use 55.5 cycles sample time per channel.
     * At 72MHz with ADC prescaler /6 → ADC clock = 12MHz.
     * Conversion time per channel ≈ (55.5 + 12.5) / 12MHz ≈ 5.67µs.
     * 7 channels × 5.67µs ≈ 39.7µs per full scan.
     * 50 scans/cycle = ~1.985ms per cycle → ~504Hz scan rate >> 1kHz/ch needed.
     * Note: STM32F103 ADC max clock is 14MHz; 12MHz is within spec.
     */
    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;

    for (uint8_t i = 0U; i < ADC_NUM_CHANNELS; i++) {
        sConfig.Channel = ch_cfg[i].channel;
        sConfig.Rank    = ch_cfg[i].rank;

        if (HAL_ADC_ConfigChannel(s_hadc, &sConfig) != HAL_OK) {
            return HAL_ERROR;
        }
    }

    /* Initialise buffer */
    memset(&g_adc_buffer, 0, sizeof(g_adc_buffer));
    memset(s_adc_dma_buf, 0, sizeof(s_adc_dma_buf));

    /* Run internal calibration (mandatory on STM32F1 before first conversion) */
    if (HAL_ADCEx_Calibration_Start(s_hadc) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}


HAL_StatusTypeDef ADC_StartDMA(void)
{
    if (s_hadc == NULL) {
        return HAL_ERROR;
    }

    return HAL_ADC_Start_DMA(
        s_hadc,
        (uint32_t *)s_adc_dma_buf,
        (uint32_t)(ADC_NUM_CHANNELS * ADC_SAMPLES_PER_CYCLE)
    );
}


HAL_StatusTypeDef ADC_StopDMA(void)
{
    if (s_hadc == NULL) {
        return HAL_ERROR;
    }
    return HAL_ADC_Stop_DMA(s_hadc);
}


HAL_StatusTypeDef ADC_TriggerSingleScan(void)
{
    if (s_hadc == NULL) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = HAL_ADC_Start(s_hadc);
    if (status != HAL_OK) {
        return status;
    }

    /* Poll for all 7 channel conversions (timeout = 10ms per channel) */
    for (uint8_t ch = 0U; ch < ADC_NUM_CHANNELS; ch++) {
        status = HAL_ADC_PollForConversion(s_hadc, 10U);
        if (status != HAL_OK) {
            HAL_ADC_Stop(s_hadc);
            return status;
        }
        /* Store directly into sample 0 of each channel buffer */
        g_adc_buffer.raw[ch][0] = (uint16_t)HAL_ADC_GetValue(s_hadc);
    }

    HAL_ADC_Stop(s_hadc);
    return HAL_OK;
}


void ADC_CalibrateOffset(ADC_SensorData_t *sensor_data)
{
    if (sensor_data == NULL) {
        return;
    }

    /* Default: use theoretical mid-rail for all AC channels */
    for (uint8_t ch = 0U; ch < ADC_NUM_CHANNELS; ch++) {
        sensor_data->dc_offset[ch] = ADC_DC_OFFSET_V;
    }

    /*
     * Optional: measure actual mid-rail using a single blocking scan.
     * Uncomment if hardware is powered but no AC signal is present at startup.
     *
    if (ADC_TriggerSingleScan() == HAL_OK) {
        for (uint8_t ch = 0U; ch < ADC_NUM_CHANNELS; ch++) {
            sensor_data->dc_offset[ch] = ADC_RawToVoltage(g_adc_buffer.raw[ch][0]);
        }
    }
     */
}

/**
 * @brief  Calculate true RMS from a raw ADC sample array.
 *
 *         Formula:
 *           1. Convert raw count to voltage: v = (raw × 3.3 / 4095) − offset
 *           2. Accumulate sum of squares: sum_sq += v²
 *           3. RMS = sqrt(sum_sq / count)
 *
 * @param  samples  Pointer to uint16_t array of ADC raw counts.
 * @param  count    Number of samples (typically ADC_SAMPLES_PER_CYCLE).
 * @param  offset   DC offset voltage (V) to subtract (e.g. 1.65V).
 * @retval RMS voltage (V).
 */
float ADC_CalculateRMS(const uint16_t *samples, uint32_t count, float offset)
{
    if ((samples == NULL) || (count == 0U)) {
        return 0.0f;
    }

    float sum_sq = 0.0f;

    for (uint32_t i = 0U; i < count; i++) {
        /* Convert raw ADC count to voltage and remove DC bias */
        float v = ADC_RawToVoltage(samples[i]) - offset;
        sum_sq += (v * v);
    }

    return sqrtf(sum_sq / (float)count);
}


void ADC_ProcessAll(ADC_SensorData_t *sensor_data)
{
    if (sensor_data == NULL) {
        return;
    }

    /* De-interleave DMA buffer → g_adc_buffer.raw[ch][sample] */
    ADC_DeinterleaveBuffer();

    /* ---- Temperature (PA0, CH0) ---- */
    /* Temperature uses a single raw reading (averaged over samples) */
    uint32_t temp_sum = 0U;
    for (uint32_t s = 0U; s < ADC_SAMPLES_PER_CYCLE; s++) {
        temp_sum += g_adc_buffer.raw[ADC_CH_TEMP][s];
    }
    uint16_t temp_avg_raw = (uint16_t)(temp_sum / ADC_SAMPLES_PER_CYCLE);
    sensor_data->temperature_oil = ADC_ConvertTemperature(temp_avg_raw);

    /* ---- Current Channels (CT) — PA1, PA2, PA3 ---- */
    float rms_ct1 = ADC_CalculateRMS(
        g_adc_buffer.raw[ADC_CH_CT1],
        ADC_SAMPLES_PER_CYCLE,
        sensor_data->dc_offset[ADC_CH_CT1]
    );
    float rms_ct2 = ADC_CalculateRMS(
        g_adc_buffer.raw[ADC_CH_CT2],
        ADC_SAMPLES_PER_CYCLE,
        sensor_data->dc_offset[ADC_CH_CT2]
    );
    float rms_ct3 = ADC_CalculateRMS(
        g_adc_buffer.raw[ADC_CH_CT3],
        ADC_SAMPLES_PER_CYCLE,
        sensor_data->dc_offset[ADC_CH_CT3]
    );

    /* Scale ADC RMS voltage → real current (A) */
    sensor_data->current_R = rms_ct1 * CT_SCALING_FACTOR;
    sensor_data->current_Y = rms_ct2 * CT_SCALING_FACTOR;
    sensor_data->current_B = rms_ct3 * CT_SCALING_FACTOR;

    /* ---- Voltage Channels (PT) — PA4, PA5, PA6 ---- */
    float rms_pt1 = ADC_CalculateRMS(
        g_adc_buffer.raw[ADC_CH_PT1],
        ADC_SAMPLES_PER_CYCLE,
        sensor_data->dc_offset[ADC_CH_PT1]
    );
    float rms_pt2 = ADC_CalculateRMS(
        g_adc_buffer.raw[ADC_CH_PT2],
        ADC_SAMPLES_PER_CYCLE,
        sensor_data->dc_offset[ADC_CH_PT2]
    );
    float rms_pt3 = ADC_CalculateRMS(
        g_adc_buffer.raw[ADC_CH_PT3],
        ADC_SAMPLES_PER_CYCLE,
        sensor_data->dc_offset[ADC_CH_PT3]
    );

    /* Scale ADC RMS voltage → real phase voltage (V) */
    sensor_data->voltage_R = rms_pt1 * PT_SCALING_FACTOR;
    sensor_data->voltage_Y = rms_pt2 * PT_SCALING_FACTOR;
    sensor_data->voltage_B = rms_pt3 * PT_SCALING_FACTOR;

    /* ---- Phase Imbalance (Section 4.2) ---- */
    float v_avg = (sensor_data->voltage_R +
                   sensor_data->voltage_Y +
                   sensor_data->voltage_B) / 3.0f;

    if (v_avg > 0.0f) {
        float dev_R = fabsf(sensor_data->voltage_R - v_avg);
        float dev_Y = fabsf(sensor_data->voltage_Y - v_avg);
        float dev_B = fabsf(sensor_data->voltage_B - v_avg);

        float max_dev = dev_R;
        if (dev_Y > max_dev) { max_dev = dev_Y; }
        if (dev_B > max_dev) { max_dev = dev_B; }

        sensor_data->phase_imbalance_pct = (max_dev / v_avg) * 100.0f;
    } else {
        sensor_data->phase_imbalance_pct = 0.0f;
    }

    /* ---- Update metadata ---- */
    sensor_data->data_valid     = true;
    sensor_data->timestamp_tick = HAL_GetTick();
}


float ADC_ConvertTemperature(uint16_t raw_adc)
{
    float v_ntc = ADC_RawToVoltage(raw_adc);

    /* Avoid division by zero if reading is at rail */
    if (v_ntc >= NTC_VCC) { return -273.15f; }
    if (v_ntc <= 0.0f)    { return  999.0f;  }

    float r_ntc = NTC_SERIES_R * v_ntc / (NTC_VCC - v_ntc);

    return ADC_BetaEquation(r_ntc);
}


bool ADC_IsDataReady(void)
{
    if (g_adc_buffer.conversion_complete) {
        g_adc_buffer.conversion_complete = false;
        return true;
    }
    return false;
}


void ADC_DMA_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == s_hadc) {
        g_adc_buffer.conversion_complete = true;
    }
}


static void ADC_DeinterleaveBuffer(void)
{
    for (uint32_t s = 0U; s < ADC_SAMPLES_PER_CYCLE; s++) {
        for (uint32_t ch = 0U; ch < ADC_NUM_CHANNELS; ch++) {
            g_adc_buffer.raw[ch][s] =
                s_adc_dma_buf[(s * ADC_NUM_CHANNELS) + ch];
        }
    }
}


static float ADC_RawToVoltage(uint16_t raw)
{
    return ((float)raw * ADC_VREF_V) / ADC_RESOLUTION;
}


static float ADC_BetaEquation(float resistance)
{
    /* T(K) = (Beta × T0) / (Beta + T0 × ln(R / R25))
     * where T0 = 298.15K (25°C) */
    const float T0_K = 298.15f;

    if (resistance <= 0.0f) {
        return 999.0f; /* Sensor open-circuit */
    }

    float ln_ratio = logf(resistance / NTC_R25);
    float temp_K   = (NTC_BETA * T0_K) / (NTC_BETA + T0_K * ln_ratio);

    return temp_K - 273.15f;
}
