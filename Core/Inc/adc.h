/*
 * adc.h
 *
 *  Created on: Apr 10, 2026
 *      Author: test1
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_


#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>



static ADC_HandleTypeDef *s_hadc = NULL;

static uint16_t s_adc_dma_buf[ADC_NUM_CHANNELS * ADC_SAMPLES_PER_CYCLE];

ADC_RawBuffer_t g_adc_buffer;


#define ADC_NUM_CHANNELS        7

/** Number of samples per RMS window (50 samples/cycle @ 50Hz → 1kHz) */
#define ADC_SAMPLES_PER_CYCLE   50

/** ADC full-scale voltage (3.3V) */
#define ADC_VREF_V              3.3f

/** ADC resolution (12-bit) */
#define ADC_RESOLUTION          4095.0f

/** DC mid-rail offset (approximately 1.65V for AC-coupled signals) */
#define ADC_DC_OFFSET_V         1.65f

/**
 * CT channel scaling factor.
 * Burden resistor: 100Ω, 5mA CT output = 0.5V AC peak.
 * Scale: RMS ADC voltage → real current (A).
 * Example: 0.5V peak / √2 = 0.354V RMS → represents a known CT ratio.
 * Adjust CT_TURNS_RATIO to match the physical CT used (e.g. 100:5A).
 */
#define CT_TURNS_RATIO          20.0f   /**< e.g. 100A primary / 5A secondary */
#define CT_BURDEN_OHMS          100.0f  /**< Burden resistor value */
#define CT_SCALING_FACTOR       (CT_TURNS_RATIO / CT_BURDEN_OHMS)  /**< A/V */

/**
 * PT channel scaling factor.
 * Voltage divider: 470kΩ / 10kΩ → scales 5V AC PT output to ~0.1V–0.9V AC peak.
 * PT secondary nominal: 110V (phase) → divider ratio ≈ 47+1 = 48.
 * Adjust PT_PRIMARY_NOMINAL to match the actual PT ratio used.
 */
#define PT_DIVIDER_RATIO        48.0f   /**< (470k + 10k) / 10k */
#define PT_SECONDARY_NOMINAL_V  110.0f  /**< PT secondary RMS voltage */
#define PT_ADC_NOMINAL_V        (PT_SECONDARY_NOMINAL_V / PT_DIVIDER_RATIO)
#define PT_SCALING_FACTOR       (PT_SECONDARY_NOMINAL_V / PT_ADC_NOMINAL_V)

/* -------------------------------------------------------------------------
 * Channel Index Definitions
 * ------------------------------------------------------------------------- */
#define ADC_CH_TEMP     0   /**< PA0 — Oil temperature (NTC/DS18B20) */
#define ADC_CH_CT1      1   /**< PA1 — Current Phase R */
#define ADC_CH_CT2      2   /**< PA2 — Current Phase Y */
#define ADC_CH_CT3      3   /**< PA3 — Current Phase B */
#define ADC_CH_PT1      4   /**< PA4 — Voltage Phase R */
#define ADC_CH_PT2      5   /**< PA5 — Voltage Phase Y */
#define ADC_CH_PT3      6   /**< PA6 — Voltage Phase B */

/* -------------------------------------------------------------------------
 * Temperature Conversion (NTC-based, linear approximation)
 * Adjust NTC_R25, NTC_BETA, NTC_SERIES_R as per actual component used.
 * ------------------------------------------------------------------------- */
#define NTC_R25         10000.0f    /**< NTC resistance at 25°C (10kΩ) */
#define NTC_BETA        3950.0f     /**< NTC Beta value */
#define NTC_SERIES_R    10000.0f    /**< Series resistor in voltage divider */
#define NTC_VCC         3.3f        /**< Supply voltage */

/* -------------------------------------------------------------------------
 * Fault Thresholds (from Section 5 of TSGP-1 Spec)
 * ------------------------------------------------------------------------- */
#define FAULT_OVERVOLTAGE_V         260.0f  /**< F01: Any phase > 260V */
#define FAULT_UNDERVOLTAGE_V        180.0f  /**< F02: Any phase < 180V */
#define FAULT_OVERCURRENT_A         90.0f   /**< F03: Any phase > 90A */
#define FAULT_SHORT_CIRCUIT_A       150.0f  /**< F04: Any phase > 150A */
#define FAULT_PHASE_IMBALANCE_PCT   15.0f   /**< F05: Imbalance > 15% */
#define FAULT_OIL_TEMP_C            85.0f   /**< F06: Oil temp > 85°C */

/* -------------------------------------------------------------------------
 * Data Structures
 * ------------------------------------------------------------------------- */

/**
 * @brief Raw ADC sample buffer for all channels (DMA).
 *        Organized as interleaved: [CH0, CH1, ... CH6] × SAMPLES_PER_CYCLE
 */
typedef struct {
    uint16_t raw[ADC_NUM_CHANNELS][ADC_SAMPLES_PER_CYCLE];
    volatile bool     conversion_complete;
} ADC_RawBuffer_t;

/**
 * @brief Processed sensor values (shared data structure for FreeRTOS tasks).
 */
typedef struct {
    /* Phase voltages (V RMS) */
    float voltage_R;
    float voltage_Y;
    float voltage_B;

    /* Phase currents (A RMS) */
    float current_R;
    float current_Y;
    float current_B;

    /* Oil temperature (°C) */
    float temperature_oil;

    /* Phase imbalance (%) */
    float phase_imbalance_pct;

    /* Raw DC offset per channel (calibrated mid-rail in ADC counts) */
    float dc_offset[ADC_NUM_CHANNELS];

    /* Validity flag */
    bool data_valid;

    /* Timestamp (FreeRTOS tick) */
    uint32_t timestamp_tick;
} ADC_SensorData_t;

/* -------------------------------------------------------------------------
 * Function Prototypes
 * ------------------------------------------------------------------------- */

/**
 * @brief  Initialise ADC peripheral with DMA for all 7 channels.
 *         Must be called once during system init, after HAL_Init().
 * @param  hadc  Pointer to HAL ADC handle (hadc1, configured in CubeMX).
 * @retval HAL_OK on success, HAL_ERROR on failure.
 */
HAL_StatusTypeDef ADC_Init(ADC_HandleTypeDef *hadc);

/**
 * @brief  Start continuous ADC DMA conversion for all channels.
 *         Call once after ADC_Init(). Conversions run autonomously.
 * @retval HAL_OK on success.
 */
HAL_StatusTypeDef ADC_StartDMA(void);

/**
 * @brief  Stop ADC DMA conversion.
 * @retval HAL_OK on success.
 */
HAL_StatusTypeDef ADC_StopDMA(void);

/**
 * @brief  Trigger a single scan conversion of all channels (polling mode).
 *         Use during startup/calibration only. Blocks until complete.
 * @retval HAL_OK on success.
 */
HAL_StatusTypeDef ADC_TriggerSingleScan(void);

/**
 * @brief  Perform DC offset calibration.
 *         Call during startup with no AC signal present (or use mid-rail assumption).
 *         Stores calibrated offset per channel in sensor_data.dc_offset[].
 * @param  sensor_data  Pointer to shared sensor data structure.
 */
void ADC_CalibrateOffset(ADC_SensorData_t *sensor_data);

/**
 * @brief  Calculate true RMS value from raw ADC samples for one channel.
 * @param  samples  Pointer to raw ADC sample array for this channel.
 * @param  count    Number of samples.
 * @param  offset   DC offset in volts (1.65V for mid-rail AC signals).
 * @retval RMS voltage in volts (float).
 */
float ADC_CalculateRMS(const uint16_t *samples, uint32_t count, float offset);

/**
 * @brief  Process all raw ADC buffers and populate the sensor data structure.
 *         Applies RMS calculation, DC offset correction, and scaling factors.
 *         Calculates phase imbalance percentage.
 *         Call from Task_SensorRead (High priority, 100ms period).
 * @param  sensor_data  Pointer to shared sensor data structure (output).
 */
void ADC_ProcessAll(ADC_SensorData_t *sensor_data);

/**
 * @brief  Convert raw ADC temperature reading to °C (NTC thermistor).
 * @param  raw_adc  Raw 12-bit ADC count.
 * @retval Temperature in degrees Celsius.
 */
float ADC_ConvertTemperature(uint16_t raw_adc);

/**
 * @brief  Check if a new DMA conversion cycle is complete.
 *         Clears the flag after reading.
 * @retval true if new data is available, false otherwise.
 */
bool ADC_IsDataReady(void);

/**
 * @brief  HAL ADC conversion complete callback (DMA).
 *         Called by HAL_ADC_ConvCpltCallback(). Do not call directly.
 */
void ADC_DMA_ConvCpltCallback(ADC_HandleTypeDef *hadc);

extern ADC_RawBuffer_t g_adc_buffer;

/**
 * @brief  De-interleave DMA flat buffer into per-channel sample arrays.
 *
 *         DMA layout (interleaved, as ADC scans all channels per trigger):
 *           Index 0: CH0 sample 0
 *           Index 1: CH1 sample 0
 *           ...
 *           Index 6: CH6 sample 0
 *           Index 7: CH0 sample 1
 *           ...
 *           Index (N*7 + ch): CHch sample N
 */
static void     ADC_DeinterleaveBuffer(void);



/**
 * @brief  Convert a 12-bit ADC raw count to voltage (V).
 * @param  raw  12-bit ADC count (0–4095).
 * @retval Voltage in V (0.0–3.3V).
 */
static float    ADC_RawToVoltage(uint16_t raw);



/**
 * @brief  Steinhart–Hart Beta equation: R_ntc → temperature (°C).
 * @param  resistance  NTC resistance in ohms.
 * @retval Temperature in °C.
 */
static float    ADC_BetaEquation(float resistance);

#endif /* INC_ADC_H_ */
