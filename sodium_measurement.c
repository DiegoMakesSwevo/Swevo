#include <math.h>
#include <stdio.h>
#include <stdbool.h>

#define BUFFER_SIZE 10
#define CAL_POINTS 3
#define MAX_TEMP_VARIATION 2.0f
#define MAX_SIGNAL_NOISE 0.05f

// Buffer and calibration parameters
static float measurementBuffer[BUFFER_SIZE];
static int bufferIndex = 0;
static bool bufferFilled = false;
static float calSlope = 1.0f;
static float calIntercept = 0.0f;
static float lastTemp = 37.0f;
static float lastSignal = 0.0f;

// --- Easy-to-edit functions ---

// Add new value and compute moving average
float filter_moving_average(float newVal) {
    measurementBuffer[bufferIndex++] = newVal;
    if (bufferIndex >= BUFFER_SIZE) {
        bufferIndex = 0;
        bufferFilled = true;
    }
    int count = bufferFilled ? BUFFER_SIZE : bufferIndex;
    float sum = 0.0f;
    for (int i = 0; i < count; ++i)
        sum += measurementBuffer[i];
    return sum / count;
}

// Reject noisy outlier readings
bool reject_outlier(float newVal, float avg) {
    return fabs(newVal - avg) < MAX_SIGNAL_NOISE * avg;
}

// Compensate for temperature effects (edit tempCoeff for your sensor)
float compensate_temperature(float signal, float temp) {
    float tempCoeff = 0.01f; // Change this value to match your sensor
    return signal * (1.0f + tempCoeff * (temp - 37.0f));
}

// Calibrate with sodium standards (linear fit)
void recalibrate_sensor(const float *standards, const float *readings) {
    float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    for (int i = 0; i < CAL_POINTS; ++i) {
        sumX += standards[i];
        sumY += readings[i];
        sumXY += standards[i] * readings[i];
        sumX2 += standards[i] * standards[i];
    }
    calSlope = (CAL_POINTS * sumXY - sumX * sumY) / (CAL_POINTS * sumX2 - sumX * sumX);
    calIntercept = (sumY - calSlope * sumX) / CAL_POINTS;
}

// --- Stub: Replace with your actual sensor functions ---
float read_sensor_raw(void) { return 0.0f; }
float read_temperature(void) { return 37.0f; }

// --- Main sodium measurement function ---
float measure_sodium_concentration(void) {
    float rawSignal = read_sensor_raw();
    float temp = read_temperature();

    // Error check: unstable temperature
    if (fabs(temp - lastTemp) > MAX_TEMP_VARIATION) {
        printf("Temperature unstable. Measurement rejected.\n");
        return -1;
    }
    lastTemp = temp;

    float filteredSignal = filter_moving_average(rawSignal);

    // Error check: noisy signal
    if (!reject_outlier(rawSignal, filteredSignal)) {
        printf("Noisy or unstable signal detected. Measurement rejected.\n");
        return -1;
    }
    lastSignal = filteredSignal;

    float compensatedSignal = compensate_temperature(filteredSignal, temp);

    float sodiumConc = calSlope * compensatedSignal + calIntercept;

    return sodiumConc;
}

// --- Example: recalibration routine ---
void perform_recalibration(void) {
    float sodiumStandards[CAL_POINTS] = {50.0, 100.0, 150.0}; // mM
    float sensorReadings[CAL_POINTS] = { /* Fill with calibration readings */ };
    recalibrate_sensor(sodiumStandards, sensorReadings);
    printf("Calibration updated: slope=%f, intercept=%f\n", calSlope, calIntercept);
}

// --- How to edit ---
// 1. Implement read_sensor_raw() and read_temperature() for your hardware.
// 2. Change BUFFER_SIZE, CAL_POINTS, MAX_TEMP_VARIATION, MAX_SIGNAL_NOISE if needed.
// 3. Update tempCoeff in compensate_temperature() for your sensor response.
// 4. Use perform_recalibration() when recalibrating with standards.
// 5. Call measure_sodium_concentration() for sodium estimation.
// 6. Add advanced filtering (e.g., Kalman) if you wish.
