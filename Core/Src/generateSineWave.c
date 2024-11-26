#include "arm_math.h"

// Part 1
#define SAMPLING_C2 1000 // Sampling frequency in Hz (1 millisecond per sample)
#define C2 65      // Sine wave frequency in Hz (65 Hz, approximately C2)


// Part 1: Generate note C2
uint32_t generateSine(uint32_t precision, uint32_t sampleIndex) {
    float32_t sineValue = arm_sin_f32(2*PI*C2/SAMPLING_C2*sampleIndex);
    // Scale sineValue value (4096 - 1)
    return (uint32_t) ((sineValue+1.0f)/2.0f * (float32_t)(precision-1));
}

// Part 2 and Part 3
void generateArray(uint32_t* array, uint32_t precision, uint32_t size) {
    for (uint32_t i = 0; i < size; i++) {
        float rad = (float)(2.0f * PI * i / size);
        // Scale sineValue to use (precision - 1) and target 2/3 of 12-bit DAC range
        float sineValue = ((arm_sin_f32(rad) + 1.0f) / 2.0f) * (float)(precision - 1);
        array[i] = (uint32_t)sineValue;
    }
}




