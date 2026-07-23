#ifndef ICR8600_CORE_H
#define ICR8600_CORE_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handle to an opened IC-R8600 [I/Q OUT] device (streaming personality).
typedef struct icr8600_device icr8600_device;

typedef enum {
    ICR8600_OK = 0,
    ICR8600_ERR_NOT_FOUND = -1,
    ICR8600_ERR_FIRMWARE_LOAD_FAILED = -2,
    ICR8600_ERR_FIRMWARE_FILE_NOT_FOUND = -3,
    ICR8600_ERR_ALREADY_OPEN = -4,
    ICR8600_ERR_IO = -5
} icr8600_status;

// Sample rate codes used by icr8600_start_streaming.
typedef enum {
    ICR8600_RATE_5_12_MSPS = 0x01,
    ICR8600_RATE_3_84_MSPS = 0x02,
    ICR8600_RATE_1_92_MSPS = 0x03,
    ICR8600_RATE_960_KSPS  = 0x04,
    ICR8600_RATE_480_KSPS  = 0x05,
    ICR8600_RATE_240_KSPS  = 0x06
} icr8600_rate_code;

// Gain elements exposed via icr8600_set_gain/icr8600_get_gain. Units:
//   ICR8600_GAIN_ATT    - attenuator, dB (valid: 0, 10, 20, 30)
//   ICR8600_GAIN_RF     - RF gain, 0..255 (raw scale; 255 = max)
//   ICR8600_GAIN_PREAMP - preamp, 0.0 (off) or 1.0 (on)
//   ICR8600_GAIN_IPP    - IP+, 0.0 (off) or 1.0 (on)
typedef enum {
    ICR8600_GAIN_ATT    = 0,
    ICR8600_GAIN_RF     = 1,
    ICR8600_GAIN_PREAMP = 2,
    ICR8600_GAIN_IPP    = 3
} icr8600_gain_element;

// Opens the radio's [I/Q OUT] USB device.
//
// If the device currently enumerates in its loader personality
// (VID 0x0C26 / PID 0x0022), this call replays the user-supplied firmware
// sequence at `firmwarePath` to bring it into streaming mode (PID 0x0023)
// before returning a handle.
//
// If the device is already in streaming mode, `firmwarePath` is ignored and
// may be NULL.
//
// On success, `outDevice` receives an opaque handle; caller must release it
// with icr8600_close().
icr8600_status icr8600_open(const char *firmwarePath, icr8600_device **outDevice);

void icr8600_close(icr8600_device *device);

// Starts I/Q bulk streaming on the device at the given sample rate
// (icr8600_rate_code) and initial tuned center frequency in Hz. Runs the
// full CI-V startup sequence (mode, freq, atten, antenna, RF gain, preamp,
// IP+, then I/Q ON) before returning.
icr8600_status icr8600_start_streaming(icr8600_device *device, uint8_t rateCode, uint64_t frequencyHz);

// Returns current async reader counters for smoke-testing the bulk stream.
// Any output pointer may be NULL. Valid only after start_streaming succeeds.
icr8600_status icr8600_get_stream_stats(icr8600_device *device, uint64_t *outBytes, uint64_t *outSyncs, uint64_t *outCompletions);

// Drains up to maxSamplePairs aligned (I, Q) int16 pairs (little-endian,
// interleaved) into outBuffer, which must be at least maxSamplePairs*4
// bytes, via a direct copy from the internal FIFO (no intermediate Data
// allocation). Sync markers have already been stripped and I/Q
// phase-locked. *outPairsWritten receives the actual number of pairs
// written (may be less than requested, including zero, if the internal
// FIFO is empty).
//
// *outDropped is set to 1 if the internal FIFO overflowed and trimmed
// (dropped) the oldest samples since the last call to icr8600_read_iq, or
// 0 otherwise. Callers implementing a SoapySDR-style streaming API should
// surface this as an overflow/discontinuity indicator (e.g.
// SOAPY_SDR_OVERFLOW) on the read immediately following a drop. May be
// NULL if the caller doesn't care about overflow reporting.
icr8600_status icr8600_read_iq(icr8600_device *device, void *outBuffer, size_t maxSamplePairs, size_t *outPairsWritten, int *outDropped);

icr8600_status icr8600_stop_streaming(icr8600_device *device);

// Sets a gain element (see icr8600_gain_element). Updates the device's
// cached setting immediately; if streaming is currently active, also sends
// the corresponding CI-V command live (no stream restart required -- these
// settings change live over the device control protocol). If not
// currently streaming, the value is cached and applied on the next
// icr8600_start_streaming call using the stored startup settings.
icr8600_status icr8600_set_gain(icr8600_device *device, uint8_t element, double value);

// Reads back the cached value for a gain element (no hardware round-trip).
icr8600_status icr8600_get_gain(icr8600_device *device, uint8_t element, double *outValue);

// Sets the tuned center frequency in Hz. Same live/cached semantics as
// icr8600_set_gain above: updates the cached value immediately, and if
// streaming is active, also sends the live CI-V frequency command without
// restarting the stream.
icr8600_status icr8600_set_frequency(icr8600_device *device, uint64_t frequencyHz);

// Reads back the cached tuned center frequency (no hardware round-trip).
icr8600_status icr8600_get_frequency(icr8600_device *device, uint64_t *outFrequencyHz);

// Selects the antenna (0 = ANT1, 1 = ANT2). Same live/cached semantics as
// icr8600_set_gain above.
icr8600_status icr8600_set_antenna(icr8600_device *device, uint8_t index);

icr8600_status icr8600_get_antenna(icr8600_device *device, uint8_t *outIndex);

#ifdef __cplusplus
}
#endif

#endif // ICR8600_CORE_H
