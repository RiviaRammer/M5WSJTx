#ifndef _INCLUDE_ENCODE_H_
#define _INCLUDE_ENCODE_H_

#include <stdbool.h>
#include <stdint.h>

#include "constants.h"

#ifdef __cplusplus
extern "C"
{
#endif

// typedef struct
// {
//     uint8_t tones[FT8_NN];
//     // for waveform readout:
//     int n_spsym;       // Number of waveform samples per symbol
//     float *pulse;      // [3 * n_spsym]
//     int idx_symbol;    // Index of the current symbol
//     float f0;          // Base frequency, Hertz
//     float signal_rate; // Waveform sample rate, Hertz
// } encoder_t;

// void encoder_init(float signal_rate, float *pulse_buffer);
// void encoder_set_f0(float f0);
// void encoder_process(const message_t *message); // in: message
// void encoder_generate(float *block);            // out: block of waveforms

/// Generate FT8 tone sequence from payload data
/// @param[in] payload - 10 byte array consisting of 77 bit payload
/// @param[out] tones  - array of FT8_NN (79) bytes to store the generated tones (encoded as 0..7)
void ft8_encode(const uint8_t* payload, uint8_t* tones);

/// Generate FT4 tone sequence from payload data
/// @param[in] payload - 10 byte array consisting of 77 bit payload
/// @param[out] tones  - array of FT4_NN (105) bytes to store the generated tones (encoded as 0..3)
void ft4_encode(const uint8_t* payload, uint8_t* tones);

/// Generate tone sequence from payload data for the selected protocol.
/// @return true when the protocol is supported.
bool ftx_encode(ftx_protocol_t protocol, const uint8_t* payload, uint8_t* tones);

#ifdef __cplusplus
}
#endif

#endif // _INCLUDE_ENCODE_H_
