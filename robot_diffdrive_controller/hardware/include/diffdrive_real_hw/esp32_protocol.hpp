/**
 * esp32_protocol.hpp  —  shared binary serial protocol
 * =====================================================
 *
 * Frame layout (little-endian):
 *
 *   [START 0xAA] [CMD] [PAYLOAD …] [XOR-checksum]
 *
 * Commands  PC → ESP32:
 *   0x01  CMD_SET_VEL   8 B  : float32 left_vel,  float32 right_vel  (rad/s)
 *   0x02  CMD_STOP      0 B
 *   0x03  CMD_PING      0 B
 *
 * Responses  ESP32 → PC:
 *   0x10  CMD_STATE    16 B  : float32 left_pos, float32 right_pos  (rad)
 *                              float32 left_vel, float32 right_vel  (rad/s)
 *   0x13  CMD_PONG      0 B
 *   0xFF  CMD_ERR       1 B  : error code
 *
 * Baud rate: 921600 on both sides.
 * Wire time @ 921600:  SET_VEL frame (11 B) ≈ 0.12 ms
 *                      STATE   frame (19 B) ≈ 0.20 ms
 */

#pragma once
#include <stdint.h>

static const uint8_t PROTO_START   = 0xAA;

static const uint8_t CMD_SET_VEL   = 0x01;
static const uint8_t CMD_STOP      = 0x02;
static const uint8_t CMD_PING      = 0x03;
static const uint8_t CMD_STATE     = 0x10;
static const uint8_t CMD_PONG      = 0x13;
static const uint8_t CMD_ERR       = 0xFF;

static const int PAYLOAD_SET_VEL   = 8;
static const int PAYLOAD_STATE     = 16;

// XOR checksum of CMD + all payload bytes
inline uint8_t proto_checksum(uint8_t cmd, const uint8_t* payload, int len)
{
    uint8_t c = cmd;
    for (int i = 0; i < len; i++) c ^= payload[i];
    return c;
}

// float ↔ 4 bytes, little-endian, no UB
inline void proto_pack_f32(uint8_t* buf, float v)
{
    uint32_t u; __builtin_memcpy(&u, &v, 4);
    buf[0] = u & 0xFF; buf[1] = (u>>8)&0xFF;
    buf[2] = (u>>16)&0xFF; buf[3] = (u>>24)&0xFF;
}
inline float proto_unpack_f32(const uint8_t* buf)
{
    uint32_t u = (uint32_t)buf[0] | ((uint32_t)buf[1]<<8)
               | ((uint32_t)buf[2]<<16) | ((uint32_t)buf[3]<<24);
    float v; __builtin_memcpy(&v, &u, 4); return v;
}
