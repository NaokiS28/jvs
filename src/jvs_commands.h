#ifndef JVS_CMD
#define JVS_CMD

// JVS Command reference helper

#include <Arduino.h>

#define JVS_CMD_LIST 32

// Standard command list
const uint8_t jvsCmdList[32][4] = {
    // Command number, how many bytes command sends, how many bytes (MIN) to recieve, how many bytes (MAX) to receive
    // Broadcast
    {0xF0, 2, 0},       // RESET
    {0xF1, 2, 1, 1},    // Set ID
    {0xF2, 2, 0},       // Change Comm Method
    // Init
    {0x10, 1, 2, 102},  // Request name
    {0x11, 1, 2, 2},    // Request command revision
    {0x12, 1, 2, 2},    // Request JVS revision
    {0x13, 1, 2, 2},    // Request communication revision
    {0x14, 1, 6, 254},  // Request device features
    {0x15, 102, 1, 1},  // Send main board name
    // Inputs
    {0x20, 3, 3, 254},  // Read switch inputs
    {0x21, 2, 2, 254},  // Read coin counters
    {0x22, 2, 2, 254},  // Read analogue inputs
    {0x23, 2, 2, 254},  // Read rotary inputs
    {0x24, 1, 2, 2},    // Read keyboard keycodes
    {0x25, 2, 5, 5},    // Read touch/gun inputs
    {0x26, 2, 2, 254},  // Read misc. switch inputs
    // Outputs
    {0x2E, 2, 5, 5},    // Return remaining payout counter
    {0x2F, 1, 0, 0},    // Checksum error, request retransmit
    {0x30, 4, 1, 1},    // Coin counter decrease
    {0x31, 4, 1, 1},    // Payout counter increase
    {0x32, 254, 1, 1},  // Set GP Output 1
    {0x33, 254, 1, 1},  // Set analogue output
    {0x34, 254, 1, 1},  // Set character output
    {0x35, 4, 1, 1},    // Coin counter increase
    {0x36, 4, 1, 1},    // Decrement payout counter
    {0x37, 3, 1, 1},    // Set GP output 2
    {0x38, 3, 1, 1},    // Set GP output 3
};


#endif