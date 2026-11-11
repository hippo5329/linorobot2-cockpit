// The SEN0628 8x8 ToF over I2C, as a non-blocking state machine. See tof.h for the protocol.
// Protocol from DFRobot_MatrixLidar (https://github.com/DFRobot/DFRobot_MatrixLidar),
// Copyright 2024 DFRobot Co.Ltd, MIT licence; reimplemented here without its delays and
// heap allocations.
#include "tof.h"

#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include "mcu_env.h"   // the console wrapper: Serial goes where base_controller.console says

namespace {

const uint8_t HEAD = 0x55;
const uint8_t CMD_SETMODE = 1;
const uint8_t CMD_ALLDATA = 2;
const uint8_t STATUS_OK = 0x53;
const uint8_t STATUS_FAILED = 0x63;
const uint8_t MATRIX_8X8 = 8;
const size_t CHUNK = 32;                 // the board's and Wire's per-transfer limit

const uint32_t MODE_SETTLE_MS = 5000;    // DFRobot's library waits this long after a mode change
const uint32_t FRAME_PERIOD_MS = 66;     // the sensor ranges at 15 Hz in 8x8
const uint32_t POLL_MS = 5;              // how often a pending reply is asked for
const uint32_t REPLY_TIMEOUT_MS = 500;
const uint8_t MAX_FAILS = 5;             // in a row: start over with the mode command

enum State : uint8_t { OFF, MODE_REPLY, SETTLING, IDLE, DATA_REPLY, DATA_PAYLOAD };

uint8_t s_addr = 0;
State s_state = OFF;
uint32_t s_t0 = 0;          // when the current wait began
uint32_t s_last_poll = 0;
uint32_t s_last_request = 0;
uint8_t s_cmd = 0;          // the command whose reply is awaited
uint16_t s_len = 0;         // payload length of the reply being read
uint16_t s_got = 0;
uint8_t s_fails = 0;
uint8_t s_rx[TOF_FRAME_BYTES];
uint8_t s_frame[TOF_FRAME_BYTES];
bool s_frame_ready = false;
uint32_t s_frames = 0;
uint32_t s_errors = 0;

bool sendCommand(uint8_t cmd, const uint8_t *args, uint8_t nargs)
{
    const uint16_t n = (uint16_t)nargs + 1;     // the count includes the command
    Wire.beginTransmission(s_addr);
    Wire.write(HEAD);
    Wire.write((uint8_t)(n >> 8));
    Wire.write((uint8_t)(n & 0xFF));
    Wire.write(cmd);
    for (uint8_t i = 0; i < nargs; i++)
        Wire.write(args[i]);
    return Wire.endTransmission() == 0;
}

bool readBytes(uint8_t *buf, size_t n)
{
    if (Wire.requestFrom((int)s_addr, (int)n) != (int)n)
        return false;
    for (size_t i = 0; i < n; i++)
        buf[i] = (uint8_t)Wire.read();
    return true;
}

void fail()
{
    s_errors++;
    if (++s_fails >= MAX_FAILS)
    {
        // Something reset or lost the mode: begin again, as at boot.
        s_fails = 0;
        static const uint8_t mode_args[4] = {0, 0, 0, MATRIX_8X8};
        s_cmd = CMD_SETMODE;
        s_state = sendCommand(CMD_SETMODE, mode_args, 4) ? MODE_REPLY : IDLE;
        s_t0 = millis();
        return;
    }
    s_state = IDLE;
}

// Poll for the reply header of s_cmd. Returns true once the header is in (s_len set).
// On a failed or foreign reply, or a timeout, records a failure and returns false.
bool pollHeader()
{
    const uint32_t now = millis();
    if (now - s_t0 > REPLY_TIMEOUT_MS)
    {
        fail();
        return false;
    }
    if (now - s_last_poll < POLL_MS)
        return false;
    s_last_poll = now;
    uint8_t status = 0xFF;
    if (!readBytes(&status, 1) || (status != STATUS_OK && status != STATUS_FAILED))
        return false;                       // not ready yet: ask again later
    uint8_t hdr[3];                         // command, length low, length high
    if (!readBytes(hdr, 3) || hdr[0] != s_cmd || status == STATUS_FAILED)
    {
        fail();
        return false;
    }
    s_len = (uint16_t)hdr[1] | ((uint16_t)hdr[2] << 8);
    s_got = 0;
    return true;
}

}  // namespace

bool initTof(uint8_t addr)
{
    s_addr = addr;
    s_state = OFF;
    if (addr == 0)
        return false;
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() != 0)
    {
        Serial.printf("[tof] no 8x8 ToF answering at 0x%02X (tof_addr)\n", addr);
        return false;
    }
    static const uint8_t mode_args[4] = {0, 0, 0, MATRIX_8X8};
    s_cmd = CMD_SETMODE;
    if (!sendCommand(CMD_SETMODE, mode_args, 4))
    {
        Serial.printf("[tof] 0x%02X ACKed but refused the mode command\n", addr);
        return false;
    }
    s_state = MODE_REPLY;
    s_t0 = millis();
    Serial.printf("[tof] 8x8 ToF (SEN0628) at 0x%02X: 8x8 mode requested, frames in ~%lu s\n",
                  addr, (unsigned long)(MODE_SETTLE_MS / 1000));
    return true;
}

bool tofPresent() { return s_state != OFF; }
uint32_t tofFrames() { return s_frames; }
uint32_t tofErrors() { return s_errors; }

void tofPoll()
{
    const uint32_t now = millis();
    switch (s_state)
    {
    case OFF:
        return;
    case MODE_REPLY:
        if (!pollHeader())
            return;
        // The mode reply carries a short payload: drain it (it is under one chunk).
        if (s_len > 0 && s_len <= CHUNK)
            readBytes(s_rx, s_len);
        s_state = SETTLING;
        s_t0 = now;
        return;
    case SETTLING:
        if (now - s_t0 >= MODE_SETTLE_MS)
            s_state = IDLE;
        return;
    case IDLE:
        if (now - s_last_request < FRAME_PERIOD_MS)
            return;
        s_last_request = now;
        s_cmd = CMD_ALLDATA;
        if (!sendCommand(CMD_ALLDATA, NULL, 0))
        {
            fail();
            return;
        }
        s_state = DATA_REPLY;
        s_t0 = now;
        return;
    case DATA_REPLY:
        if (!pollHeader())
            return;
        if (s_len != TOF_FRAME_BYTES)
        {
            // An 8x8 frame is 128 bytes; anything else (a 4x4 board, a partial reset) is
            // not read as one.
            fail();
            return;
        }
        s_state = DATA_PAYLOAD;
        return;
    case DATA_PAYLOAD:
    {
        // One chunk per call: four calls for a frame, each a ~1 ms transfer at 400 kHz.
        const size_t n = (s_len - s_got) > CHUNK ? CHUNK : (size_t)(s_len - s_got);
        if (!readBytes(s_rx + s_got, n))
        {
            fail();
            return;
        }
        s_got += n;
        if (s_got >= s_len)
        {
            memcpy(s_frame, s_rx, TOF_FRAME_BYTES);
            s_frame_ready = true;
            s_frames++;
            s_fails = 0;
            s_state = IDLE;
        }
        return;
    }
    }
}

bool tofTakeFrame(uint8_t *out)
{
    if (!s_frame_ready)
        return false;
    memcpy(out, s_frame, TOF_FRAME_BYTES);
    s_frame_ready = false;
    return true;
}
