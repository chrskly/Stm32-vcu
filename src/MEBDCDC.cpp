/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2026 Damien Maguire <info@evbmw.com>
 *                    Christian Kelly <zombie@chrskly.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "MEBDCDC.h"
#include "errormessage.h"

// CAN message IDs
#define MEB_HVK_01 0x503
#define MEB_AIRBAG_01 0x040
#define MEB_BMS_04 0x5A2

// UDS message IDs
#define MEB_UDS_REQ 0x17FC00B9
#define MEB_UDS_RESP 0x17FE00B9

#define MEB_TX_TICKS 1

// Timing of UDS-based checks
#define MEB_UDS_POLL_TICKS 100 // 1s
#define MEB_UDS_TIMEOUT_TICKS 300 // 3s

static const uint8_t HVK_01_PDU_CONST[16] = {
  0xED, 0xD6, 0x96, 0x63, 0xA5, 0x12, 0xD5, 0x9A,
  0x1E, 0x0D, 0x24, 0xCD, 0x8C, 0xA6, 0x2F, 0x41
};

static const uint8_t AIRBAG_01_PDU_CONST[16] = {
  0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40,
  0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40
};

static const uint8_t BMS_04_PDU_CONST[16] = {
  0xEB, 0x4C, 0x44, 0xAF, 0x21, 0x8D, 0x01, 0x58,
  0xFA, 0x93, 0xDB, 0x89, 0x15, 0x10, 0x4A, 0x61
};

uint8_t MEBDCDC::Crc8H2F(const uint8_t *data, uint8_t length, uint8_t init,
                         uint8_t finalXor) {
  uint8_t crc = init;

  for (uint8_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x80)
        crc = (uint8_t)((crc << 1) ^ 0x2F);
      else
        crc = (uint8_t)(crc << 1);
    }
  }

  return crc ^ finalXor;
}

uint8_t MEBDCDC::VagCrc(const uint8_t *frame, uint8_t length,
                        const uint8_t *pduConst) {
  uint8_t cnt = frame[1] & 0x0F;
  uint8_t crc = Crc8H2F(&frame[1], length - 1, 0xFF, 0x00);

  return Crc8H2F(&pduConst[cnt], 1, crc, 0xFF);
}

void MEBDCDC::SetCanInterface(CanHardware *c) {
  can = c;
  can->RegisterUserMessage(MEB_UDS_RESP);
}

void MEBDCDC::DecodeCAN(int id, uint8_t *data) {
  if (id == MEB_UDS_RESP) {
    DecodeUdsResponse(data);
  }
}

void MEBDCDC::SendHvk01(bool hvActive) {
  uint8_t bytes[8];

  bytes[0] = 0x00; // CRC
  bytes[1] = 0x00; // counter and mode request
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  bytes[4] = 0x7F;
  bytes[5] = 0x80;
  bytes[6] = 0xE3;
  bytes[7] = 0x03;

  if (hvActive) {
    bytes[1] = 0x30;
    bytes[3] = (uint8_t)((MEB_DCDC_BUCK << 3) | 0x01);
    bytes[5] = 0x82;
    bytes[6] = 0xE0;
  } else {
    bytes[1] = 0x10;
    bytes[3] = (uint8_t)((MEB_DCDC_STANDBY << 3) | 0);
    bytes[5] = 0x80;
    bytes[6] = 0xE3;
  }

  bytes[1] = (uint8_t)((bytes[1] & 0xF0) | counter);
  bytes[0] = VagCrc(bytes, 8, HVK_01_PDU_CONST);

  can->Send(MEB_HVK_01, (uint32_t *)bytes, 8);
}

void MEBDCDC::SendAirbag01() {
  uint8_t bytes[8];

  bytes[0] = 0x00;
  bytes[1] = 0x00;
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x00;

  bytes[1] = (uint8_t)((bytes[1] & 0xF0) | counter);
  bytes[0] = VagCrc(bytes, 8, AIRBAG_01_PDU_CONST);

  can->Send(MEB_AIRBAG_01, (uint32_t *)bytes, 8);
}

void MEBDCDC::SendBms04() {
  uint8_t bytes[8];

  bytes[0] = 0x00;
  bytes[1] = 0x00;
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x00;

  bytes[1] = (uint8_t)((bytes[1] & 0xF0) | counter);
  bytes[0] = VagCrc(bytes, 8, BMS_04_PDU_CONST);

  can->Send(MEB_BMS_04, (uint32_t *)bytes, 8);
}

// Send UDS message to the DCDC module to check liveness
void MEBDCDC::SendUdsPing() {
  uint8_t bytes[8] = { 0x02, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  can->Send(MEB_UDS_REQ, (uint32_t *)bytes, 8);
}

/* Process UDS liveness response. Doesn't matter if it's postive or negative,
 * either way the module has responded so is there. This doesn't tell us that
 * it's working, just that it's alive. */
void MEBDCDC::DecodeUdsResponse(const uint8_t *data) {
  bool positive = (data[0] == 0x02) && (data[1] == 0x7E);
  bool negative = (data[0] == 0x03) && (data[1] == 0x7F) && (data[2] == 0x3E);

  if (!positive && !negative)
    return;

  udsEverSeen = true;
  udsSilenceTicks = 0;
}

// Decide whether the converter is still there.
void MEBDCDC::MonitorLiveness() {
  if (!udsEverSeen || udsSilenceTicks >= MEB_UDS_TIMEOUT_TICKS) {
    return;
  }

  if (++udsSilenceTicks >= MEB_UDS_TIMEOUT_TICKS) {
    ErrorMessage::Post(ERR_DCDCFAULT);
  }
}

void MEBDCDC::Task10Ms() {
  if (can == 0)
    return;

  int opmode = Param::GetInt(Param::opmode);
  bool hvActive = (opmode == MOD_RUN) || (opmode == MOD_CHARGE) ||
                  (opmode == MOD_MAINTAIN) || (opmode == MOD_PREHEAT);

  MonitorLiveness();

  if (++udsPollTick >= MEB_UDS_POLL_TICKS) {
    udsPollTick = 0;
    SendUdsPing();
  }

  if (++tick < MEB_TX_TICKS)
    return;

  tick = 0;

  SendHvk01(hvActive);
  SendAirbag01();
  SendBms04();

  counter = (uint8_t)((counter + 1) & 0x0F);
}

void MEBDCDC::DeInit() {
  counter = 0;
  tick = 0;
  udsPollTick = 0;
  udsEverSeen = false;
  udsSilenceTicks = 0;
  Param::SetInt(Param::DcdcUdsAlive, 0);
}
