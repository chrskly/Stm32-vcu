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

#include "MGgen1charger.h"

#define MG1_CMD_ID 0x29C
#define MG1_STATUS_ID 0x3B8

#define MG1_COUNTS_PER_AMP 20
#define MG1_MAX_COUNTS 511

#define MG1_RAMP_COUNTS 2

#define MG1_AC_COUNTS_PER_AMP 4
#define MG1_MAX_AC_LIMIT 0x40

#define MG1_MIN_UDC 50.0f

#define MG1_STAGE1_TICKS 30
#define MG1_STAGE2_TICKS 5
#define MG1_STAGE3_TICKS 30

void MGgen1charger::SetCanInterface(CanHardware *c) {
  can = c;
  can->RegisterUserMessage(MG1_STATUS_ID);
}

void MGgen1charger::DecodeCAN(int id, uint32_t data[2]) {
  if (id != MG1_STATUS_ID)
    return;

  uint8_t *bytes = (uint8_t *)data;

  cpDuty = bytes[0];

  acAmps = bytes[1] * 0.25f;
  acVolts = bytes[2] * 2.0f;

  Param::SetFloat(Param::AC_Volts, acVolts);
  Param::SetFloat(Param::AC_Amps, acAmps);
}

bool MGgen1charger::ControlCharge(bool RunCh, bool ACReq) {
  bool evsePresent = (cpDuty > 0) && (cpDuty < 100);

  if (Param::GetInt(Param::interface) == Unused) {
    active = evsePresent && ACReq;
  } else {
    active = RunCh && ACReq;
  }

  return active;
}

uint8_t MGgen1charger::AcCurrentLimit() {
  int acVoltNom = Param::GetInt(Param::ChgAcVolt);
  if (acVoltNom < 100)
    acVoltNom = 230;

  float amps = Param::GetInt(Param::Pwrspnt) / (float)acVoltNom;
  int counts = amps * MG1_AC_COUNTS_PER_AMP;

  return MIN(MAX(counts, 0), MG1_MAX_AC_LIMIT);
}

uint16_t MGgen1charger::CalcCurrentRequest() {
  float udc = Param::GetFloat(Param::udc);
  float target = 0;

  if (udc > MG1_MIN_UDC)
    target = Param::GetInt(Param::Pwrspnt) / udc;

  target = MIN(target, (float)Param::GetInt(Param::BMS_ChargeLim));
  target = MAX(target, 0.0f);

  int targetCounts = target * MG1_COUNTS_PER_AMP;
  targetCounts = MIN(targetCounts, MG1_MAX_COUNTS);

  if (targetCounts > currentReq)
    currentReq += MIN(MG1_RAMP_COUNTS, targetCounts - currentReq);
  else if (targetCounts < currentReq)
    currentReq -= MIN(MG1_RAMP_COUNTS, currentReq - targetCounts);

  return currentReq;
}

void MGgen1charger::SendCommand(uint16_t req) {
  uint8_t bytes[8];

  bytes[0] = byte0;
  bytes[1] = byte1;
  bytes[2] = (byte2 == 0x24) ? (0x24 | ((req >> 8) & 0x01)) : byte2;
  bytes[3] = req & 0xFF;
  bytes[4] = 0x00;
  bytes[5] = byte5;
  bytes[6] = byte6;
  bytes[7] = byte7;

  can->Send(MG1_CMD_ID, (uint32_t *)bytes, 8);
}

void MGgen1charger::Task100Ms() {
  if (can == 0)
    return;

  if (active) {
    stage3 = MG1_STAGE3_TICKS;
    byte1 = 0x08;

    if (stage1)
      stage1--;

    if (stage1 == 0) {
      byte2 = 0x24;
      byte5 = 0x8C;
      byte6 = 0x5A;
      byte7 = 0x3C;

      if (stage2)
        stage2--;

      if (stage2 == 0)
        byte0 = AcCurrentLimit();
    }
  } else {
    stage1 = MG1_STAGE1_TICKS;
    stage2 = MG1_STAGE2_TICKS;

    byte0 = 0x20;
    byte1 = 0x01;
    byte2 = 0x60;
    byte5 = 0x00;
    byte6 = 0x00;
    byte7 = 0x00;

    if (stage3)
      stage3--;

    if (stage3 == 0) {
      byte0 = 0x00;
      byte1 = 0x00;
      byte2 = 0x00;
    }
  }

  bool running = active && (stage1 == 0) && (stage2 == 0);
  uint16_t req = 0;

  if (running) {
    req = CalcCurrentRequest();
  } else {
    currentReq = 0;
  }

  SendCommand(req);
}

void MGgen1charger::Off() {
  active = false;
  currentReq = 0;
  stage1 = MG1_STAGE1_TICKS;
  stage2 = MG1_STAGE2_TICKS;
  stage3 = 0;
  byte0 = 0x00;
  byte1 = 0x00;
  byte2 = 0x00;
  byte5 = 0x00;
  byte6 = 0x00;
  byte7 = 0x00;

  if (can != 0)
    SendCommand(0);
}

void MGgen1charger::DeInit() {
  Off();
  cpDuty = 0;
  acVolts = 0;
  acAmps = 0;
}
