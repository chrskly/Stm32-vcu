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

#ifndef MGgen1charger_H
#define MGgen1charger_H

#include "chargerhw.h"
#include "my_fp.h"
#include "my_math.h"
#include "params.h"
#include <stdint.h>

class MGgen1charger : public Chargerhw {

public:
  void SetCanInterface(CanHardware *c) override;
  void DecodeCAN(int id, uint32_t data[2]) override;
  void Task100Ms() override;
  void Off() override;
  bool ControlCharge(bool RunCh, bool ACReq) override;
  void DeInit() override;

private:
  void SendCommand(uint16_t req);
  uint16_t CalcCurrentRequest();
  uint8_t AcCurrentLimit();

  uint8_t byte0 = 0x00;
  uint8_t byte1 = 0x00;
  uint8_t byte2 = 0x00;
  uint8_t byte5 = 0x00;
  uint8_t byte6 = 0x00;
  uint8_t byte7 = 0x00;

  uint16_t currentReq = 0;
  bool active = false;

  uint8_t stage1 = 30;
  uint8_t stage2 = 5;
  uint8_t stage3 = 30;

  uint8_t cpDuty = 0;
  float acVolts = 0;
  float acAmps = 0;
};

#endif // MGgen1charger_H
