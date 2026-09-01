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

#ifndef MEBDCDC_H
#define MEBDCDC_H


#include "dcdc.h"
#include "my_math.h"
#include "params.h"
#include <stdint.h>

class MEBDCDC : public DCDC {

public:
  void SetCanInterface(CanHardware *c) override;
  void DecodeCAN(int id, uint8_t *data) override;
  void Task10Ms() override;
  void DeInit() override;

  enum DcdcMode {
    MEB_DCDC_STANDBY = 0,
    MEB_DCDC_PRECHARGE = 1,
    MEB_DCDC_BUCK = 2,
    MEB_DCDC_BOOST = 3,
    MEB_DCDC_TEST = 4,
    MEB_DCDC_INIT = 7
  };

private:
  void SendHvk01(bool hvActive);
  void MonitorLiveness();
  void SendAirbag01();
  void SendBms04();
  void SendUdsPing();
  void DecodeUdsResponse(const uint8_t *data);
  static uint8_t Crc8H2F(const uint8_t *data, uint8_t length, uint8_t init,
                         uint8_t finalXor);
  static uint8_t VagCrc(const uint8_t *frame, uint8_t length,
                        const uint8_t *pduConst);
  uint8_t counter = 0;
  uint8_t tick = 0;
  uint8_t udsPollTick = 0;
  bool udsEverSeen = false;
  uint16_t udsSilenceTicks = 0;
};

#endif // MEBDCDC_H
