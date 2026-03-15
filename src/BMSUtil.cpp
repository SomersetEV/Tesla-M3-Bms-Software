/*
 *
 * Copyright (C) 2023 Tom de Bree
 *                      Damien Maguire <info@evbmw.com>
 * Yes I'm really writing software now........run.....run away.......
 *
 * Based on info from https://github.com/jsphuebner/FlyingAdcBms
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
#include "BMSUtil.h"
#include "isa_shunt.h"

// voltage to state of charge                0%    10%   20%   30%   40%   50%
// 60%   70%   80%   90%   100%
uint16_t voltageToSoc[] = {3300, 3400, 3450, 3500, 3560, 3600,
                           3700, 3800, 4000, 4100, 4200};

uint16_t TempSOC = 0;      // SOC for internal code use
uint32_t NoCurCounter = 0; //
uint32_t NoCurRun =
    20; // 100ms counts before SOC will be determined based on voltage
float NoCurLim = 1.0; // current limit under which Voltage based
float asDiff = 0;     // Ampsecond change since last SOC update
int32_t lastAh = 0;  // ISA::Ah value from previous SOC update

void BMSUtil::UpdateSOC() {
  TempSOC = Param::GetInt(Param::soc);
  asDiff = ISA::Ah - lastAh;
  lastAh = ISA::Ah;

  if (ABS(Param::GetFloat(Param::idc)) < NoCurLim) {
    NoCurCounter++;
  } else {
    NoCurCounter = 0;
  }

  if (NoCurCounter > NoCurRun) {
    TempSOC = EstimateSocFromVoltage();
  } else {
    TempSOC = TempSOC + (100 * asDiff / (3600 * Param::GetInt(Param::nomcap)));
  }

  Param::SetInt(Param::soc, TempSOC);
}

int BMSUtil::EstimateSocFromVoltage() {
  float lowestVoltage = Param::GetFloat(Param::umin);
  int n = sizeof(voltageToSoc) / sizeof(voltageToSoc[0]);

  for (int i = 0; i < n; i++) {
    if (lowestVoltage < voltageToSoc[i]) {
      if (i == 0)
        return 0;

      float soc = i * 10;
      float lutDiff = voltageToSoc[i] - voltageToSoc[i - 1];
      float valDiff = voltageToSoc[i] - lowestVoltage;
      // interpolate
      soc -= (valDiff / lutDiff) * 10;
      return soc;
    }
  }
  return 100;
}

float BMSUtil::ProcessUdc() {
  float udc = Param::GetFloat(Param::udc);

  if (Param::GetInt(Param::idcmode) == IDC_OFF) {
    // no current sensor.
  } else if (Param::GetInt(Param::idcmode) == IDC_ISACAN) // ISA shunt
  {
    float udc =
        ((float)ISA::Voltage) /
        1000; // get voltage from isa sensor and post to parameter database
    Param::SetFloat(Param::udc, udc);
    float udc2 =
        ((float)ISA::Voltage2) /
        1000; // get voltage from isa sensor and post to parameter database
    Param::SetFloat(Param::udc2, udc2);
    float udc3 =
        ((float)ISA::Voltage3) /
        1000; // get voltage from isa sensor and post to parameter database
    Param::SetFloat(Param::udc3, udc3);
    float idc =
        ((float)ISA::Amperes) /
        1000; // get current from isa sensor and post to parameter database
    Param::SetFloat(Param::idc, idc);
    float kw = ((float)ISA::KW) /
               1000; // get power from isa sensor and post to parameter database
    Param::SetFloat(Param::power, kw);
    float kwh = ((float)ISA::KWh) /
                1000; // get kwh from isa sensor and post to parameter database
    Param::SetFloat(Param::KWh, kwh);
    float Amph = ((float)ISA::Ah) /
                 3600; // get Ah from isa sensor and post to parameter database
    Param::SetFloat(Param::AMPh, Amph);
    float deltaVolts1 = (udc2 / 2) - udc3;
    Param::SetFloat(Param::deltaV, deltaVolts1);
  }
  return udc;
}