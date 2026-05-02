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

float SocAccum = 0.0f;  // float accumulator to preserve sub-1% increments
float asDiff = 0;       // amp-second change since last SOC update
int32_t lastAh = 0;     // ISA::Ah value from previous SOC update

void BMSUtil::UpdateSOC() {
  if (Param::GetInt(Param::idcmode) == IDC_ISACAN) {
    asDiff = ISA::Ah - lastAh;
    lastAh = ISA::Ah;

    if (Param::GetFloat(Param::umax) >= Param::GetInt(Param::CellVmax)) {
      SocAccum = 100.0f;
    } else {
      SocAccum += 100.0f * asDiff / (3600.0f * Param::GetInt(Param::nomcap));
    }
  } else {
    SocAccum = EstimateSocFromVoltage();
  }

  if      (SocAccum > 100.0f) SocAccum = 100.0f;
  else if (SocAccum <   0.0f) SocAccum = 0.0f;

  Param::SetInt(Param::soc, (int)SocAccum);
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

void BMSUtil::UpdateChargeLimits() {
  float tempMin  = Param::GetFloat(Param::TempMin);
  float umax     = Param::GetFloat(Param::umax);
  float cellVmax = Param::GetInt(Param::CellVmax);
  float maxCur   = Param::GetFloat(Param::maxchargecur);
  float tempFactor, voltFactor;

  if      (tempMin < -10.0f) tempFactor = 0.0f;
  else if (tempMin <   0.0f) tempFactor = 0.05f + (tempMin + 10.0f) * 0.005f;
  else if (tempMin <   5.0f) tempFactor = 0.10f + tempMin            * 0.020f;
  else if (tempMin <  10.0f) tempFactor = 0.20f + (tempMin -  5.0f) * 0.040f;
  else if (tempMin <  25.0f) tempFactor = 0.40f + (tempMin - 10.0f) * 0.040f;
  else if (tempMin <  35.0f) tempFactor = 1.0f;
  else if (tempMin <  40.0f) tempFactor = 1.0f  - (tempMin - 35.0f) * 0.050f;
  else if (tempMin <  45.0f) tempFactor = 0.75f - (tempMin - 40.0f) * 0.050f;
  else if (tempMin <  50.0f) tempFactor = 0.50f - (tempMin - 45.0f) * 0.060f;
  else                       tempFactor = 0.0f;

  if      (umax >= cellVmax) voltFactor = 0.0f;
  else if (umax >= 4050.0f)  voltFactor = (cellVmax - umax) / (cellVmax - 4050.0f);
  else                       voltFactor = 1.0f;

  Param::SetFloat(Param::chargelim, maxCur * tempFactor * voltFactor);
}

void BMSUtil::UpdateDischargeLimits() {
  float tempMin = Param::GetFloat(Param::TempMin);
  float tempMax = Param::GetFloat(Param::TempMax);
  float maxCur  = Param::GetFloat(Param::maxdischargecur);
  float coldFactor, hotFactor;

  if      (tempMin < -20.0f) coldFactor = 0.0f;
  else if (tempMin < -10.0f) coldFactor = 0.15f + (tempMin + 20.0f) * 0.025f;
  else if (tempMin <   0.0f) coldFactor = 0.40f + (tempMin + 10.0f) * 0.025f;
  else if (tempMin <   5.0f) coldFactor = 0.65f + tempMin            * 0.070f;
  else                       coldFactor = 1.0f;

  if      (tempMax > 60.0f)  hotFactor = 0.0f;
  else if (tempMax > 55.0f)  hotFactor = 0.25f + (60.0f - tempMax) * 0.050f;
  else if (tempMax > 50.0f)  hotFactor = 0.50f + (55.0f - tempMax) * 0.050f;
  else if (tempMax > 45.0f)  hotFactor = 0.75f + (50.0f - tempMax) * 0.050f;
  else                       hotFactor = 1.0f;

  Param::SetFloat(Param::dischargelim, maxCur * coldFactor * hotFactor);
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