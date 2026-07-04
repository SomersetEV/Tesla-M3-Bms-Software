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
#include "param_save.h"

// voltage to state of charge                0%    10%   20%   30%   40%   50%
// 60%   70%   80%   90%   100%
uint16_t voltageToSoc[] = {3300, 3400, 3450, 3500, 3560, 3600,
                           3700, 3800, 4000, 4100, 4200};

float SocAccum = 0.0f;  // float accumulator to preserve sub-1% increments
float asDiff = 0;       // amp-second change since last SOC update
int32_t lastAh = 0;     // ISA::Ah value from previous SOC update
static bool socInitialized = false;
static uint32_t socSaveCounter = 0;
static const uint32_t SOC_SAVE_INTERVAL = 3000; // flush to flash every 5 min (3000 x 100ms)

// Coulomb-counter baseline health. The ISA shunt is separately powered and its
// amp-second counter keeps running across our own reboots, so ISA::Ah must not
// be integrated until the first real 0x527 frame has set a baseline. Otherwise
// a reboot debits the shunt's entire cumulative count from SOC again.
static bool ahBaselined = false;
static uint32_t lastAhFrames = 0;
static uint32_t ahStaleTicks = 0;              // ticks since last fresh Ah frame
static const uint32_t AH_STALE_TICKS_WARN = 10; // 1s without Ah data -> flag in socstate

// End-of-charge detection: reset SOC to 100% when charge current tapers off
// while max cell voltage is within eocmargin of CellVmax. Requires a real
// charging phase first so a pack merely resting at high voltage never resets.
enum EocState { EOC_IDLE = 0, EOC_CHARGING, EOC_DONE };
static EocState eocState = EOC_IDLE;
static uint8_t eocDebounce = 0;
static const uint8_t EOC_DEBOUNCE_TICKS = 20; // 2s @ 100ms

// Resting-voltage sanity check on the coulomb counter
static uint32_t restTicks = 0;
static bool restCorrected = false;
static const uint32_t REST_TICKS_REQUIRED = 3000; // 5 min @ 100ms

// One-shot rested-voltage cross-check shortly after boot. Catches a
// catastrophically wrong restore (e.g. SOC stuck at 0 in flash) after the pack
// has been sitting, without waiting the full 5 min rest window.
static uint32_t bootRestTicks = 0;
static bool bootCheckDone = false;
static const uint32_t BOOT_REST_TICKS = 300;   // 30s @ 100ms - enough voltage relaxation
static const float BOOT_SOC_ERR_MIN = 15.0f;   // % - only fix restores that are badly wrong

// umin/umax are only trustworthy when the BMBs are actually reporting. During
// a BMB dropout/re-wake umin reads its 5000mV seed and umax reads 0 - feeding
// those into any voltage-based SOC path snaps SOC to 0 or 100%.
static bool CellDataValid() {
  float umin = Param::GetFloat(Param::umin);
  float umax = Param::GetFloat(Param::umax);
  return Param::GetInt(Param::CellsPresent) > 0 &&
         umin >= 1500.0f && umin <= 4600.0f &&
         umax >= 1500.0f && umax <= 4600.0f &&
         umax >= umin;
}

// idc sign convention: positive = charging (same as ISA::Ah integration)
static void CheckEndOfCharge(float idc, bool cellsValid) {
  if (!cellsValid)
    return; // hold state (incl. debounce) until cell voltages are trustworthy

  float taperA  = Param::GetFloat(Param::eoctaper);
  float vTarget = Param::GetInt(Param::CellVmax) - Param::GetFloat(Param::eocmargin);
  float umax    = Param::GetFloat(Param::umax);

  switch (eocState) {
  case EOC_IDLE:
    if (idc > taperA) {
      eocState = EOC_CHARGING;
      eocDebounce = 0;
    }
    break;
  case EOC_CHARGING:
    if (idc < -taperA) {
      eocState = EOC_IDLE; // discharge started, charge session aborted
    } else if (idc < taperA) {
      if (umax >= vTarget) {
        if (++eocDebounce >= EOC_DEBOUNCE_TICKS) {
          SocAccum = 100.0f;
          eocState = EOC_DONE;
          socSaveCounter = SOC_SAVE_INTERVAL; // flush to flash next cycle
        }
      } else {
        eocState = EOC_IDLE; // charge stopped before reaching top voltage
      }
    } else {
      eocDebounce = 0; // still charging hard
    }
    break;
  case EOC_DONE:
    // latched so the CV tail can't re-fire; re-arm once discharging or relaxed
    if (idc < -taperA || umax < vTarget)
      eocState = EOC_IDLE;
    break;
  }
}

static void CheckRestCorrection(float idc, bool cellsValid) {
  if (ABS(idc) < Param::GetFloat(Param::restcur)) {
    if (restTicks < REST_TICKS_REQUIRED) {
      restTicks++;
    } else if (!restCorrected && cellsValid) { // wait for valid cells, retry next tick
      float vSoc = (float)BMSUtil::EstimateSocFromVoltage();
      if (ABS(vSoc - SocAccum) > Param::GetFloat(Param::socerr))
        SocAccum = vSoc; // snap, coulomb counting continues from here
      restCorrected = true; // at most one correction per rest period
    }
  } else {
    restTicks = 0;
    restCorrected = false;
  }
}

static void CheckBootRestCorrection(float idc, bool cellsValid) {
  if (bootCheckDone)
    return;

  if (ABS(idc) >= Param::GetFloat(Param::restcur)) {
    bootCheckDone = true; // load applied soon after boot: not a rested pack
    return;
  }

  if (!cellsValid)
    return; // BMBs still waking up, keep waiting

  if (++bootRestTicks >= BOOT_REST_TICKS) {
    float vSoc = (float)BMSUtil::EstimateSocFromVoltage();
    float thresh = MAX(BOOT_SOC_ERR_MIN, Param::GetFloat(Param::socerr));
    if (ABS(vSoc - SocAccum) > thresh)
      SocAccum = vSoc;
    bootCheckDone = true;
  }
}

void BMSUtil::UpdateSOC() {
  bool cellsValid = CellDataValid();
  bool isaMode = Param::GetInt(Param::idcmode) == IDC_ISACAN;

  if (!socInitialized) {
    int savedSoc = Param::GetInt(Param::socSaved);
    if (savedSoc > 0 && savedSoc <= 100) {
      SocAccum = (float)savedSoc;
    } else if (cellsValid) {
      // No usable saved SOC (first boot, or 0 saved after a fault): seed from
      // the OCV estimate instead of silently starting at 0%. If the pack
      // really is empty the estimate returns ~0 anyway.
      SocAccum = (float)EstimateSocFromVoltage();
    } else {
      // Nothing trustworthy yet - wait rather than publish garbage.
      Param::SetInt(Param::socstate, 1000);
      return;
    }
    Param::SetInt(Param::soc, (int)SocAccum);
    socInitialized = true;
  }

  if (isaMode) {
    // The shunt's amp-second counter is absolute, so missed frames self-heal:
    // the first diff after a CAN gap is the true consumption during the gap.
    // What must never happen is integrating against a baseline we never took.
    uint32_t ahFrames = ISA::AhFrames;
    bool freshAh = (ahFrames != lastAhFrames);
    lastAhFrames = ahFrames;

    if (freshAh) {
      if (ahBaselined) {
        asDiff = ISA::Ah - lastAh;
        lastAh = ISA::Ah;

        // Plausibility: more amp-seconds than the pack limits could have
        // moved since the last fresh frame means a counter reset (shunt
        // power cycle / RESTART command) or a corrupt frame. Re-baseline
        // without integrating.
        float maxCur = 2.0f * MAX(Param::GetFloat(Param::maxchargecur),
                                  Param::GetFloat(Param::maxdischargecur));
        maxCur = MAX(maxCur, 100.0f); // floor so a zeroed limit can't freeze SOC
        float maxAs = maxCur * 0.1f * (ahStaleTicks + 1);
        int nomcap = Param::GetInt(Param::nomcap);
        if (ABS(asDiff) <= maxAs && nomcap >= 1)
          SocAccum += 100.0f * asDiff / (3600.0f * nomcap);
      } else {
        lastAh = ISA::Ah; // first Ah frame since boot: baseline only
        ahBaselined = true;
      }
      ahStaleTicks = 0;
    } else if (ahStaleTicks < 0xFFFFFF) {
      ahStaleTicks++; // shunt silent; coulomb count holds
    }

    float idc = Param::GetFloat(Param::idc);
    CheckEndOfCharge(idc, cellsValid);        // may set SocAccum = 100
    CheckRestCorrection(idc, cellsValid);     // may snap SocAccum to voltage estimate
    CheckBootRestCorrection(idc, cellsValid); // one-shot sanity check after boot

    if (cellsValid &&
        Param::GetFloat(Param::umax) >= Param::GetInt(Param::CellVmax))
      SocAccum = 100.0f; // exact-top backstop
  } else {
    if (cellsValid)
      SocAccum = EstimateSocFromVoltage();
    // else: hold last SOC until cell data returns
  }

  if      (SocAccum > 100.0f) SocAccum = 100.0f;
  else if (SocAccum <   0.0f) SocAccum = 0.0f;

  Param::SetInt(Param::soc, (int)SocAccum);
  Param::SetInt(Param::socSaved, (int)SocAccum);

  int state = (int)eocState + (restTicks >= REST_TICKS_REQUIRED ? 10 : 0);
  if (isaMode && ahStaleTicks > AH_STALE_TICKS_WARN)
    state += 100; // no Ah data from shunt for >1s
  if (!cellsValid)
    state += 200; // BMB cell data invalid/missing
  Param::SetInt(Param::socstate, state);

  if (++socSaveCounter >= SOC_SAVE_INTERVAL) {
    socSaveCounter = 0;
    parm_save();
  }
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