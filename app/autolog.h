/* Copyright 2023 Dual Tachyon
 * https://github.com/DualTachyon
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *     Unless required by applicable law or agreed to in writing, software
 *     distributed under the License is distributed on an "AS IS" BASIS,
 *     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *     See the License for the specific language governing permissions and
 *     limitations under the License.
 */

#ifndef APP_AUTOLOG_H
#define APP_AUTOLOG_H

#ifdef ENABLE_AUTO_LOG

#include <stdbool.h>
#include <stdint.h>

#include "dcs.h"

#define MAX_AUTO_LOGS 64

typedef struct {
	uint32_t       frequency;
	DCS_CodeType_t code_type;   // CODE_TYPE_OFF means no CSS detected
	uint8_t        code_val;
	uint8_t        hit_count;   // total times this freq was rediscovered during scan
	uint8_t        rssi;        // dBm + 160 (so 0..255 stores -160..+95 dBm)
} AutoLog_t;

extern AutoLog_t gAutoLogs[MAX_AUTO_LOGS];
extern uint8_t   gAutoLogCount;
extern bool      gAutoLogMode;

// Scan strategy:
//   FAST = BK4819 hardware frequency counter (wideband, fast, less sensitive)
//   SLOW = normal RX with full per-channel setup + interrupt-driven squelch
//          detection (full sensitivity, ~90 ms per channel)
//   RSSI = like the fagci spectrum: lightweight per-channel retune + read
//          RSSI register directly. Roughly 10× faster than SLOW with full
//          sensitivity. CTCSS detection happens once a signal is found.
enum {
	AUTOLOG_SCAN_FAST = 0,
	AUTOLOG_SCAN_SLOW = 1,
	AUTOLOG_SCAN_RSSI = 2,
};
extern uint8_t gAutoLogScanMode;

// SLOW-mode scan range scope. ENABLE_SCAN_RANGES (if user has set a range
// via VFO A/B) overrides any of these.
//   ALL  = sweep every firmware band
//   HAM  = US 2m (144-148 MHz) + 70cm (420-450 MHz)
//   RPTR = US ham repeater output sub-bands only:
//          145.20-145.50, 146.61-147.39, 442.00-445.00, 447.00-450.00
enum {
	AUTOLOG_RANGE_ALL  = 0,
	AUTOLOG_RANGE_HAM  = 1,
	AUTOLOG_RANGE_RPTR = 2,
};
extern uint8_t gAutoLogSlowRange;

// Logging filter. ALL = log everything that opens squelch. CSS = only log
// hits where CTCSS/DCS was identified (skips simplex / no-tone hits).
enum {
	AUTOLOG_FILTER_ALL = 0,
	AUTOLOG_FILTER_CSS = 1,
};
extern uint8_t gAutoLogFilter;

// RSSI-mode trigger thresholds (raw RSSI values; dBm = raw/2 - 160).
//   LOW  = -115 dBm (raw 90)  — catches very weak; noisier
//   MID  = -105 dBm (raw 110) — default; ~S4
//   HIGH = -90  dBm (raw 140) — strong signals only; NOAA S6 still triggers
enum {
	AUTOLOG_RSSI_TH_LOW  = 0,
	AUTOLOG_RSSI_TH_MID  = 1,
	AUTOLOG_RSSI_TH_HIGH = 2,
};
extern uint8_t  gAutoLogRssiTh;
extern const uint8_t autolog_rssi_thresholds[3];

void AUTOLOG_Flush(void);

#endif

#endif
