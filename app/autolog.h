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

// Scan strategy: FAST uses the BK4819 hardware frequency counter (wideband,
// fast, but lower sensitivity). SLOW uses normal RX with channel stepping
// (slower but full receiver sensitivity — catches NOAA etc.).
enum {
	AUTOLOG_SCAN_FAST = 0,
	AUTOLOG_SCAN_SLOW = 1,
};
extern uint8_t gAutoLogScanMode;

void AUTOLOG_Flush(void);

#endif

#endif
