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

#include "app/autolog.h"

#ifdef ENABLE_AUTO_LOG

#include "external/printf/printf.h"
#include "misc.h"
#include "radio.h"
#include "settings.h"

AutoLog_t gAutoLogs[MAX_AUTO_LOGS];
uint8_t   gAutoLogCount;
bool      gAutoLogMode;
uint8_t   gAutoLogScanMode = AUTOLOG_SCAN_FAST;

void AUTOLOG_Flush(void)
{
	if (gAutoLogCount == 0) {
		gAutoLogMode = false;
		return;
	}

	VFO_Info_t vfo;
	uint8_t    chan = MR_CHANNEL_FIRST;

	// Two-pass partition: CSS-detected hits first (most-interesting → lowest
	// channel numbers), then non-CSS hits.
	for (uint8_t pass = 0; pass < 2; pass++) {
		const bool want_css = (pass == 0);
		for (uint8_t i = 0; i < gAutoLogCount; i++) {
			if ((gAutoLogs[i].code_type != CODE_TYPE_OFF) != want_css)
				continue;

			while (chan <= MR_CHANNEL_LAST && RADIO_CheckValidChannel(chan, false, 0))
				chan++;
			if (chan > MR_CHANNEL_LAST)
				goto done;

			const AutoLog_t *log = &gAutoLogs[i];
			RADIO_InitInfo(&vfo, chan, log->frequency);

			if ((log->code_type != CODE_TYPE_OFF)) {
				vfo.freq_config_RX.CodeType = log->code_type;
				vfo.freq_config_RX.Code     = log->code_val;
				vfo.freq_config_TX          = vfo.freq_config_RX;

				// US ham band-plan offsets, only when CSS was detected.
				// Frequencies are in 10 Hz units.
				uint32_t txOffset = 0;
				if (log->frequency >= 14400000 && log->frequency < 14800000)
					txOffset = 60000;
				else if (log->frequency >= 42000000 && log->frequency < 45000000)
					txOffset = 500000;

				if (txOffset != 0) {
					vfo.TX_OFFSET_FREQUENCY           = txOffset;
					vfo.TX_OFFSET_FREQUENCY_DIRECTION = TX_OFFSET_FREQUENCY_DIRECTION_SUB;
					vfo.freq_config_TX.Frequency      = log->frequency - txOffset;
				}
			}

			SETTINGS_SaveChannel(chan, gEeprom.TX_VFO, &vfo, 2);

			// Channel-name annotation. Examples:
			//   "67.0"       CTCSS 67.0 Hz, FM (implicit), heard once
			//   "67.0 x3"    CTCSS 67.0 Hz, heard 3 times
			//   "D023"       DCS code 023
			//   "AM"         aviation-band hit (108-137 MHz), heard once
			//   "AM x3"      aviation, heard 3 times
			//   "x5"         simplex FM, heard 5 times
			//   ""           simplex FM, heard once
			//
			// FM is implicit (default). AM is inferred from the aviation
			// band — the BK4819 can't actually detect modulation, but
			// anything in 108-137 MHz is overwhelmingly AM in practice.
			char  name[11] = {0};
			char *p        = name;
			if (log->frequency >= 10800000 && log->frequency < 13700000)
				p += sprintf(p, "AM");
			if ((log->code_type != CODE_TYPE_OFF)) {
				if (p != name) *p++ = ' ';
				if (log->code_type == CODE_TYPE_CONTINUOUS_TONE) {
					const uint16_t hz10 = CTCSS_Options[log->code_val];
					p += sprintf(p, "%u.%u", hz10 / 10, hz10 % 10);
				} else {
					p += sprintf(p, "D%03o", DCS_Options[log->code_val]);
				}
			}
			if (log->hit_count > 1) {
				p += sprintf(p, "x%u", log->hit_count);
			}
			// dBm suffix with always-present sign (sign IS the separator).
			// Worst-case 4 chars ("-160"); only emit if there's room for it.
			if ((p - name) <= 6) {
				sprintf(p, "%+d", (int)log->rssi - 160);
			}
			SETTINGS_SaveChannelName(chan, name);

			chan++;
		}
	}
done:
	gAutoLogCount = 0;
	gAutoLogMode  = false;
}

#endif
