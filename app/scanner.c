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

#include "app/app.h"
#include "app/dtmf.h"
#include "app/generic.h"
#include "app/menu.h"
#include "app/chFrScanner.h"
#include "app/scanner.h"
#include "driver/systick.h"
#ifdef ENABLE_AUTO_LOG
	#include "app/autolog.h"
#endif
#include "audio.h"
#include "driver/bk4819.h"
#include "frequencies.h"
#include "misc.h"
#include "radio.h"
#include "settings.h"
#include "ui/inputbox.h"
#include "ui/ui.h"

DCS_CodeType_t    gScanCssResultType;
uint8_t           gScanCssResultCode;
bool              gScanSingleFrequency; // scan CTCSS/DCS codes for current frequency
SCAN_SaveState_t  gScannerSaveState;
uint8_t           gScanChannel;
uint32_t          gScanFrequency;
SCAN_CssState_t   gScanCssState;
uint8_t           gScanProgressIndicator;
bool              gScanUseCssResult;

STEP_Setting_t    stepSetting;
uint8_t           scanHitCount;


#ifdef ENABLE_AUTO_LOG
// LNA multiplex state: while in STATE_OFF we alternate VHF/UHF LNA every 2 s
// so the freq counter has full sensitivity in each band. Starting OFF (both
// disabled) is matched to what SCANNER_Start sets up; first 500ms tick flips
// to VHF, second flips to UHF, etc.
static uint8_t autolog_lna_phase;  // 0=off, 1=VHF, 2=UHF (cycles 1→2→1→...)

// dBm (offset by 160) captured at the moment a SLOW/RSSI hit is detected.
// Reading RSSI in STATE_SCANNING (CSS scan mode) is unreliable, so we save
// the value while still in normal RX and use it at record time.
static uint8_t autolog_hit_rssi;

// Last freq the RSSI sweep tuned the chip to. Used to skip redundant retunes
// across consecutive STATE_OFF ticks. Reset to 0 whenever the chip config is
// disturbed (e.g., transition to STATE_SCANNING) so the next sweep iteration
// re-tunes cleanly.
static uint32_t rssi_tuned_freq;

// Configure the BK4819 for spectrum-style RSSI sweep. Mirrors ToggleRX(false)
// from app/spectrum.c: audio path off, AFDAC bit off, AFBit off, and writes
// the specific REG_43 value spectrum uses for scanning (different from what
// BK4819_SetFilterBandwidth produces). This is the critical chip state that
// makes RSSI reads valid during sweep instead of saturated/stuck.
static void autolog_rssi_sweep_setup(void) {
	AUDIO_AudioPathOff();
	// AFDAC off — REG_30 bit 9.
	{
		const uint16_t r = BK4819_ReadRegister(BK4819_REG_30);
		BK4819_WriteRegister(BK4819_REG_30, r & ~(1u << 9));
	}
	// AFBit off — REG_47 bit 8.
	{
		const uint16_t r = BK4819_ReadRegister(BK4819_REG_47);
		BK4819_WriteRegister(BK4819_REG_47, r & ~(1u << 8));
	}
	// REG_43 scan-mode bandwidth (25 kHz integration window).
	BK4819_WriteRegister(BK4819_REG_43, 0b0011011000101000);
	// Force re-tune on the next sweep tick.
	rssi_tuned_freq = 0;
}

// US ham repeater output sub-bands (frequencies in 10 Hz units).
// Used by AUTOLOG_RANGE_RPTR — restricts SLOW sweep to where repeaters live.
typedef struct { uint32_t lower; uint32_t upper; } slow_range_t;
static const slow_range_t repeater_ranges[] = {
	{14520000, 14550000},  // 2m  145.20-145.50 MHz
	{14661000, 14739000},  // 2m  146.61-147.39 MHz
	{44200000, 44500000},  // 70cm 442.00-445.00 MHz
	{44700000, 45000000},  // 70cm 447.00-450.00 MHz
};

// Advance freq for SLOW mode based on the current range setting. Returns
// the next frequency to tune to. Caller updates gRxVfo and reconfigures.
static uint32_t slow_advance_freq(void)
{
	uint32_t f = gRxVfo->freq_config_RX.Frequency + gRxVfo->StepFrequency;

#ifdef ENABLE_SCAN_RANGES
	if (gScanRangeStart) {
		f = APP_SetFreqByStepAndLimits(gRxVfo, 1,
		        gScanRangeStart, gScanRangeStop);
		gRxVfo->Band = FREQUENCY_GetBand(f);
		return f;
	}
#endif

	if (gAutoLogSlowRange == AUTOLOG_RANGE_HAM) {
		if (f >= 14800000 && f < 42000000)
			f = 42000000;   // jump past gap from 2m end to 70cm start
		else if (f >= 45000000)
			f = 14400000;   // wrap 70cm end → 2m start
		gRxVfo->Band = FREQUENCY_GetBand(f);
		return f;
	}

	if (gAutoLogSlowRange == AUTOLOG_RANGE_RPTR) {
		// Step into the next repeater sub-band when we walk off the end
		// of the current one (or jump in from outside all of them).
		for (uint8_t i = 0; i < ARRAY_SIZE(repeater_ranges); i++) {
			if (f < repeater_ranges[i].upper) {
				if (f < repeater_ranges[i].lower)
					f = repeater_ranges[i].lower;
				gRxVfo->Band = FREQUENCY_GetBand(f);
				return f;
			}
		}
		f = repeater_ranges[0].lower;  // past last range — wrap
		gRxVfo->Band = FREQUENCY_GetBand(f);
		return f;
	}

	// AUTOLOG_RANGE_ALL: sweep every firmware band, wrapping at boundaries.
	if (f >= frequencyBandTable[gRxVfo->Band].upper) {
		uint8_t next = gRxVfo->Band + 1;
		if (next >= BAND_N_ELEM)
			next = 0;
		f             = frequencyBandTable[next].lower;
		gRxVfo->Band  = next;
	}
	return f;
}

static void AUTOLOG_RecordAndResume(void)
{
	if (!gAutoLogMode || gCssBackgroundScan)
		return;

	// Round to 2.5 kHz before dedup/storage. The BK4819 frequency counter
	// has ~200 Hz of jitter, so 146.5 reads as 146.4998 / 146.5001 / etc.
	// across passes — exact-match dedup would log each as a "new" channel.
	const uint32_t freq = FREQUENCY_RoundToStep(gScanFrequency, 250);

	bool dup = false;
	for (uint8_t i = 0; i < gAutoLogCount; i++) {
		if (gAutoLogs[i].frequency == freq) {
			gAutoLogs[i].hit_count++;
			dup = true;
			break;
		}
	}

	if (!dup) {
		// Filter: when CSS-only, drop hits without CTCSS/DCS identified.
		// The scan still advances (handled below) — we just don't record.
		if (gAutoLogFilter == AUTOLOG_FILTER_CSS && !gScanUseCssResult) {
			// fall through to the state reset + advance below
		} else if (gAutoLogCount >= MAX_AUTO_LOGS) {
			// buffer full: stop sweeping, leave state at FOUND so the
			// user's EXIT press routes through SCANNER_Stop for flush.
			gBeepToPlay = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
			return;
		} else {

		gAutoLogs[gAutoLogCount].frequency = freq;
		gAutoLogs[gAutoLogCount].code_type = gScanUseCssResult ? gScanCssResultType : CODE_TYPE_OFF;
		gAutoLogs[gAutoLogCount].code_val  = gScanUseCssResult ? gScanCssResultCode : 0;
		gAutoLogs[gAutoLogCount].hit_count = 1;
		// Signal strength. SLOW/RSSI modes captured this at the moment of
		// detection (normal RX); FAST reads it fresh from the chip's
		// freq-counter path. dBm offset by 160 so it fits in a uint8_t.
		gAutoLogs[gAutoLogCount].rssi = (gAutoLogScanMode == AUTOLOG_SCAN_FAST)
			? (uint8_t)(BK4819_GetRSSI_dBm() + 160)
			: autolog_hit_rssi;
		gAutoLogCount++;
		gBeepToPlay = BEEP_1KHZ_60MS_OPTIONAL;
		}
	}

	// kick state machine back to phase 1 (frequency hunt).
	gScanCssState          = SCAN_CSS_STATE_OFF;
	gScanCssResultCode     = 0xFF;
	gScanCssResultType     = 0xFF;
	gScanUseCssResult      = false;
	scanHitCount           = 0;
	gScanProgressIndicator = 0;

	if (gAutoLogScanMode == AUTOLOG_SCAN_SLOW ||
	    gAutoLogScanMode == AUTOLOG_SCAN_RSSI) {
		// SLOW/RSSI: advance to next freq so a continuous carrier doesn't
		// re-trigger us forever. RADIO_SetupRegisters here restores normal
		// RX state (was put into CSS scan mode by the STATE_SCANNING path).
		gRxVfo->freq_config_RX.Frequency = slow_advance_freq();
		RADIO_ApplyOffset(gRxVfo);
		RADIO_ConfigureSquelchAndOutputPower(gRxVfo);
		RADIO_SetupRegisters(true);
		// RSSI mode: re-enter spectrum-style sweep config so the next
		// STATE_OFF tick gets valid RSSI readings.
		if (gAutoLogScanMode == AUTOLOG_SCAN_RSSI)
			autolog_rssi_sweep_setup();
	} else {
		// FAST: restart the wideband freq counter
		gScanFrequency = 0xFFFFFFFF;
		BK4819_EnableFrequencyScan();
	}
	gScanDelay_10ms = scan_delay_10ms;
}
#endif


static void SCANNER_Key_DIGITS(KEY_Code_t Key, bool bKeyPressed, bool bKeyHeld)
{
	if (!bKeyHeld && bKeyPressed)
	{
		if (gScannerSaveState == SCAN_SAVE_CHAN_SEL) {
			gBeepToPlay = BEEP_1KHZ_60MS_OPTIONAL;

			INPUTBOX_Append(Key);

			gRequestDisplayScreen = DISPLAY_SCANNER;

			if (gInputBoxIndex < 3) {
#ifdef ENABLE_VOICE
				gAnotherVoiceID = (VOICE_ID_t)Key;
#endif
				return;
			}

			gInputBoxIndex = 0;

			uint16_t chan = ((gInputBox[0] * 100) + (gInputBox[1] * 10) + gInputBox[2]) - 1;
			if (IS_MR_CHANNEL(chan)) {
#ifdef ENABLE_VOICE
				gAnotherVoiceID = (VOICE_ID_t)Key;
#endif
				gShowChPrefix = RADIO_CheckValidChannel(chan, false, 0);
				gScanChannel  = (uint8_t)chan;
				return;
			}
		}

		gBeepToPlay = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
	}
}

static void SCANNER_Key_EXIT(bool bKeyPressed, bool bKeyHeld)
{
	if (!bKeyHeld && bKeyPressed) { // short pressed
		gBeepToPlay = BEEP_1KHZ_60MS_OPTIONAL;

		switch (gScannerSaveState) {
			case SCAN_SAVE_NO_PROMPT:
				SCANNER_Stop();
				gRequestDisplayScreen    = DISPLAY_MAIN;
				break;

			case SCAN_SAVE_CHAN_SEL:
				if (gInputBoxIndex > 0) {
					gInputBox[--gInputBoxIndex] = 10;
					gRequestDisplayScreen       = DISPLAY_SCANNER;
					break;
				}

				// Fallthrough

			case SCAN_SAVE_CHANNEL:
				gScannerSaveState     = SCAN_SAVE_NO_PROMPT;
#ifdef ENABLE_VOICE
				gAnotherVoiceID   = VOICE_ID_CANCEL;
#endif
				gRequestDisplayScreen = DISPLAY_SCANNER;
				break;
		}
	}
}

static void SCANNER_Key_MENU(bool bKeyPressed, bool bKeyHeld)
{
	if (bKeyHeld || !bKeyPressed) // ignore long press or release button events
		return;

	if (gScanCssState == SCAN_CSS_STATE_OFF && !gScanSingleFrequency) {
		gBeepToPlay = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
		return;
	}

	if (gScanCssState == SCAN_CSS_STATE_SCANNING && gScanSingleFrequency) {
		gBeepToPlay = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
		return;
	}

	if (gScanCssState == SCAN_CSS_STATE_FAILED) {
		gBeepToPlay = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
		return;
	}

	gBeepToPlay = BEEP_1KHZ_60MS_OPTIONAL;

	switch (gScannerSaveState) {
		case SCAN_SAVE_NO_PROMPT:
			if (!gScanSingleFrequency)
			{
				uint32_t freq250  = FREQUENCY_RoundToStep(gScanFrequency, 250);
				uint32_t freq625  = FREQUENCY_RoundToStep(gScanFrequency, 625);

				uint32_t diff250 = gScanFrequency > freq250 ? gScanFrequency - freq250 : freq250 - gScanFrequency;
				uint32_t diff625 = gScanFrequency > freq625 ? gScanFrequency - freq625 : freq625 - gScanFrequency;

				if(diff250 > diff625) {
					stepSetting   = STEP_6_25kHz;
					gScanFrequency = freq625;
				}
				else {
					stepSetting   = STEP_2_5kHz;
					gScanFrequency = freq250;
				}
			}

			if (IS_MR_CHANNEL(gTxVfo->CHANNEL_SAVE)) {
				gScannerSaveState = SCAN_SAVE_CHAN_SEL;
				gScanChannel      = gTxVfo->CHANNEL_SAVE;
				gShowChPrefix     = RADIO_CheckValidChannel(gTxVfo->CHANNEL_SAVE, false, 0);
			}
			else {
				gScannerSaveState = SCAN_SAVE_CHANNEL;
			}

			gScanCssState         = SCAN_CSS_STATE_FOUND;
#ifdef ENABLE_VOICE
			gAnotherVoiceID   = VOICE_ID_MEMORY_CHANNEL;
#endif
			gRequestDisplayScreen = DISPLAY_SCANNER;
			
			gUpdateStatus = true;
			break;

		case SCAN_SAVE_CHAN_SEL:
			if (gInputBoxIndex == 0) {
				gBeepToPlay           = BEEP_1KHZ_60MS_OPTIONAL;
				gRequestDisplayScreen = DISPLAY_SCANNER;
				gScannerSaveState     = SCAN_SAVE_CHANNEL;
			}
			break;

		case SCAN_SAVE_CHANNEL:
			if (!gScanSingleFrequency) {
				RADIO_InitInfo(gTxVfo, gTxVfo->CHANNEL_SAVE, gScanFrequency);

				if (gScanUseCssResult) {
					gTxVfo->freq_config_RX.CodeType = gScanCssResultType;
					gTxVfo->freq_config_RX.Code     = gScanCssResultCode;
				}

				gTxVfo->freq_config_TX     = gTxVfo->freq_config_RX;
				gTxVfo->STEP_SETTING = stepSetting;
			}
			else {
				RADIO_ConfigureChannel(0, VFO_CONFIGURE_RELOAD);
				RADIO_ConfigureChannel(1, VFO_CONFIGURE_RELOAD);

				gTxVfo->freq_config_RX.CodeType = gScanCssResultType;
				gTxVfo->freq_config_RX.Code     = gScanCssResultCode;
				gTxVfo->freq_config_TX.CodeType = gScanCssResultType;
				gTxVfo->freq_config_TX.Code     = gScanCssResultCode;
			}

			uint8_t chan;
			if (IS_MR_CHANNEL(gTxVfo->CHANNEL_SAVE)) {
				chan = gScanChannel;
				gEeprom.MrChannel[gEeprom.TX_VFO] = chan;
			}
			else {
				chan = gTxVfo->Band + FREQ_CHANNEL_FIRST;
				gEeprom.FreqChannel[gEeprom.TX_VFO] = chan;
			}

			gTxVfo->CHANNEL_SAVE = chan;
			gEeprom.ScreenChannel[gEeprom.TX_VFO] = chan;
#ifdef ENABLE_VOICE	
			gAnotherVoiceID = VOICE_ID_CONFIRM;
#endif
			gRequestDisplayScreen = DISPLAY_SCANNER;
			gRequestSaveChannel = 2;
			gScannerSaveState = SCAN_SAVE_NO_PROMPT;
			break;

		default:
			gBeepToPlay = BEEP_1KHZ_60MS_OPTIONAL;
			break;
	}
}

static void SCANNER_Key_STAR(bool bKeyPressed, bool bKeyHeld)
{
	if (!bKeyHeld && bKeyPressed) {
		gBeepToPlay    = BEEP_1KHZ_60MS_OPTIONAL;
		SCANNER_Start(gScanSingleFrequency);
	}
	return;
}

static void SCANNER_Key_UP_DOWN(bool bKeyPressed, bool pKeyHeld, int8_t Direction)
{
	if (pKeyHeld) {
		if (!bKeyPressed)
			return;
	}
	else {
		if (!bKeyPressed)
			return;

		gInputBoxIndex = 0;
		gBeepToPlay    = BEEP_1KHZ_60MS_OPTIONAL;
	}

	if (gScannerSaveState == SCAN_SAVE_CHAN_SEL) {
		gScanChannel          = NUMBER_AddWithWraparound(gScanChannel, Direction, 0, MR_CHANNEL_LAST);
		gShowChPrefix         = RADIO_CheckValidChannel(gScanChannel, false, 0);
		gRequestDisplayScreen = DISPLAY_SCANNER;
	}
	else
		gBeepToPlay = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
}

void SCANNER_ProcessKeys(KEY_Code_t Key, bool bKeyPressed, bool bKeyHeld)
{
	switch (Key) {
		case KEY_0:
		case KEY_1:
		case KEY_2:
		case KEY_3:
		case KEY_4:
		case KEY_5:
		case KEY_6:
		case KEY_7:
		case KEY_8:
		case KEY_9:
			SCANNER_Key_DIGITS(Key, bKeyPressed, bKeyHeld);
			break;
		case KEY_MENU:
			SCANNER_Key_MENU(bKeyPressed, bKeyHeld);
			break;
		case KEY_UP:
			SCANNER_Key_UP_DOWN(bKeyPressed, bKeyHeld,  1);
			break;
		case KEY_DOWN:
			SCANNER_Key_UP_DOWN(bKeyPressed, bKeyHeld, -1);
			break;
		case KEY_EXIT:
			SCANNER_Key_EXIT(bKeyPressed, bKeyHeld);
			break;
		case KEY_STAR:
			SCANNER_Key_STAR(bKeyPressed, bKeyHeld);
			break;
		case KEY_PTT:
			GENERIC_Key_PTT(bKeyPressed);
			break;
		default:
			if (!bKeyHeld && bKeyPressed)
				gBeepToPlay = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
			break;
	}
}

void SCANNER_Start(bool singleFreq)
{
	gScanSingleFrequency = singleFreq;
	gMonitor = false;

#ifdef ENABLE_VOICE
	gAnotherVoiceID = VOICE_ID_SCANNING_BEGIN;
#endif

	BK4819_StopScan();
	RADIO_SelectVfos();

#ifdef ENABLE_NOAA
	if (IS_NOAA_CHANNEL(gRxVfo->CHANNEL_SAVE))
		gRxVfo->CHANNEL_SAVE = FREQ_CHANNEL_FIRST + BAND6_400MHz;
#endif

	uint8_t  backupStep      = gRxVfo->STEP_SETTING;
	uint16_t backupFrequency = gRxVfo->StepFrequency;

	RADIO_InitInfo(gRxVfo, gRxVfo->CHANNEL_SAVE, gRxVfo->pRX->Frequency);

	gRxVfo->STEP_SETTING  = backupStep;
	gRxVfo->StepFrequency = backupFrequency;

	RADIO_SetupRegisters(true);

#ifdef ENABLE_NOAA
	gIsNoaaMode = false;
#endif

	if (gScanSingleFrequency) {
		gScanCssState  = SCAN_CSS_STATE_SCANNING;
		gScanFrequency = gRxVfo->pRX->Frequency;
		stepSetting   = gRxVfo->STEP_SETTING;

		BK4819_PickRXFilterPathBasedOnFrequency(gScanFrequency);
		BK4819_SetScanFrequency(gScanFrequency);

		gUpdateStatus = true;
	}
	else {
		gScanCssState  = SCAN_CSS_STATE_OFF;

#ifdef ENABLE_AUTO_LOG
		if (gAutoLogMode && (gAutoLogScanMode == AUTOLOG_SCAN_SLOW
		                  || gAutoLogScanMode == AUTOLOG_SCAN_RSSI)) {
			// SLOW/RSSI: leave the chip in normal RX. STATE_OFF will step
			// frequencies + check signal presence from here. No BK4819
			// freq-counter setup (FAST-only).
			gScanFrequency = gRxVfo->pRX->Frequency;
			BK4819_PickRXFilterPathBasedOnFrequency(gScanFrequency);
			if (gAutoLogScanMode == AUTOLOG_SCAN_RSSI)
				autolog_rssi_sweep_setup();
		} else
#endif
		{
			gScanFrequency = 0xFFFFFFFF;
			BK4819_PickRXFilterPathBasedOnFrequency(gScanFrequency);
			BK4819_EnableFrequencyScan();
		}

		gUpdateStatus = true;
	}

#ifdef ENABLE_DTMF_CALLING
	DTMF_clear_RX();
#endif

	gScanDelay_10ms        = scan_delay_10ms;
	gScanCssResultCode     = 0xFF;
	gScanCssResultType     = 0xFF;
	scanHitCount          = 0;
	gScanUseCssResult      = false;
	g_CxCSS_TAIL_Found     = false;
	g_CDCSS_Lost           = false;
	gCDCSSCodeType         = 0;
	g_CTCSS_Lost           = false;
#ifdef ENABLE_VOX
	g_VOX_Lost         = false;
#endif
	g_SquelchLost          = false;
	gScannerSaveState      = SCAN_SAVE_NO_PROMPT;
	gScanProgressIndicator = 0;
}

void SCANNER_Stop(void)
{
	if(SCANNER_IsScanning()) {
#ifdef ENABLE_AUTO_LOG
		if (gAutoLogMode)
			AUTOLOG_Flush();
#endif
		gEeprom.CROSS_BAND_RX_TX = gBackup_CROSS_BAND_RX_TX;
		gVfoConfigureMode        = VFO_CONFIGURE_RELOAD;
		gFlagResetVfos           = true;
		gUpdateStatus            = true;
		gCssBackgroundScan 			 = false;
		gScanUseCssResult = false;
#ifdef ENABLE_VOICE
		gAnotherVoiceID          = VOICE_ID_CANCEL;
#endif
		BK4819_StopScan();
	}
}

void SCANNER_TimeSlice10ms(void)
{
	if (!SCANNER_IsScanning())
		return;

	if (gScanDelay_10ms > 0) {
		gScanDelay_10ms--;
		return;
	}

	if (gScannerSaveState != SCAN_SAVE_NO_PROMPT) {
		return;
	}

	switch (gScanCssState) {
		case SCAN_CSS_STATE_OFF: {
#ifdef ENABLE_AUTO_LOG
			if (gAutoLogMode && gAutoLogScanMode == AUTOLOG_SCAN_RSSI) {
				// RSSI sweep — tune + strobe + wait + read all in the same
				// tick (like the fagci spectrum analyzer). The chip has
				// already been put into sweep config by SCANNER_Start /
				// AUTOLOG_RecordAndResume; we just step + measure here.
				if (rssi_tuned_freq != gRxVfo->pRX->Frequency) {
					BK4819_SetFrequency(gRxVfo->pRX->Frequency);
					BK4819_PickRXFilterPathBasedOnFrequency(gRxVfo->pRX->Frequency);
					// Strobe REG_30 to reset RSSI integration.
					const uint16_t r30 = BK4819_ReadRegister(BK4819_REG_30);
					BK4819_WriteRegister(BK4819_REG_30, 0);
					BK4819_WriteRegister(BK4819_REG_30, r30);
					rssi_tuned_freq = gRxVfo->pRX->Frequency;
				}
				// Wait for the glitch indicator to settle below max.
				while ((BK4819_ReadRegister(0x63) & 0xFF) >= 255)
					SYSTICK_DelayUs(100);
				const uint16_t rssi = BK4819_GetRSSI();

				if (rssi >= autolog_rssi_thresholds[gAutoLogRssiTh]) {
					// Capture RSSI NOW — STATE_SCANNING / CSS scan mode
					// gives unreliable readings (rssi/2 fits in uint8_t for
					// any sane signal, matches dBm+160 storage).
					autolog_hit_rssi       = (uint8_t)(rssi >> 1);
					gScanFrequency         = gRxVfo->pRX->Frequency;
					scanHitCount           = 0;
					gScanCssResultCode     = 0xFF;
					gScanCssResultType     = 0xFF;
					gScanUseCssResult      = false;
					gScanProgressIndicator = 0;
					RADIO_ConfigureSquelchAndOutputPower(gRxVfo);
					RADIO_SetupRegisters(true);
					BK4819_SetScanFrequency(gScanFrequency);
					gScanCssState   = SCAN_CSS_STATE_SCANNING;
					gScanDelay_10ms = 9;
					rssi_tuned_freq = 0;  // force retune when we return
				} else {
					// Advance to next freq; actual chip retune happens at
					// the top of the next tick (above) once gRxVfo->freq
					// differs from rssi_tuned_freq.
					gRxVfo->freq_config_RX.Frequency = slow_advance_freq();
					gScanDelay_10ms = 1;  // ~10 ms per channel
				}
				break;
			}
			if (gAutoLogMode && gAutoLogScanMode == AUTOLOG_SCAN_SLOW) {
				// SLOW mode: full-sensitivity squelch check at each freq.
				// On squelch open → log freq-only and advance. CTCSS/DCS
				// detection in SLOW is a TODO — making it work requires
				// switching the chip between normal RX and CSS scan modes
				// per hit, and the timing/state coordination with the rest
				// of the firmware has been flaky. For CTCSS-bearing
				// channels, use FAST mode instead (works correctly).
				if (g_SquelchLost) {
					// Signal detected — capture RSSI now (CSS scan mode
					// makes it unreliable), then transition to STATE_SCANNING
					// for CTCSS/DCS detection (up to 2s timeout if no code
					// is found, set by SCANNER_TimeSlice500ms).
					autolog_hit_rssi       = (uint8_t)(BK4819_GetRSSI_dBm() + 160);
					gScanFrequency         = gRxVfo->pRX->Frequency;
					scanHitCount           = 0;
					gScanCssResultCode     = 0xFF;
					gScanCssResultType     = 0xFF;
					gScanUseCssResult      = false;
					gScanProgressIndicator = 0;
					BK4819_SetScanFrequency(gScanFrequency);
					gScanCssState          = SCAN_CSS_STATE_SCANNING;
					gScanDelay_10ms        = 9;
					break;
				} else {
					gRxVfo->freq_config_RX.Frequency = slow_advance_freq();
					RADIO_ApplyOffset(gRxVfo);
					RADIO_ConfigureSquelchAndOutputPower(gRxVfo);
					RADIO_SetupRegisters(true);
				}
				gScanDelay_10ms = 9;   // ~90 ms dwell per channel
				break;
			}
#endif
			// FAST mode: BK4819 hardware frequency counter
			uint32_t result;
			if (!BK4819_GetFrequencyScanResult(&result))
				break;

			int32_t delta = result - gScanFrequency;
			gScanFrequency = result;

			if (delta < 0)
				delta = -delta;
			if (delta < 100)
				scanHitCount++;
			else
				scanHitCount = 0;

			BK4819_DisableFrequencyScan();

			if (scanHitCount < 3) {
				BK4819_EnableFrequencyScan();
			}
			else {
				BK4819_SetScanFrequency(gScanFrequency);
				gScanCssResultCode     = 0xFF;
				gScanCssResultType     = 0xFF;
				scanHitCount          = 0;
				gScanUseCssResult      = false;
				gScanProgressIndicator = 0;
				gScanCssState          = SCAN_CSS_STATE_SCANNING;

				if(!gCssBackgroundScan)
					GUI_SelectNextDisplay(DISPLAY_SCANNER);

				gUpdateStatus          = true;
			}

			gScanDelay_10ms = scan_delay_10ms;
			//gScanDelay_10ms = 1;   // 10ms
			break;
		}
		case SCAN_CSS_STATE_SCANNING: {
			uint32_t cdcssFreq;
			uint16_t ctcssFreq;
			BK4819_CssScanResult_t scanResult = BK4819_GetCxCSSScanResult(&cdcssFreq, &ctcssFreq);
			if (scanResult == BK4819_CSS_RESULT_NOT_FOUND)
				break;

			BK4819_Disable();

			if (scanResult == BK4819_CSS_RESULT_CDCSS) {
				const uint8_t Code = DCS_GetCdcssCode(cdcssFreq);
				if (Code != 0xFF)
				{
					gScanCssResultCode = Code;
					gScanCssResultType = CODE_TYPE_DIGITAL;
					gScanCssState      = SCAN_CSS_STATE_FOUND;
					gScanUseCssResult  = true;
					gUpdateStatus      = true;
				}
			}
			else if (scanResult == BK4819_CSS_RESULT_CTCSS) {
				const uint8_t Code = DCS_GetCtcssCode(ctcssFreq);
				if (Code != 0xFF) {
					if (Code == gScanCssResultCode && gScanCssResultType == CODE_TYPE_CONTINUOUS_TONE) {
						if (++scanHitCount >= 2) {
							gScanCssState     = SCAN_CSS_STATE_FOUND;
							gScanUseCssResult = true;
							gUpdateStatus     = true;
						}
					}
					else
						scanHitCount = 0;

					gScanCssResultType = CODE_TYPE_CONTINUOUS_TONE;
					gScanCssResultCode = Code;
				}
			}

			if (gScanCssState < SCAN_CSS_STATE_FOUND) { // scanning or off
				BK4819_SetScanFrequency(gScanFrequency);
				gScanDelay_10ms = scan_delay_10ms;
				break;
			}

#ifdef ENABLE_AUTO_LOG
			if (gAutoLogMode && !gCssBackgroundScan) {
				AUTOLOG_RecordAndResume();
				break;
			}
#endif

			if(gCssBackgroundScan) {
				gCssBackgroundScan = false;
				if(gScanUseCssResult)
					MENU_CssScanFound();
			}
			else
				GUI_SelectNextDisplay(DISPLAY_SCANNER);


			break;
		}
		default:
			gCssBackgroundScan = false;
			break;
	}

}

void SCANNER_TimeSlice500ms(void)
{
#ifdef ENABLE_AUTO_LOG
	// Battery-low watchdog. When the radio reports the battery icon at "low"
	// (≤ level 1, roughly ≤5% / sub-6.30V), do the final flush + stop the
	// scan. This gives us roughly a minute to write everything before the
	// chip browns out. Otherwise we keep accumulating in RAM so we can do a
	// single sorted flush — most-interesting first — at exit.
	if (gAutoLogMode && gBatteryDisplayLevel <= 1) {
		SCANNER_Stop();
		gRequestDisplayScreen = DISPLAY_MAIN;
	}
#endif

	if (SCANNER_IsScanning() && gScannerSaveState == SCAN_SAVE_NO_PROMPT && gScanCssState < SCAN_CSS_STATE_FOUND) {
		gScanProgressIndicator++;
#ifndef ENABLE_NO_CODE_SCAN_TIMEOUT
		if (gScanProgressIndicator > 32) {
			if (gScanCssState == SCAN_CSS_STATE_SCANNING && !gScanSingleFrequency)
				gScanCssState = SCAN_CSS_STATE_FOUND;
			else
				gScanCssState = SCAN_CSS_STATE_FAILED;

			gUpdateStatus = true;
		}
#endif
#ifdef ENABLE_AUTO_LOG
		// 2-second dwell timeout when auto-logging: if a carrier is locked
		// (STATE_SCANNING) but no CSS is detected, record as frequency-only
		// and resume sweep. Independent of ENABLE_NO_CODE_SCAN_TIMEOUT.
		if (gAutoLogMode && !gCssBackgroundScan &&
			gScanCssState == SCAN_CSS_STATE_SCANNING &&
			gScanProgressIndicator > 4) {
			AUTOLOG_RecordAndResume();
		}

		// LNA multiplex: only in FAST STATE_OFF. SLOW and RSSI manage LNA
		// per step themselves (RADIO_SetupRegisters / PickRXFilterPath
		// picks the right path), and overriding it would put the chip on
		// the wrong LNA for the current step frequency.
		if (gAutoLogMode && !gCssBackgroundScan &&
			gAutoLogScanMode == AUTOLOG_SCAN_FAST &&
			gScanCssState == SCAN_CSS_STATE_OFF &&
			(gScanProgressIndicator & 3) == 1) {
			autolog_lna_phase = (autolog_lna_phase == 1) ? 2 : 1;
			BK4819_PickRXFilterPathBasedOnFrequency(
				(autolog_lna_phase == 1) ? 14600000 : 44600000);
		}
#endif
		gUpdateDisplay = true;
	}
	else if(gCssBackgroundScan) {
		gUpdateDisplay = true;
	}
}

bool SCANNER_IsScanning(void)
{
	return gCssBackgroundScan || (gScreenToDisplay == DISPLAY_SCANNER);
}