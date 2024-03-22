/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * cam_helper_gc8613.cpp - camera helper for gc8613 sensor
 */

#include <algorithm>
#include <assert.h>
#include <cmath>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#include <libcamera/base/log.h>

#include "cam_helper.h"
#include "md_parser.h"

using namespace RPiController;
using namespace libcamera;
using libcamera::utils::Duration;

namespace libcamera {
LOG_DECLARE_CATEGORY(IPARPI)
}

/*
 * We care about two gain registers and a pair of exposure registers. Their
 * I2C addresses from the Sony IMX477 datasheet:
 */
constexpr uint32_t expHiReg = 0x0202;
constexpr uint32_t expLoReg = 0x0203;
constexpr uint32_t gainHiReg = 0x0204;
constexpr uint32_t gainLoReg = 0x0205;
constexpr uint32_t frameLengthHiReg = 0x0340;
constexpr uint32_t frameLengthLoReg = 0x0341;
constexpr uint32_t lineLengthHiReg = 0x0342;
constexpr uint32_t lineLengthLoReg = 0x0343;
constexpr uint32_t temperatureReg = 0x013a;
constexpr std::initializer_list<uint32_t> registerList =
	{ expHiReg, expLoReg, gainHiReg, gainLoReg, frameLengthHiReg, frameLengthLoReg,
	  lineLengthHiReg, lineLengthLoReg, temperatureReg };

class CamHelperGc8613 : public CamHelper
{
public:
	CamHelperGc8613();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
	void prepare(libcamera::Span<const uint8_t> buffer, Metadata &metadata) override;
	std::pair<uint32_t, uint32_t> getBlanking(Duration &exposure, Duration minFrameDuration,
						  Duration maxFrameDuration) const override;
	void getDelays(int &exposureDelay, int &gainDelay,
		       int &vblankDelay, int &hblankDelay) const override;
	bool sensorEmbeddedDataPresent() const override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr int frameIntegrationDiff = 22;
	/* Maximum frame length allowable for long exposure calculations. */
	static constexpr int frameLengthMax = 0xffdc;
	/* Largest long exposure scale factor given as a left shift on the frame length. */
	static constexpr int longExposureShiftMax = 7;

	void populateMetadata(const MdParser::RegisterMap &registers,
			      Metadata &metadata) const override;
};

CamHelperGc8613::CamHelperGc8613()
	: CamHelper(std::make_unique<MdParserSmia>(registerList), frameIntegrationDiff)
{
	printf("[MZQ]%s: %s,%d\n", __FILE__, __func__, __LINE__);
}

uint32_t CamHelperGc8613::gainCode(double gain) const
{
	printf("[MZQ]%s,%d: %s: gain=%lf\n", __FILE__, __LINE__, __func__, gain);
	return static_cast<uint32_t>(1024 - 1024 / gain);
}

double CamHelperGc8613::gain(uint32_t gainCode) const
{
	printf("[MZQ]%s,%d: %s: gainCode=%d\n", __FILE__, __LINE__, __func__, gainCode);
	return 1024.0 / (1024 - gainCode);
}

void CamHelperGc8613::prepare(libcamera::Span<const uint8_t> buffer, Metadata &metadata)
{
	MdParser::RegisterMap registers;
	DeviceStatus deviceStatus;

	if (metadata.get("device.status", deviceStatus)) {
		LOG(IPARPI, Error) << "DeviceStatus not found from DelayedControls";
		return;
	}

	parseEmbeddedData(buffer, metadata);

	/*
	 * The DeviceStatus struct is first populated with values obtained from
	 * DelayedControls. If this reports frame length is > frameLengthMax,
	 * it means we are using a long exposure mode. Since the long exposure
	 * scale factor is not returned back through embedded data, we must rely
	 * on the existing exposure lines and frame length values returned by
	 * DelayedControls.
	 *
	 * Otherwise, all values are updated with what is reported in the
	 * embedded data.
	 */
	if (deviceStatus.frameLength > frameLengthMax) {
		DeviceStatus parsedDeviceStatus;

		metadata.get("device.status", parsedDeviceStatus);
		parsedDeviceStatus.shutterSpeed = deviceStatus.shutterSpeed;
		parsedDeviceStatus.frameLength = deviceStatus.frameLength;
		metadata.set("device.status", parsedDeviceStatus);

		LOG(IPARPI, Debug) << "Metadata updated for long exposure: "
				   << parsedDeviceStatus;
	}
	printf("[MZQ]%s: %s,%d\n", __FILE__, __func__, __LINE__);
}

std::pair<uint32_t, uint32_t> CamHelperGc8613::getBlanking(Duration &exposure,
							   Duration minFrameDuration,
							   Duration maxFrameDuration) const
{
	uint32_t frameLength, exposureLines;
	unsigned int shift = 0;

	auto [vblank, hblank] = CamHelper::getBlanking(exposure, minFrameDuration,
						       maxFrameDuration);

	frameLength = mode_.height + vblank;
	Duration lineLength = hblankToLineLength(hblank);

	/*
	 * Check if the frame length calculated needs to be setup for long
	 * exposure mode. This will require us to use a long exposure scale
	 * factor provided by a shift operation in the sensor.
	 */
	while (frameLength > frameLengthMax) {
		if (++shift > longExposureShiftMax) {
			shift = longExposureShiftMax;
			frameLength = frameLengthMax;
			break;
		}
		frameLength >>= 1;
	}

	if (shift) {
		/* Account for any rounding in the scaled frame length value. */
		frameLength <<= shift;
		exposureLines = CamHelperGc8613::exposureLines(exposure, lineLength);
		exposureLines = std::min(exposureLines, frameLength - frameIntegrationDiff);
		exposure = CamHelperGc8613::exposure(exposureLines, lineLength);
	}

	printf("[MZQ]%s: %s,%d\n", __FILE__, __func__, __LINE__);
	return { frameLength - mode_.height, hblank };
}

void CamHelperGc8613::getDelays(int &exposureDelay, int &gainDelay,
				int &vblankDelay, int &hblankDelay) const
{
	exposureDelay = 2;
	gainDelay = 2;
	vblankDelay = 3;
	hblankDelay = 3;
	printf("[MZQ]%s: %s,%d\n", __FILE__, __func__, __LINE__);
}

bool CamHelperGc8613::sensorEmbeddedDataPresent() const
{
	printf("[MZQ]%s: %s,%d\n", __FILE__, __func__, __LINE__);
	return true;
}

void CamHelperGc8613::populateMetadata(const MdParser::RegisterMap &registers,
				       Metadata &metadata) const
{
	DeviceStatus deviceStatus;

	deviceStatus.lineLength = lineLengthPckToDuration(registers.at(lineLengthHiReg) * 256 +
							  registers.at(lineLengthLoReg));
	deviceStatus.shutterSpeed = exposure(registers.at(expHiReg) * 256 + registers.at(expLoReg),
					     deviceStatus.lineLength);
	deviceStatus.analogueGain = gain(registers.at(gainHiReg) * 256 + registers.at(gainLoReg));
	deviceStatus.frameLength = registers.at(frameLengthHiReg) * 256 + registers.at(frameLengthLoReg);
	deviceStatus.sensorTemperature = std::clamp<int8_t>(registers.at(temperatureReg), -20, 80);

	printf("[MZQ]%s: %s,%d\n", __FILE__, __func__, __LINE__);
	metadata.set("device.status", deviceStatus);
}

static CamHelper *create()
{
	printf("[MZQ]%s: %s,%d\n", __FILE__, __func__, __LINE__);
	return new CamHelperGc8613();
}

static RegisterCamHelper reg("gc8613", &create);
