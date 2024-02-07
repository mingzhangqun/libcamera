/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * cam_helper_imx415.cpp - camera helper for imx415 sensor
 */

#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

/*
 * We have observed that the imx415 embedded data stream randomly returns junk
 * register values. Do not rely on embedded data until this has been resolved.
 */
#define ENABLE_EMBEDDED_DATA 0

#include "cam_helper.h"
#if ENABLE_EMBEDDED_DATA
#include "md_parser.h"
#endif

using namespace RPiController;

/*
 * We care about one gain register and a pair of exposure registers. Their I2C
 * addresses from the Sony IMX415 datasheet:
 */
constexpr uint32_t gainReg = 0x157;
constexpr uint32_t expHiReg = 0x15a;
constexpr uint32_t expLoReg = 0x15b;
constexpr uint32_t frameLengthHiReg = 0x160;
constexpr uint32_t frameLengthLoReg = 0x161;
constexpr uint32_t lineLengthHiReg = 0x162;
constexpr uint32_t lineLengthLoReg = 0x163;
constexpr std::initializer_list<uint32_t> registerList [[maybe_unused]]
	= { expHiReg, expLoReg, gainReg, frameLengthHiReg, frameLengthLoReg,
	    lineLengthHiReg, lineLengthLoReg };

class CamHelperImx419 : public CamHelper
{
public:
	CamHelperImx419();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
	unsigned int mistrustFramesModeSwitch() const override;
	bool sensorEmbeddedDataPresent() const override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr int frameIntegrationDiff = 4;

	void populateMetadata(const MdParser::RegisterMap &registers,
			      Metadata &metadata) const override;
};

CamHelperImx419::CamHelperImx419()
#if ENABLE_EMBEDDED_DATA
	: CamHelper(std::make_unique<MdParserSmia>(registerList), frameIntegrationDiff)
#else
	: CamHelper({}, frameIntegrationDiff)
#endif
{
}

uint32_t CamHelperImx419::gainCode(double gain) const
{
	return (uint32_t)(256 - 256 / gain);
}

double CamHelperImx419::gain(uint32_t gainCode) const
{
	return 256.0 / (256 - gainCode);
}

unsigned int CamHelperImx419::mistrustFramesModeSwitch() const
{
	/*
	 * For reasons unknown, we do occasionally get a bogus metadata frame
	 * at a mode switch (though not at start-up). Possibly warrants some
	 * investigation, though not a big deal.
	 */
	return 1;
}

bool CamHelperImx419::sensorEmbeddedDataPresent() const
{
	return ENABLE_EMBEDDED_DATA;
}

void CamHelperImx419::populateMetadata(const MdParser::RegisterMap &registers,
				       Metadata &metadata) const
{
	DeviceStatus deviceStatus;

	deviceStatus.lineLength = lineLengthPckToDuration(registers.at(lineLengthHiReg) * 256 +
							  registers.at(lineLengthLoReg));
	deviceStatus.shutterSpeed = exposure(registers.at(expHiReg) * 256 + registers.at(expLoReg),
					     deviceStatus.lineLength);
	deviceStatus.analogueGain = gain(registers.at(gainReg));
	deviceStatus.frameLength = registers.at(frameLengthHiReg) * 256 + registers.at(frameLengthLoReg);

	metadata.set("device.status", deviceStatus);
}

static CamHelper *create()
{
	printf("[MZQ]%s: %s,%d\n", __FILE__, __func__, __LINE__);
	return new CamHelperImx419();
}

static RegisterCamHelper reg("imx415", &create);
