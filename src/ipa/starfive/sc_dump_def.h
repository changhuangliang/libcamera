/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Starfive Technology Co., Ltd.
 *
 * sc_dump_def.h - Starfive statistic dump definitioin
 */

#pragma once

namespace libcamera {

namespace ipa::starfive {

#define ISP_SCD_BUFFER_SIZE (19 * 256 * 4) // align 128
#define ISP_YHIST_BUFFER_SIZE (64 * 4)

#define AE_WEIGHT_SUM_Y_OFFSET (0x0084)
#define AE_HIST_Y_OFFSET (0x4204)
#define SC_DUMP_ITEM_NEXT_SEGMENT_OFFSET (0x0400)

#define AWB_WEIGHTING_SUM_W_OFFSET (0x0200)

struct SCCropInfo {
	uint16_t hStart;
	uint16_t vStart;
	uint8_t swWidth;
	uint8_t swHeight;
	uint8_t hPeriod;
	uint8_t hKeep;
	uint8_t vPeriod;
	uint8_t vKeep;
};

} // namespace ipa::starfive

} // namespace libcamera
