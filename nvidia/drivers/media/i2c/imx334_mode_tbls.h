/*
 * imx334_mode_tbls.h - imx334 sensor mode tables
 *
 * Copyright (c) 2019. FRAMOS.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __IMX334_TABLES__
#define __IMX334_TABLES__

/**
 * Image sensor registers as described in the IMX334 register map
 */

#define STANDBY              0x3000
#define REGHOLD              0x3001
#define XMSTA                0x3002

#define BCWAIT_TIME          0x300C
#define CPWAIT_TIME          0x300D
#define SECOND_SLAVE_ADD     0x3010

#define WINMODE              0x3018

#define HTRIMMING_START_LOW  0x302C
#define HTRIMMING_START_HIGH 0x302D
#define HNUM_LOW             0x302E
#define HNUM_HIGH            0x302F

#define VMAX_LOW             0x3030
#define VMAX_MID             0x3031
#define VMAX_HIGH            0x3032
#define HMAX_LOW             0x3034
#define HMAX_HIGH            0x3035

#define OPB_SIZE_V           0x304C

#define HREVERSE             0x304E
#define VREVERSE             0x304F

#define ADBIT                0x3050

#define SHR0_LOW             0x3058
#define SHR0_MID             0x3059
#define SHR0_HIGH            0x305A

#define AREA3_ST_ADR_1_LOW   0x3074
#define AREA3_ST_ADR_1_HIGH  0x3075
#define AREA3_WIDTH_1_LOW    0x3076
#define AREA3_WIDTH_1_HIGH   0x3077

#define AREA3_ST_ADR_2_LOW   0x308E
#define AREA3_ST_ADR_2_HIGH  0x308F
#define AREA3_WIDTH_2_LOW    0x3090
#define AREA3_WIDTH_2_HIGH   0x3091

#define BLACK_OFSET_ADR_LOW  0x30C6
#define BLACK_OFSET_ADR_HIGH 0x30C7

#define UNRD_LINE_MAX_LOW    0x30CE
#define UNRD_LINE_MAX_HIGH   0x30CF

#define UNREAD_ED_ADR_LOW    0x30D8
#define UNREAD_ED_ADR_HIGH   0x30D9

#define GAIN_LOW             0x30E8
#define GAIN_HIGH            0x30E9

#define INCKSEL1_LOW         0x314C
#define INCKSEL1_HIGH        0x314D
#define INCKSEL2             0x315A
#define INCKSEL3             0x3168
#define INCKSEL4             0x316A

#define HADD_VADD            0x3199

#define MDBIT                0x319D
#define SYS_MODE             0x319E

#define XHSOUTSEL_XVSOUTSEL  0x31A0
#define XVS_XHS_DRV          0x31A1

#define XVSLNG               0x31D4
#define XHSLNG               0x31D5
#define EXTMODE              0x31D9

#define VALID_EXPAND         0x31DD

#define TCYCLE               0x3300
#define BLKLEVEL_LOW         0x3302
#define BLKLEVEL_HIGH        0x3303
#define Y_OUT_SIZE_LOW       0x3308
#define Y_OUT_SIZE_HIGH      0x3309

#define ADBIT1_LOW           0x341C
#define ADBIT1_HIGH          0x341D

#define LANE_MODE            0x3A01

#define TCLKPOST_LOW         0x3A18
#define TCLKPOST_HIGH        0x3A19
#define TCLKPREPARE_LOW      0x3A1A
#define TCLKPREPARE_HIGH     0x3A1B
#define TCLKTRAIL_LOW        0x3A1C
#define TCLKTRAIL_HIGH       0x3A1D
#define TCLKZERO_LOW         0x3A1E
#define TCLKZERO_HIGH        0x3A1F

#define THSPREPARE_LOW       0x3A20
#define THSPREPARE_HIGH      0x3A21
#define THSZERO_LOW          0x3A22
#define THSZERO_HIGH         0x3A23
#define THSTRAIL_LOW         0x3A24
#define THSTRAIL_HIGH        0x3A25
#define THSEXIT_LOW          0x3A26
#define THSEXIT_HIGH         0x3A27
#define TLPX_LOW             0x3A28
#define TLPX_HIGH            0x3A29

#define DIG_CLP_MODE         0x3280
#define TPG_EN_DUOUT         0x329C
#define TPG_PATSEL_DUOUT     0x329E
#define TPG_COLORWIDTH       0x32A0

#define WRJ_OPEN             0x336C

#define IMX334_REG_3288      0x3288
#define IMX334_REG_328A      0x328A
#define IMX334_REG_3414      0x3414
#define IMX334_REG_3416      0x3416
#define IMX334_REG_35AC      0x35AC

#define IMX334_REG_3648      0x3648
#define IMX334_REG_364A      0x364A
#define IMX334_REG_364C      0x364C
#define IMX334_REG_3678      0x3678
#define IMX334_REG_367C      0x367C
#define IMX334_REG_367E      0x367E

#define IMX334_REG_3708      0x3708
#define IMX334_REG_3714      0x3714
#define IMX334_REG_3715      0x3715
#define IMX334_REG_3716      0x3716
#define IMX334_REG_3717      0x3717
#define IMX334_REG_371C      0x371C
#define IMX334_REG_371D      0x371D
#define IMX334_REG_372C      0x372C
#define IMX334_REG_372D      0x372D
#define IMX334_REG_372E      0x372E
#define IMX334_REG_372F      0x372F
#define IMX334_REG_3730      0x3730
#define IMX334_REG_3731      0x3731
#define IMX334_REG_3732      0x3732
#define IMX334_REG_3733      0x3733
#define IMX334_REG_3734      0x3734
#define IMX334_REG_3735      0x3735
#define IMX334_REG_375D      0x375D
#define IMX334_REG_375E      0x375E
#define IMX334_REG_375F      0x375F
#define IMX334_REG_3760      0x3760
#define IMX334_REG_3768      0x3768
#define IMX334_REG_3769      0x3769
#define IMX334_REG_376A      0x376A
#define IMX334_REG_376B      0x376B
#define IMX334_REG_376C      0x376C
#define IMX334_REG_376D      0x376D
#define IMX334_REG_376E      0x376E
#define IMX334_REG_3776      0x3776
#define IMX334_REG_3777      0x3777
#define IMX334_REG_3778      0x3778
#define IMX334_REG_3779      0x3779
#define IMX334_REG_377A      0x377A
#define IMX334_REG_377B      0x377B
#define IMX334_REG_377C      0x377C
#define IMX334_REG_377D      0x377D
#define IMX334_REG_377E      0x377E
#define IMX334_REG_377F      0x377F
#define IMX334_REG_3780      0x3780
#define IMX334_REG_3781      0x3781
#define IMX334_REG_3782      0x3782
#define IMX334_REG_3783      0x3783
#define IMX334_REG_3784      0x3784
#define IMX334_REG_3788      0x3788
#define IMX334_REG_378A      0x378A
#define IMX334_REG_378B      0x378B
#define IMX334_REG_378C      0x378C
#define IMX334_REG_378D      0x378D
#define IMX334_REG_378E      0x378E
#define IMX334_REG_378F      0x378F
#define IMX334_REG_3790      0x3790
#define IMX334_REG_3792      0x3792
#define IMX334_REG_3794      0x3794
#define IMX334_REG_3796      0x3796
#define IMX334_REG_37B0      0x37B0
#define IMX334_REG_3E04      0x3E04

/**
 * Special values for the write table function
 */
#define IMX334_TABLE_WAIT_MS 0
#define IMX334_TABLE_END     1
#define IMX334_WAIT_MS       10

#define IMX334_DEFAULT_WIDTH  3864
#define IMX334_DEFAULT_HEIGHT 2180

/**
 * Minimal value of frame length is resolution height + 48
 * Minimal value for scaling modes is full pixel mode height + 48
 *
 * Determined from the default value of FRM_LENGTH_LINES register
 * and empirically confirmed
 */
#define IMX334_MIN_FRAME_LENGTH_DELTA 48

#define IMX334_TO_LOW_BYTE(x) (x & 0xFF)
#define IMX334_TO_MID_BYTE(x) (x >> 8)

typedef struct reg_8 imx334_reg;

/**
 * Tables for the write table function
 */

static const imx334_reg imx334_start[] = {

    {STANDBY,              0x00},
    {IMX334_TABLE_WAIT_MS, 30},
    {XMSTA,                0x00},

    {IMX334_TABLE_WAIT_MS, IMX334_WAIT_MS},
    {IMX334_TABLE_END,     0x00}
};

static const imx334_reg imx334_stop[] = {

    {XMSTA,                0x01},
    {IMX334_TABLE_WAIT_MS, 30},
    {STANDBY,              0x01},

    {IMX334_TABLE_WAIT_MS, IMX334_WAIT_MS},
    {IMX334_TABLE_END,     0x00}
};

static const imx334_reg imx334_10bit_mode[] = {

    {ADBIT,                0x00},
    {ADBIT1_HIGH,          0x01},
    {ADBIT1_LOW,           0xFF},

    {MDBIT,                0x00},

    {IMX334_TABLE_WAIT_MS, IMX334_WAIT_MS},
    {IMX334_TABLE_END,     0x00}
};

static const imx334_reg imx334_12bit_mode[] = {

    {ADBIT,                0x01},
    {ADBIT1_HIGH,          0x00},
    {ADBIT1_LOW,           0x47},

    {MDBIT,                0x01},

    {IMX334_TABLE_WAIT_MS, IMX334_WAIT_MS},
    {IMX334_TABLE_END,     0x00}
};

static const imx334_reg imx334_1782_mbps[] = {

    {TCLKPOST_LOW,         0xB7},
    {TCLKPREPARE_LOW,      0x67},
    {TCLKTRAIL_LOW,        0x6F},
    {TCLKZERO_HIGH,        0x01},
    {TCLKZERO_LOW,         0xDF},
    {THSPREPARE_LOW,       0x6F},
    {THSZERO_LOW,          0xCF},
    {THSTRAIL_LOW,         0x6F},
    {THSEXIT_LOW,          0xB7},
    {TLPX_LOW,             0x5F},

    {BCWAIT_TIME,          0x5B},
    {CPWAIT_TIME,          0x40},
    {INCKSEL1_HIGH,        0x00},
    {INCKSEL1_LOW,         0xC0},
    {INCKSEL2,             0x02},

    {INCKSEL3,             0x68},
    {INCKSEL4,             0x7E},

    {SYS_MODE,             0x00},

    {IMX334_TABLE_WAIT_MS, IMX334_WAIT_MS},
    {IMX334_TABLE_END,     0x0000}
};

static const imx334_reg imx334_1188_mbps[] = {

    {TCLKPOST_LOW,         0x8F},
    {TCLKPREPARE_LOW,      0x4F},
    {TCLKTRAIL_LOW,        0x47},
    {TCLKZERO_HIGH,        0x01},
    {TCLKZERO_LOW,         0x37},
    {THSPREPARE_LOW,       0x4F},
    {THSZERO_LOW,          0x87},
    {THSTRAIL_LOW,         0x4F},
    {THSEXIT_LOW,          0x7F},
    {TLPX_LOW,             0x3F},

    {BCWAIT_TIME,          0x5B},
    {CPWAIT_TIME,          0x40},
    {INCKSEL1_HIGH,        0x00},
    {INCKSEL1_LOW,         0x80},
    {INCKSEL2,             0x02},

    {INCKSEL3,             0x68},
    {INCKSEL4,             0x7E},

    {SYS_MODE,             0x01},

    {IMX334_TABLE_WAIT_MS, IMX334_WAIT_MS},
    {IMX334_TABLE_END,     0x0000}
};

static const imx334_reg imx334_891_mbps[] = {

    {TCLKPOST_LOW,         0x7F},
    {TCLKPREPARE_LOW,      0x37},
    {TCLKTRAIL_LOW,        0x37},
    {TCLKZERO_HIGH,        0x00},
    {TCLKZERO_LOW,         0xF7},
    {THSPREPARE_LOW,       0x3F},
    {THSZERO_LOW,          0x6F},
    {THSTRAIL_LOW,         0x3F},
    {THSEXIT_LOW,          0x5F},
    {TLPX_LOW,             0x2F},

    {BCWAIT_TIME,          0x5B},
    {CPWAIT_TIME,          0x40},
    {INCKSEL1_HIGH,        0x00},
    {INCKSEL1_LOW,         0xC0},
    {INCKSEL2,             0x06},

    {INCKSEL3,             0x68},
    {INCKSEL4,             0x7E},

    {SYS_MODE,             0x02},

    {IMX334_TABLE_WAIT_MS, IMX334_WAIT_MS},
    {IMX334_TABLE_END,     0x0000}
};

static const imx334_reg imx334_init_settings[] = {

    {WINMODE,              0x00},
    {VMAX_MID,             0x08},
    {VMAX_LOW,             0xCA},
    {HMAX_HIGH,            0x04},
    {HMAX_LOW,             0x4C},
    {OPB_SIZE_V,           0x14},
    {HADD_VADD,            0x00},
    {VALID_EXPAND,         0x03},
    {LANE_MODE,            0x03},

    {IMX334_REG_3288,      0x21},
    {IMX334_REG_328A,      0x02},
    {IMX334_REG_3414,      0x05},
    {IMX334_REG_3416,      0x18},
    {IMX334_REG_35AC,      0x0E},

    {IMX334_REG_3648,      0x01},
    {IMX334_REG_364A,      0x04},
    {IMX334_REG_364C,      0x04},
    {IMX334_REG_3678,      0x01},
    {IMX334_REG_367C,      0x31},
    {IMX334_REG_367E,      0x31},

    {IMX334_REG_3708,      0x02},
    {IMX334_REG_3714,      0x01},
    {IMX334_REG_3715,      0x02},
    {IMX334_REG_3716,      0x02},
    {IMX334_REG_3717,      0x02},
    {IMX334_REG_371C,      0x3D},

    {IMX334_REG_371D,      0x3F},
    {IMX334_REG_372C,      0x00},
    {IMX334_REG_372D,      0x00},
    {IMX334_REG_372E,      0x46},
    {IMX334_REG_372F,      0x00},
    {IMX334_REG_3730,      0x89},

    {IMX334_REG_3731,      0x00},
    {IMX334_REG_3732,      0x08},
    {IMX334_REG_3733,      0x01},
    {IMX334_REG_3734,      0xFE},
    {IMX334_REG_3735,      0x05},
    {IMX334_REG_375D,      0x00},
    {IMX334_REG_375E,      0x00},
    {IMX334_REG_375F,      0x61},
    {IMX334_REG_3760,      0x06},
    {IMX334_REG_3768,      0x1B},
    {IMX334_REG_3769,      0x1B},
    {IMX334_REG_376A,      0x1A},
    {IMX334_REG_376B,      0x19},
    {IMX334_REG_376C,      0x18},
    {IMX334_REG_376D,      0x14},
    {IMX334_REG_376E,      0x0F},
    {IMX334_REG_3776,      0x00},
    {IMX334_REG_3777,      0x00},
    {IMX334_REG_3778,      0x46},
    {IMX334_REG_3779,      0x00},
    {IMX334_REG_377A,      0x08},
    {IMX334_REG_377B,      0x01},
    {IMX334_REG_377C,      0x45},
    {IMX334_REG_377D,      0x01},
    {IMX334_REG_377E,      0x23},
    {IMX334_REG_377F,      0x02},
    {IMX334_REG_3780,      0xD9},
    {IMX334_REG_3781,      0x03},
    {IMX334_REG_3782,      0xF5},
    {IMX334_REG_3783,      0x06},
    {IMX334_REG_3784,      0xA5},
    {IMX334_REG_3788,      0x0F},
    {IMX334_REG_378A,      0xD9},
    {IMX334_REG_378B,      0x03},
    {IMX334_REG_378C,      0xEB},
    {IMX334_REG_378D,      0x05},
    {IMX334_REG_378E,      0x87},
    {IMX334_REG_378F,      0x06},
    {IMX334_REG_3790,      0xF5},
    {IMX334_REG_3792,      0x43},
    {IMX334_REG_3794,      0x7A},
    {IMX334_REG_3796,      0xA1},
    {IMX334_REG_37B0,      0x36},
    {IMX334_REG_3E04,      0x0E},

    {HTRIMMING_START_HIGH, 0x00},
    {HTRIMMING_START_LOW,  0x30},
    {HNUM_HIGH,            0x0F},
    {HNUM_LOW,             0x18},
    {AREA3_ST_ADR_1_HIGH,  0x00},
    {AREA3_ST_ADR_1_LOW,   0xB0},
    {AREA3_WIDTH_1_HIGH,   0x08},
    {AREA3_WIDTH_1_LOW,    0x84},
    {AREA3_ST_ADR_2_HIGH,  0x00},
    {AREA3_ST_ADR_2_LOW,   0xB1},
    {AREA3_WIDTH_2_HIGH,   0x08},
    {AREA3_WIDTH_2_LOW,    0x84},
    {BLACK_OFSET_ADR_LOW,  0x00},
    {UNRD_LINE_MAX_LOW,    0x00},
    {UNREAD_ED_ADR_HIGH,   0x11},
    {UNREAD_ED_ADR_LOW,    0x8F},

    {IMX334_TABLE_WAIT_MS, IMX334_WAIT_MS},
    {IMX334_TABLE_END,     0x0000}
};

static const imx334_reg mode_3864x2180[] = {

    {WINMODE,              0x00},
    {HADD_VADD,            0x00},
    {TCYCLE,               0x00},

    {Y_OUT_SIZE_HIGH,      IMX334_TO_MID_BYTE(2180)},
    {Y_OUT_SIZE_LOW,       IMX334_TO_LOW_BYTE(2180)},

    {IMX334_TABLE_WAIT_MS, IMX334_WAIT_MS},
    {IMX334_TABLE_END,     0x0000}
};

static const imx334_reg mode_h2v2_binning[] = {

    {WINMODE,              0x01},
    {HADD_VADD,            0x30},
    {TCYCLE,               0x01},
    {VALID_EXPAND,         0x04},

    {Y_OUT_SIZE_HIGH,      IMX334_TO_MID_BYTE(1080)},
    {Y_OUT_SIZE_LOW,       IMX334_TO_LOW_BYTE(1080)},

    {ADBIT,                0x00},
    {ADBIT1_HIGH,          0x01},
    {ADBIT1_LOW,           0xFF},
    {MDBIT,                0x01},

    {IMX334_TABLE_WAIT_MS, IMX334_WAIT_MS},
    {IMX334_TABLE_END,     0x0000}
};

static const imx334_reg mode_crop_1920x1080[] = {

    {WINMODE,               4},
    {HADD_VADD,             0},
    {TCYCLE,                0},

    {HTRIMMING_START_HIGH,  IMX334_TO_MID_BYTE(1020)},
    {HTRIMMING_START_LOW,   IMX334_TO_LOW_BYTE(1020)},
    {HNUM_HIGH,             IMX334_TO_MID_BYTE(1920)},
    {HNUM_LOW,              IMX334_TO_LOW_BYTE(1920)},

    {AREA3_ST_ADR_1_HIGH,   IMX334_TO_MID_BYTE(1276)},
    {AREA3_ST_ADR_1_LOW,    IMX334_TO_LOW_BYTE(1276)},
    {AREA3_WIDTH_1_HIGH,    IMX334_TO_MID_BYTE(1080)},
    {AREA3_WIDTH_1_LOW,     IMX334_TO_LOW_BYTE(1080)},
    {AREA3_ST_ADR_2_HIGH,   IMX334_TO_MID_BYTE(1277)},
    {AREA3_ST_ADR_2_LOW,    IMX334_TO_LOW_BYTE(1277)},
    {AREA3_WIDTH_2_HIGH,    IMX334_TO_MID_BYTE(1080)},
    {AREA3_WIDTH_2_LOW,     IMX334_TO_LOW_BYTE(1080)},

    {Y_OUT_SIZE_HIGH,       IMX334_TO_MID_BYTE(1080)},
    {Y_OUT_SIZE_LOW,        IMX334_TO_LOW_BYTE(1080)},

    {BLACK_OFSET_ADR_LOW,   0x12},
    {UNRD_LINE_MAX_LOW,     0x64},
    {UNREAD_ED_ADR_HIGH,    IMX334_TO_MID_BYTE(3644)},
    {UNREAD_ED_ADR_LOW,     IMX334_TO_LOW_BYTE(3644)},

    {IMX334_TABLE_WAIT_MS,  IMX334_WAIT_MS},
    {IMX334_TABLE_END,      0x0000}
};

static const imx334_reg mode_enable_pattern_generator[] = {

    {DIG_CLP_MODE,         0x00},
    {TPG_EN_DUOUT,         0x01},
    {TPG_COLORWIDTH,       0x11},
    {BLKLEVEL_LOW,         0x00},
    {WRJ_OPEN,             0x00},

    {IMX334_TABLE_WAIT_MS, IMX334_WAIT_MS},
    {IMX334_TABLE_END,     0x0000}
};

static const imx334_reg mode_disable_pattern_generator[] = {

    {DIG_CLP_MODE,         0x01},
    {TPG_EN_DUOUT,         0x00},
    {TPG_COLORWIDTH,       0x10},
    {BLKLEVEL_LOW,         0x32},
    {WRJ_OPEN,             0x01},

    {IMX334_TABLE_WAIT_MS, IMX334_WAIT_MS},
    {IMX334_TABLE_END,     0x0000}
};

/**
 * Enum of available frame modes
 */

enum {
    IMX334_MODE_3864x2180,
    IMX334_MODE_CROP_1920x1080,
    IMX334_MODE_H2V2_BINNING,

    IMX334_10BIT_MODE,
    IMX334_12BIT_MODE,

    IMX334_EN_PATTERN_GEN,
    IMX334_DIS_PATTERN_GEN,

    IMX334_INIT_SETTINGS,
    IMX334_MODE_START_STREAM,
    IMX334_MODE_STOP_STREAM,
};

typedef enum {
    IMX334_1782_MBPS,
    IMX334_1188_MBPS,
    IMX334_891_MBPS,
} data_rate_mode;

static const imx334_reg *data_rate_table[] = {
    [IMX334_1782_MBPS] = imx334_1782_mbps,
    [IMX334_1188_MBPS] = imx334_1188_mbps,
    [IMX334_891_MBPS]  = imx334_891_mbps,
};

/**
 * Connecting frame modes to mode tables
 */

static const imx334_reg *mode_table[] = {

    [IMX334_MODE_3864x2180]      = mode_3864x2180,
    [IMX334_MODE_CROP_1920x1080] = mode_crop_1920x1080,
    [IMX334_MODE_H2V2_BINNING]   = mode_h2v2_binning,

    [IMX334_EN_PATTERN_GEN]      = mode_enable_pattern_generator,
    [IMX334_DIS_PATTERN_GEN]     = mode_disable_pattern_generator,

    [IMX334_10BIT_MODE]          = imx334_10bit_mode,
    [IMX334_12BIT_MODE]          = imx334_12bit_mode,

    [IMX334_INIT_SETTINGS]       = imx334_init_settings,

    [IMX334_MODE_START_STREAM]   = imx334_start,
    [IMX334_MODE_STOP_STREAM]    = imx334_stop,
};

/**
 * Framerates of available frame modes
 */

static const int imx334_60fps[] = {
    60,
};
static const int imx334_120fps[] = {
    120,
};

/**
 * Connecting resolutions, framerates and mode tables
 */

static const struct camera_common_frmfmt imx334_frmfmt[] = {
    {
        .size = {IMX334_DEFAULT_WIDTH, IMX334_DEFAULT_HEIGHT},
        .framerates = imx334_60fps,
        .num_framerates = 1,
        .hdr_en = false,
        .mode = IMX334_MODE_3864x2180
    },
    {
        .size = {1920, 1080},
        .framerates = imx334_120fps,
        .num_framerates = 1,
        .hdr_en = false,
        .mode = IMX334_MODE_CROP_1920x1080
    },
    {
        .size = {1944, 1080},
        .framerates = imx334_60fps,
        .num_framerates = 1,
        .hdr_en = false,
        .mode = IMX334_MODE_H2V2_BINNING
    },
};

#endif /* __IMX334_TABLES__ */
