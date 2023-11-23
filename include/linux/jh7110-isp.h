/* SPDX-License-Identifier: ((GPL-2.0+ WITH Linux-syscall-note) OR BSD-3-Clause) */
/*
 * jh7110-isp.h
 *
 * JH7110 ISP driver - user space header file.
 *
 * Copyright © 2023 Starfive Technology Co., Ltd.
 *
 * Author: Su Zejian (zejian.su@starfivetech.com)
 *
 */

#ifndef __JH7110_ISP_H_
#define __JH7110_ISP_H_

#include <linux/v4l2-controls.h>

#define V4L2_CID_USER_JH7110_ISP_BASE				(V4L2_CID_USER_BASE + 0x1170)

#define V4L2_CID_USER_JH7110_ISP_WB_SETTING	\
				(V4L2_CID_USER_JH7110_ISP_BASE + 0x0001)
#define V4L2_CID_USER_JH7110_ISP_CAR_SETTING	\
				(V4L2_CID_USER_JH7110_ISP_BASE + 0x0002)
#define V4L2_CID_USER_JH7110_ISP_CCM_SETTING	\
				(V4L2_CID_USER_JH7110_ISP_BASE + 0x0003)
#define V4L2_CID_USER_JH7110_ISP_CFA_SETTING		\
				(V4L2_CID_USER_JH7110_ISP_BASE + 0x0004)
#define V4L2_CID_USER_JH7110_ISP_CTC_SETTING		\
				(V4L2_CID_USER_JH7110_ISP_BASE + 0x0005)
#define V4L2_CID_USER_JH7110_ISP_DBC_SETTING	\
				(V4L2_CID_USER_JH7110_ISP_BASE + 0x0006)
#define V4L2_CID_USER_JH7110_ISP_DNYUV_SETTING	\
				(V4L2_CID_USER_JH7110_ISP_BASE + 0x0007)
#define V4L2_CID_USER_JH7110_ISP_GMARGB_SETTING		\
				(V4L2_CID_USER_JH7110_ISP_BASE + 0x0008)
#define V4L2_CID_USER_JH7110_ISP_LCCF_SETTING \
				(V4L2_CID_USER_JH7110_ISP_BASE + 0x0009)
#define V4L2_CID_USER_JH7110_ISP_OBC_SETTING	\
				(V4L2_CID_USER_JH7110_ISP_BASE + 0x000a)
#define V4L2_CID_USER_JH7110_ISP_OECF_SETTING	\
				(V4L2_CID_USER_JH7110_ISP_BASE + 0x000b)
#define V4L2_CID_USER_JH7110_ISP_R2Y_SETTING	\
				(V4L2_CID_USER_JH7110_ISP_BASE + 0x000c)
#define V4L2_CID_USER_JH7110_ISP_SAT_SETTING		\
				(V4L2_CID_USER_JH7110_ISP_BASE + 0x000d)
#define V4L2_CID_USER_JH7110_ISP_SHRP_SETTING		\
				(V4L2_CID_USER_JH7110_ISP_BASE + 0x000e)
#define V4L2_CID_USER_JH7110_ISP_YCRV_SETTING	\
				(V4L2_CID_USER_JH7110_ISP_BASE + 0x000f)
#define V4L2_CID_USER_JH7110_ISP_STAT_SETTING \
				(V4L2_CID_USER_JH7110_ISP_BASE + 0x0010)
#define V4L2_CID_USER_JH7110_ISP_OUTSS0_SETTING \
				(V4L2_CID_USER_JH7110_ISP_BASE + 0x0011)
#define V4L2_CID_USER_JH7110_ISP_OUTSS1_SETTING \
				(V4L2_CID_USER_JH7110_ISP_BASE + 0x0012)

struct jh7110_isp_wb_gain {
	__u16 gain_r;
	__u16 gain_g;
	__u16 gain_b;
};

struct jh7110_isp_wb_setting {
	__u32 enabled;
	struct jh7110_isp_wb_gain gains;
};

struct jh7110_isp_car_setting {
	__u32 enabled;
};

struct jh7110_isp_ccm_smlow {
	__s32 ccm[3][3];
	__s32 offsets[3];
};

struct jh7110_isp_ccm_setting {
	__u32 enabled;
	struct jh7110_isp_ccm_smlow ccm_smlow;
};

struct jh7110_isp_cfa_params {
	__s32 hv_width;
	__s32 cross_cov;
};

struct jh7110_isp_cfa_setting {
	__u32 enabled;
	struct jh7110_isp_cfa_params settings;
};

struct jh7110_isp_ctc_params {
	__u8 saf_mode;
	__u8 daf_mode;
	__s32 max_gt;
	__s32 min_gt;
};

struct jh7110_isp_ctc_setting {
	__u32 enabled;
	struct jh7110_isp_ctc_params settings;
};

struct jh7110_isp_dbc_params {
	__s32 bad_gt;
	__s32 bad_xt;
};

struct jh7110_isp_dbc_setting {
	__u32 enabled;
	struct jh7110_isp_dbc_params settings;
};

struct jh7110_isp_dnyuv_params {
	__u8 y_sweight[10];
	__u16 y_curve[7];
	__u8 uv_sweight[10];
	__u16 uv_curve[7];
};

struct jh7110_isp_dnyuv_setting {
	__u32 enabled;
	struct jh7110_isp_dnyuv_params settings;
};

struct jh7110_isp_gmargb_point {
	__u16 g_val;
	__u16 sg_val;
};

struct jh7110_isp_gmargb_setting {
	__u32 enabled;
	struct jh7110_isp_gmargb_point curve[15];
};

struct jh7110_isp_lccf_circle {
	__s16 center_x;
	__s16 center_y;
	__u8 radius;
};

struct jh7110_isp_lccf_curve_param {
	__s16 f1;
	__s16 f2;
};

struct jh7110_isp_lccf_setting {
	__u32 enabled;
	struct jh7110_isp_lccf_circle circle;
	struct jh7110_isp_lccf_curve_param r_param;
	struct jh7110_isp_lccf_curve_param gr_param;
	struct jh7110_isp_lccf_curve_param gb_param;
	struct jh7110_isp_lccf_curve_param b_param;
};

struct jh7110_isp_blacklevel_win_size {
	__u32 width;
	__u32 height;
};

struct jh7110_isp_blacklevel_gain {
	__u8 tl_gain;
	__u8 tr_gain;
	__u8 bl_gain;
	__u8 br_gain;
};

struct jh7110_isp_blacklevel_offset {
	__u8 tl_offset;
	__u8 tr_offset;
	__u8 bl_offset;
	__u8 br_offset;
};

struct jh7110_isp_blacklevel_setting {
	__u32 enabled;
	struct jh7110_isp_blacklevel_win_size win_size;
	struct jh7110_isp_blacklevel_gain gain[4];
	struct jh7110_isp_blacklevel_offset offset[4];
};

struct jh7110_isp_oecf_point {
	__u16 x;
	__u16 y;
	__s16 slope;
};

struct jh7110_isp_oecf_setting {
	__u32 enabled;
	struct jh7110_isp_oecf_point r_curve[16];
	struct jh7110_isp_oecf_point gr_curve[16];
	struct jh7110_isp_oecf_point gb_curve[16];
	struct jh7110_isp_oecf_point b_curve[16];
};

struct jh7110_isp_r2y_matrix {
	__s16 m[9];
};

struct jh7110_isp_r2y_setting {
	__u32 enabled;
	struct jh7110_isp_r2y_matrix matrix;
};

struct jh7110_isp_sat_curve {
	__s16 yi_min;
	__s16 yo_ir;
	__s16 yo_min;
	__s16 yo_max;
};

struct jh7110_isp_sat_hue_info {
	__s16 cos;
	__s16 sin;
};

struct jh7110_isp_sat_info {
	__s16 gain_cmab;
	__s16 gain_cmmd;
	__s16 threshold_cmb;
	__s16 threshold_cmd;
	__s16 offset_u;
	__s16 offset_v;
	__s16 cmsf;
};

struct jh7110_isp_sat_setting {
	__u32 enabled;
	struct jh7110_isp_sat_curve curve;
	struct jh7110_isp_sat_hue_info hue_info;
	struct jh7110_isp_sat_info sat_info;
};

struct jh7110_isp_sharp_weight {
	__u8 weight[15];
	__u32 recip_wei_sum;
};

struct jh7110_isp_sharp_strength {
	__s16 diff[4];
	__s16 f[3];
	__s32 s[3];
};

struct jh7110_isp_sharp_setting {
	__u32 enabled;
	struct jh7110_isp_sharp_weight weight;
	struct jh7110_isp_sharp_strength strength;
	__s8 pdirf;
	__s8 ndirf;
};

struct jh7110_isp_ycrv_curve {
	__s16 y[64];
};

struct jh7110_isp_ycrv_setting {
	__u32 enabled;
	struct jh7110_isp_ycrv_curve curve;
};

struct jh7110_isp_sc_config {
	__u16 h_start;
	__u16 v_start;
	__u8 sw_width;
	__u8 sw_height;
	__u8 hperiod;
	__u8 hkeep;
	__u8 vperiod;
	__u8 vkeep;
};

struct jh7110_isp_sc_af_config {
	__u8 es_hor_mode;
	__u8 es_sum_mode;
	__u8 hor_en;
	__u8 ver_en;
	__u8 es_ver_thr;
	__u16 es_hor_thr;
};

struct jh7110_isp_sc_awb_ps {
	__u8 awb_ps_rl;
	__u8 awb_ps_ru;
	__u8 awb_ps_gl;
	__u8 awb_ps_gu;
	__u8 awb_ps_bl;
	__u8 awb_ps_bu;
	__u8 awb_ps_yl;
	__u8 awb_ps_yu;
	__u16 awb_ps_grl;
	__u16 awb_ps_gru;
	__u16 awb_ps_gbl;
	__u16 awb_ps_gbu;
	__u16 awb_ps_grbl;
	__u16 awb_ps_grbu;
};

struct jh7110_isp_sc_awb_ws {
	__u8 awb_ws_rl;
	__u8 awb_ws_ru;
	__u8 awb_ws_grl;
	__u8 awb_ws_gru;
	__u8 awb_ws_gbl;
	__u8 awb_ws_gbu;
	__u8 awb_ws_bl;
	__u8 awb_ws_bu;
};


struct jh7110_isp_sc_awb_point {
	__u16 intensity;
	__u8 weight;
};

struct jh7110_isp_sc_awb_config {
	struct jh7110_isp_sc_awb_ps ws_ps_config;
	__u8 awb_ps_grb_ba;
	__u8 sel;
	struct jh7110_isp_sc_awb_ws ws_config;
	__u8 awb_cw[169];
	struct jh7110_isp_sc_awb_point pts[17];
};

struct jh7110_isp_sc_setting {
	__u32 enabled;
	struct jh7110_isp_sc_config crop_config;
	struct jh7110_isp_sc_af_config af_config;
	struct jh7110_isp_sc_awb_config awb_config;
};

struct jh7110_isp_outss_setting {
	__u8 which;
	__u16 stride;	// Output Image Stride Register, 8-byte(64bit) granularity.
	__u8 hsm;		// horizontal scale mode
	__u32 hsf;		// horizontal scale factor (time 4096)
	__u8 vsm;		// vertical scale mode
	__u32 vsf;		// vertical scale factor (time 4096)
};

struct jh7110_isp_sc_buffer {
	__u32 y_histogram[64];
	__u32 reserv0[33];
	__u32 bright_sc[4096];
	__u32 reserv1[96];
	__u32 ae_hist_y[128];
	__u32 reserv2[511];
	__u16 flag;
};

#endif
