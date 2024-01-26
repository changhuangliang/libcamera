/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Starfive Technology Co., Ltd.
 *
 * starfive_controls.cpp - Starfive controls definition
 */

#include "starfive_controls.h"

namespace libcamera 
{

namespace starfive
{

namespace control
{

extern const Control<Span<uint8_t, sizeof(GetModuleInformation)>> GetModuleInfo(GET_INFORMATION, "GetModuleInfo");
extern const Control<Span<uint8_t, sizeof(AGC::AECtrlParams)>> AECtrlInfo(AE_CTRL_INFOR, "AECtrlInfo");
extern const Control<Span<uint8_t, sizeof(AWB::AWBCtrlParams)>> AWBCtrlInfo(AWB_CTRL_INFOR, "AWBCtrlInfo");
extern const Control<Span<uint8_t, sizeof(AWB::AWBModuleParams)>> AWBModInfo(AWB_MOD_INFOR, "AWBModInfo");
extern const Control<Span<uint8_t, sizeof(CCM::CCMCtrlParams)>> CCMCtrlInfo(CCM_CTRL_INFOR, "CCMCtrlInfo");
extern const Control<Span<uint8_t, sizeof(CCM::CCMModuleParams)>> CCMModInfo(CCM_MOD_INFOR, "CCMModInfo");
extern const Control<Span<uint8_t, sizeof(DNYUV::DNYUVCtrlParams)>> DNYUVCtrlInfo(DNYUV_CTRL_INFOR, "DNYUVCtrlInfo");
extern const Control<Span<uint8_t, sizeof(DNYUV::DNYUVModuleParams)>> DNYUVModInfo(DNYUV_MOD_INFOR, "DNYUVModInfo");
extern const Control<Span<uint8_t, sizeof(LCCF::LCCFCtrlParams)>> LCCFCtrlInfo(LCCF_CTRL_INFOR, "LCCFCtrlInfo");
extern const Control<Span<uint8_t, sizeof(LCCF::LCCFModuleParams)>> LCCFModInfo(LCCF_MOD_INFOR, "LCCFModInfo");
extern const Control<Span<uint8_t, sizeof(SAT::SATCtrlParams)>> SATCtrlInfo(SAT_CTRL_INFOR, "SATCtrlInfo");
extern const Control<Span<uint8_t, sizeof(SAT::SATModuleParams)>> SATModInfo(SAT_MOD_INFOR, "SATModInfo");
extern const Control<Span<uint8_t, sizeof(SHARPEN::SHARPENCtrlParams)>> SHRPCtrlInfo(SHRP_CTRL_INFOR, "SHRPCtrlInfo");
extern const Control<Span<uint8_t, sizeof(SHARPEN::SHARPENModuleParams)>> SHRPModInfo(SHRP_MOD_INFOR, "SHRPModInfo");
extern const Control<Span<uint8_t, sizeof(YCRV::YCRVCtrlParams)>> YCRVCtrlInfo(YCRV_CTRL_INFOR, "YCRVCtrlInfo");
extern const Control<Span<uint8_t, sizeof(YCRV::YCRVModuleParams)>> YCRVModInfo(YCRV_MOD_INFOR, "YCRVModInfo");
extern const Control<Span<uint8_t, sizeof(CAR::CARModuleParams)>> CARModInfo(CAR_MOD_INFOR, "CARModInfo");
extern const Control<Span<uint8_t, sizeof(CFA::CFAModuleParams)>> CFAModInfo(CFA_MOD_INFOR, "CFAModInfo");
extern const Control<Span<uint8_t, sizeof(CTC::CTCModuleParams)>> CTCModInfo(CTC_MOD_INFOR, "CTCModInfo");
extern const Control<Span<uint8_t, sizeof(DBC::DBCModuleParams)>> DBCModInfo(DBC_MOD_INFOR, "DBCModInfo");
extern const Control<Span<uint8_t, sizeof(GAMMA::GAMMAModuleParams)>> GMARGBModInfo(GMARGB_MOD_INFOR, "GMARGBModInfo");
extern const Control<Span<uint8_t, sizeof(OBC::OBCModuleParams)>> OBCModInfo(OBC_MOD_INFOR, "OBCModInfo");
extern const Control<Span<uint8_t, sizeof(OECF::OECFModuleParams)>> OECFModInfo(OECF_MOD_INFOR, "OECFModInfo");
extern const Control<Span<uint8_t, sizeof(R2Y::R2YModuleParams)>> R2YModInfo(R2Y_MOD_INFOR, "R2YModInfo");
extern const Control<Span<uint8_t, sizeof(STAT::STATModuleParams)>> SCModInfo(SC_MOD_INFOR, "SCModInfo");

extern const ControlIdMap starfiveControls {
    { GET_INFORMATION, &GetModuleInfo },
    { AE_CTRL_INFOR, &AECtrlInfo },
    { AWB_CTRL_INFOR, &AWBCtrlInfo },
    { AWB_MOD_INFOR, &AWBModInfo },
    { CCM_CTRL_INFOR, &CCMCtrlInfo },
    { CCM_MOD_INFOR, &CCMModInfo },
    { DNYUV_CTRL_INFOR, &DNYUVCtrlInfo },
    { DNYUV_MOD_INFOR, &DNYUVModInfo },
    { LCCF_CTRL_INFOR, &LCCFCtrlInfo },
    { LCCF_MOD_INFOR, &LCCFModInfo },
    { SAT_CTRL_INFOR, &SATCtrlInfo },
    { SAT_MOD_INFOR, &SATModInfo },
    { SHRP_CTRL_INFOR, &SHRPCtrlInfo },
    { SHRP_MOD_INFOR, &SHRPModInfo },
    { YCRV_CTRL_INFOR, &YCRVCtrlInfo },
    { YCRV_MOD_INFOR, &YCRVModInfo },
    { CAR_MOD_INFOR, &CARModInfo },
    { CFA_MOD_INFOR, &CFAModInfo },
    { CTC_MOD_INFOR, &CTCModInfo },
    { DBC_MOD_INFOR, &DBCModInfo },
    { GMARGB_MOD_INFOR, &GMARGBModInfo },
    { OBC_MOD_INFOR, &OBCModInfo },
    { OECF_MOD_INFOR, &OECFModInfo },
    { R2Y_MOD_INFOR, &R2YModInfo },
    { SC_MOD_INFOR, &SCModInfo },
};

extern const ControlInfoMap::Map starfiveControlMap {
	{ &GetModuleInfo, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &AECtrlInfo, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &AWBCtrlInfo, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &AWBModInfo, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &CCMCtrlInfo, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &CCMModInfo, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &DNYUVCtrlInfo, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &DNYUVModInfo, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &LCCFCtrlInfo, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &LCCFModInfo, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &SATCtrlInfo, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &SATModInfo, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &SHRPCtrlInfo, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &SHRPModInfo, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &YCRVCtrlInfo, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &YCRVModInfo, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &CARModInfo, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &CFAModInfo, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &CTCModInfo, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &DBCModInfo, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &GMARGBModInfo, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &OBCModInfo, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &OECFModInfo, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &R2YModInfo, ControlInfo((uint8_t)0, (uint8_t)0) },
    { &SCModInfo, ControlInfo((uint8_t)0, (uint8_t)0) },
};

} // namespace control

} // namespace starfive

} // namespace libcamera 
