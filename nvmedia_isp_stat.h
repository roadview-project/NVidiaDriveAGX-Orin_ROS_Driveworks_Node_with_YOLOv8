/*
  * Copyright (c) 2018-2022, NVIDIA CORPORATION.  All rights reserved.  All
  * information contained herein is proprietary and confidential to NVIDIA
  * Corporation.  Any use, reproduction, or disclosure without the written
  * permission of NVIDIA Corporation is prohibited.
  */

#include <stdint.h>
#include "nvmedia_core.h"


#ifndef NVMEDIA_ISP_STAT_H
#define NVMEDIA_ISP_STAT_H



#ifdef __cplusplus
 extern "C" {
 #endif

 #define NVM_ISP_MAX_INPUT_PLANES            (3U)

 #define NVM_ISP_MAX_COLOR_COMPONENT         (4U)

 #define NVM_ISP_MAX_COLORMATRIX_DIM         (3U)

 #define NVM_ISP_RADTF_POINTS                (6U)

 #define NVM_ISP_HIST_KNEE_POINTS            (8U)

 #define NVM_ISP_MAX_LAC_ROI                 (4U)

 #define NVM_ISP_MAX_LAC_ROI_WINDOWS         (32U * 32U)

 #define NVM_ISP_HIST_BINS                   (256U)

 #define NVM_ISP_LTM_HIST_BINS               (128U)

 #define NVM_ISP_LTM_AVG_WINDOWS             (8U)

 #define NVM_ISP_MAX_FB_BANDS                (256U)

 typedef struct {
     float_t x;
     float_t y;
     double_t slope;
 } NvMediaISPSplineControlPoint;

 typedef struct {
     NvMediaPointFloat center;
     uint32_t horizontalAxis;
     uint32_t verticalAxis;
     float_t angle;
 } NvMediaISPEllipse;

 typedef struct {
     NvMediaISPEllipse radialTransform;
     NvMediaISPSplineControlPoint controlPoints[NVM_ISP_RADTF_POINTS];
 } NvMediaISPRadialTF;

 typedef struct {
     NvMediaBool enable;
     float_t offset;
     uint8_t knees[NVM_ISP_HIST_KNEE_POINTS];
     uint8_t ranges[NVM_ISP_HIST_KNEE_POINTS];
     NvMediaRect rectangularMask;
     NvMediaBool ellipticalMaskEnable;
     NvMediaISPEllipse ellipticalMask;
     NvMediaBool ellipticalWeightEnable;
     NvMediaISPRadialTF radialTF;
 } NvMediaISPHistogramStats;

 typedef struct {
     uint32_t width;
     uint32_t height;
     uint32_t numWindowsH;
     uint32_t numWindowsV;
     uint32_t horizontalInterval;
     uint32_t verticalInterval;
     NvMediaPoint startOffset;
 } NvMediaISPStatisticsWindows;

 typedef struct {
     NvMediaBool enable;
     float_t min[NVM_ISP_MAX_COLOR_COMPONENT];
     float_t max[NVM_ISP_MAX_COLOR_COMPONENT];
     NvMediaBool roiEnable[NVM_ISP_MAX_LAC_ROI];
     NvMediaISPStatisticsWindows windows[NVM_ISP_MAX_LAC_ROI];
     NvMediaBool ellipticalMaskEnable[NVM_ISP_MAX_LAC_ROI];
     NvMediaISPEllipse ellipticalMask;
 } NvMediaISPLocalAvgClipStats;

 typedef struct {
     NvMediaBool enable;
     NvMediaRect rectangularMask;
 } NvMediaISPBadPixelStats;

 typedef struct {
     NvMediaBool enable;
     NvMediaRect rectangularMask;
     NvMediaBool ellipticalMaskEnable;
     NvMediaISPEllipse ellipticalMask;
 } NvMediaISPLocalToneMapStats;

 typedef struct {
     NvMediaBool enable;
     NvMediaPoint startOffset;
     uint16_t bandCount;
     uint32_t bandWidth;
     uint32_t bandHeight;
     float_t min;
     float_t max;
     NvMediaBool ellipticalMaskEnable;
     NvMediaISPEllipse ellipticalMask;
 } NvMediaISPFlickerBandStats;

 typedef struct {
     uint32_t data[NVM_ISP_HIST_BINS][NVM_ISP_MAX_COLOR_COMPONENT];
     uint32_t excludedCount[NVM_ISP_MAX_COLOR_COMPONENT];
 } NvMediaISPHistogramStatsData;

 typedef struct {
     uint32_t numWindowsH;
     uint32_t numWindowsV;
     float_t average[NVM_ISP_MAX_LAC_ROI_WINDOWS][NVM_ISP_MAX_COLOR_COMPONENT];
     uint32_t maskedOffCount[NVM_ISP_MAX_LAC_ROI_WINDOWS][NVM_ISP_MAX_COLOR_COMPONENT];
     uint32_t clippedCount[NVM_ISP_MAX_LAC_ROI_WINDOWS][NVM_ISP_MAX_COLOR_COMPONENT];
 } NvMediaISPLocalAvgClipStatsROIData;

 typedef struct {
     NvMediaISPLocalAvgClipStatsROIData data[NVM_ISP_MAX_LAC_ROI];
 } NvMediaISPLocalAvgClipStatsData;

 typedef struct {
     uint32_t histogram[NVM_ISP_LTM_HIST_BINS];
     float_t localAverageTone[NVM_ISP_LTM_AVG_WINDOWS][NVM_ISP_LTM_AVG_WINDOWS];
     uint32_t nonMaskedCount[NVM_ISP_LTM_AVG_WINDOWS][NVM_ISP_LTM_AVG_WINDOWS];
 } NvMediaISPLocalToneMapStatsData;

 typedef struct {
     uint32_t highInWin;
     uint32_t lowInWin;
     uint32_t highMagInWin;
     uint32_t lowMagInWin;
     uint32_t highOutWin;
     uint32_t lowOutWin;
     uint32_t highMagOutWin;
     uint32_t lowMagOutWin;
 } NvMediaISPBadPixelStatsData;

 typedef struct {
     uint32_t bandCount;
     float_t luminance[NVM_ISP_MAX_FB_BANDS];
 } NvMediaISPFlickerBandStatsData;

 #ifdef __cplusplus
 }     /* extern "C" */
 #endif

 #endif /* NVMEDIA_ISP_STAT_H */
