// FDCAN message RAM layout constants for STM32G4.
// Extracted from STM32CubeG4 stm32g4xx_hal_fdcan.h (BSD 3-Clause).
// These define SRAMCAN element sizes and offsets not in the base CMSIS headers.
#pragma once

#include <cstdint>

#define FDCAN_ELEMENT_MASK_STDID ((uint32_t)0x1FFC0000U)
#define FDCAN_ELEMENT_MASK_EXTID ((uint32_t)0x1FFFFFFFU)
#define FDCAN_ELEMENT_MASK_RTR   ((uint32_t)0x20000000U)
#define FDCAN_ELEMENT_MASK_XTD   ((uint32_t)0x40000000U)
#define FDCAN_ELEMENT_MASK_ESI   ((uint32_t)0x80000000U)
#define FDCAN_ELEMENT_MASK_TS    ((uint32_t)0x0000FFFFU)
#define FDCAN_ELEMENT_MASK_DLC   ((uint32_t)0x000F0000U)
#define FDCAN_ELEMENT_MASK_BRS   ((uint32_t)0x00100000U)
#define FDCAN_ELEMENT_MASK_FDF   ((uint32_t)0x00200000U)
#define FDCAN_ELEMENT_MASK_EFC   ((uint32_t)0x00800000U)
#define FDCAN_ELEMENT_MASK_MM    ((uint32_t)0xFF000000U)
#define FDCAN_ELEMENT_MASK_FIDX  ((uint32_t)0x7F000000U)
#define FDCAN_ELEMENT_MASK_ANMF  ((uint32_t)0x80000000U)

#define SRAMCAN_FLS_NBR       (28U)
#define SRAMCAN_FLE_NBR       ( 8U)
#define SRAMCAN_RF0_NBR       ( 3U)
#define SRAMCAN_RF1_NBR       ( 3U)
#define SRAMCAN_TEF_NBR       ( 3U)
#define SRAMCAN_TFQ_NBR       ( 3U)

#define SRAMCAN_FLS_SIZE      ( 1U * 4U)
#define SRAMCAN_FLE_SIZE      ( 2U * 4U)
#define SRAMCAN_RF0_SIZE      (18U * 4U)
#define SRAMCAN_RF1_SIZE      (18U * 4U)
#define SRAMCAN_TEF_SIZE      ( 2U * 4U)
#define SRAMCAN_TFQ_SIZE      (18U * 4U)

#define SRAMCAN_FLSSA ((uint32_t)0)
#define SRAMCAN_FLESA ((uint32_t)(SRAMCAN_FLSSA + (SRAMCAN_FLS_NBR * SRAMCAN_FLS_SIZE)))
#define SRAMCAN_RF0SA ((uint32_t)(SRAMCAN_FLESA + (SRAMCAN_FLE_NBR * SRAMCAN_FLE_SIZE)))
#define SRAMCAN_RF1SA ((uint32_t)(SRAMCAN_RF0SA + (SRAMCAN_RF0_NBR * SRAMCAN_RF0_SIZE)))
#define SRAMCAN_TEFSA ((uint32_t)(SRAMCAN_RF1SA + (SRAMCAN_RF1_NBR * SRAMCAN_RF1_SIZE)))
#define SRAMCAN_TFQSA ((uint32_t)(SRAMCAN_TEFSA + (SRAMCAN_TEF_NBR * SRAMCAN_TEF_SIZE)))
#define SRAMCAN_SIZE  ((uint32_t)(SRAMCAN_TFQSA + (SRAMCAN_TFQ_NBR * SRAMCAN_TFQ_SIZE)))
