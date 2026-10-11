/**
 * @file atlas_sd.h
 * @brief Atlas polling SD driver for SDMMC1 and the FatFs volume it backs.
 *
 * Major functions:
 * - AtlasSd_Init()/DeInit(): probe/configure or reset the optional card.
 * - AtlasSd_ReadBlocks()/WriteBlocks(): bounded synchronous sector transfers.
 * - AtlasSd_GetCardState()/GetCardInfo(): readiness and validated geometry.
 * - AtlasSd_IsDetected()/IsMediaCurrent()/DetectFromISR(): J3/PD3 card detection.
 * - AtlasSd_Invalidate()/PrepareMount(): FatFs volume session control (sd_diskio.c).
 *
 * Replaces CubeMX's bsp_driver_sd.c scaffold (originally based on ST's
 * stm32h743i_eval_sd.c). No DMA or erase API is exposed.
 */
#ifndef ATLAS_SD_H
#define ATLAS_SD_H
#include "stm32h7xx_hal.h"
#ifdef __cplusplus
extern "C" {
#endif
#define AtlasSdCardInfo HAL_SD_CardInfoTypeDef
#define ATLAS_SD_OK                 ((uint8_t)0U)
#define ATLAS_SD_ERROR              ((uint8_t)1U)
#define ATLAS_SD_ERROR_NOT_PRESENT ((uint8_t)2U)
#define ATLAS_SD_TRANSFER_OK         ((uint8_t)0U)
#define ATLAS_SD_TRANSFER_BUSY       ((uint8_t)1U)
#define ATLAS_SD_TRANSFER_ERROR      ((uint8_t)2U)
#define ATLAS_SD_PRESENT             ((uint8_t)1U)
#define ATLAS_SD_NOT_PRESENT         ((uint8_t)0U)
/** @brief First failing step since the last explicit mount. Retained across reset.
 * Stages: 0 none, 1 detect, 2 initialize, 3 bus width, 4 read, 5 write,
 * 6 card state, 7 geometry. HAL error is captured before the handle is cleared. */
typedef struct { uint32_t stage, hal_status, hal_error, detect_edges; } AtlasSdDiagnostics;
void AtlasSd_GetDiagnostics(AtlasSdDiagnostics *diagnostics);
/** @brief Probe/configure optional media. @return MSD status. */
uint8_t AtlasSd_Init(void);
/** @brief Reset SDMMC1 and invalidate the handle; storage-owner context only. */
void AtlasSd_DeInit(void);
/** @brief Read synchronously. @param data Destination. @param address LBA.
 * @param count Sectors. @param timeout_ms HAL bound. @return MSD status. */
uint8_t AtlasSd_ReadBlocks(uint32_t *data, uint32_t address, uint32_t count, uint32_t timeout_ms);
/** @brief Write synchronously. @param data Source. @param address LBA.
 * @param count Sectors. @param timeout_ms HAL bound. @return MSD status. */
uint8_t AtlasSd_WriteBlocks(uint32_t *data, uint32_t address, uint32_t count, uint32_t timeout_ms);
/** @brief Probe card protocol state. @return SD_TRANSFER_* value. */
uint8_t AtlasSd_GetCardState(void);
/** @brief Read card geometry. @param info Destination. @return MSD status. */
uint8_t AtlasSd_GetCardInfo(AtlasSdCardInfo *info);
/** @brief Read raw J3 DET (active low); readiness is separate. @return Presence. */
uint8_t AtlasSd_IsDetected(void);
/** @brief Check initialized media generation/presence without sending a command.
 * @return One only while the mounted card session remains current. */
uint8_t AtlasSd_IsMediaCurrent(void);
/** @brief Record either PD3 detect edge; ISR-only, no HAL/RTOS calls. */
void AtlasSd_DetectFromISR(void);
/** @brief Invalidate/reset the optional medium; storage-owner context only. */
void AtlasSd_Invalidate(void);
/** @brief Authorize exactly one initialization as part of an explicit owner mount.
 * @note Never call from a file read/write or from another task. */
void AtlasSd_PrepareMount(void);
#ifdef __cplusplus
}
#endif
#endif
