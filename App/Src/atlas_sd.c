/**
 * @file atlas_sd.c
 * @brief Atlas polling SD driver: optional-media probe and bounded transfers.
 *
 * Major functions:
 * - AtlasSd_Init(): configures/probes optional media.
 * - AtlasSd_ReadBlocks()/WriteBlocks(): bounded synchronous transfers.
 * - AtlasSd_DeInit(): resets the controller after removal/error.
 *
 * Only the storage task may call these functions. IDMA is deliberately unused.
 * Replaces CubeMX's bsp_driver_sd.c scaffold (originally based on ST's
 * stm32h743i_eval_sd.c); the generated weak BSP_SD_* functions are unused.
 */
#include "atlas_sd.h"
#include "main.h"
#include "atlas_time.h"
#include <string.h>

extern SD_HandleTypeDef hsd1;
static uint8_t sd_initialized;
static volatile uint32_t media_generation;
static uint32_t mounted_generation;
static AtlasSdDiagnostics diagnostics;

/** @brief Keep only the first failure since the last explicit mount.
 * @param stage Failing step (see AtlasSdDiagnostics). @param status HAL result. */
static void sd_failure(uint32_t stage, HAL_StatusTypeDef status)
{
    if (diagnostics.stage != 0U) return;
    diagnostics.stage = stage;
    diagnostics.hal_status = (uint32_t)status;
    diagnostics.hal_error = hsd1.ErrorCode;
}

/** @brief Copy retained failure evidence and detect-edge count. @param output Destination. */
void AtlasSd_GetDiagnostics(AtlasSdDiagnostics *output)
{
    if (output != NULL) { *output = diagnostics; output->detect_edges = media_generation; }
}

/** @brief Record either detect-switch edge; a replacement invalidates the old FAT. */
void AtlasSd_DetectFromISR(void)
{
    ++media_generation;
}

/** @brief Reset the optional controller and discard stale software state. */
void AtlasSd_DeInit(void)
{
    /* HAL_SD_Abort can return before disabling IDMA on an error. Peripheral
       reset, not a successful-looking handle state, establishes quiescence. */
    HAL_NVIC_DisableIRQ(SDMMC1_IRQn);
    __HAL_RCC_SDMMC1_FORCE_RESET();
    __DSB();
    __HAL_RCC_SDMMC1_RELEASE_RESET();
    HAL_NVIC_ClearPendingIRQ(SDMMC1_IRQn);
    memset(&hsd1, 0, sizeof(hsd1));
    sd_initialized = 0U;
}

/** @brief Configure and probe a card without making absence fatal. @return MSD status. */
uint8_t AtlasSd_Init(void)
{
    AtlasSd_DeInit();
    memset(&diagnostics, 0, sizeof(diagnostics));
    const uint32_t generation = media_generation;
    if (AtlasSd_IsDetected() != ATLAS_SD_PRESENT)
    { sd_failure(1U, HAL_ERROR); return ATLAS_SD_ERROR_NOT_PRESENT; }
    AtlasTime_DelayMs(20U);
    if (AtlasSd_IsDetected() != ATLAS_SD_PRESENT || media_generation != generation)
    { sd_failure(1U, HAL_ERROR); return ATLAS_SD_ERROR_NOT_PRESENT; }
    mounted_generation = generation;
    hsd1.Instance = SDMMC1;
    hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
    hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    /* Identification is single-bit. Change card AND host width only afterward. */
    hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
    hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
    hsd1.Init.ClockDiv = 1U; /* PLL2R 50 MHz / (2 * 1) = 25 MHz transfer clock. */
    HAL_StatusTypeDef status = HAL_SD_Init(&hsd1);
    if (status != HAL_OK) sd_failure(2U, status);
    else
    {
        status = HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B);
        if (status != HAL_OK) sd_failure(3U, status);
    }
    if (status != HAL_OK)
    {
        AtlasSd_DeInit();
        return ATLAS_SD_ERROR;
    }
    /* Polling owns completion; no SD ISR may consume its flags. */
    HAL_NVIC_DisableIRQ(SDMMC1_IRQn);
    if (media_generation != mounted_generation || AtlasSd_IsDetected() != ATLAS_SD_PRESENT)
    {
        AtlasSd_DeInit();
        return ATLAS_SD_ERROR_NOT_PRESENT;
    }
    sd_initialized = 1U;
    return ATLAS_SD_OK;
}

/**
 * @brief Read sectors synchronously; caller memory is CPU-accessed only.
 * @param data Destination. @param address LBA. @param count Sector count.
 * @param timeout_ms HAL transfer bound. @return MSD status.
 */
uint8_t AtlasSd_ReadBlocks(uint32_t *data, uint32_t address,
                         uint32_t count, uint32_t timeout_ms)
{
    if (!sd_initialized || data == NULL || count == 0U || timeout_ms == 0U ||
        media_generation != mounted_generation || AtlasSd_IsDetected() != ATLAS_SD_PRESENT)
        return ATLAS_SD_ERROR;
    const HAL_StatusTypeDef status = HAL_SD_ReadBlocks(&hsd1, (uint8_t *)data, address, count, timeout_ms);
    if (status != HAL_OK) sd_failure(4U, status);
    return status == HAL_OK &&
           media_generation == mounted_generation &&
           AtlasSd_IsDetected() == ATLAS_SD_PRESENT ? ATLAS_SD_OK : ATLAS_SD_ERROR;
}

/**
 * @brief Write sectors synchronously; never retains the source.
 * @param data Source. @param address LBA. @param count Sector count.
 * @param timeout_ms HAL transfer bound. @return MSD status.
 */
uint8_t AtlasSd_WriteBlocks(uint32_t *data, uint32_t address,
                          uint32_t count, uint32_t timeout_ms)
{
    if (!sd_initialized || data == NULL || count == 0U || timeout_ms == 0U ||
        media_generation != mounted_generation || AtlasSd_IsDetected() != ATLAS_SD_PRESENT)
        return ATLAS_SD_ERROR;
    const HAL_StatusTypeDef status = HAL_SD_WriteBlocks(&hsd1, (uint8_t *)data, address, count, timeout_ms);
    if (status != HAL_OK) sd_failure(5U, status);
    return status == HAL_OK &&
           media_generation == mounted_generation &&
           AtlasSd_IsDetected() == ATLAS_SD_PRESENT ? ATLAS_SD_OK : ATLAS_SD_ERROR;
}

/** @brief Probe protocol readiness, not mechanical presence. @return Transfer state. */
uint8_t AtlasSd_GetCardState(void)
{
    uint32_t state;
    if (!sd_initialized || media_generation != mounted_generation ||
        AtlasSd_IsDetected() != ATLAS_SD_PRESENT) return ATLAS_SD_TRANSFER_ERROR;
    state = HAL_SD_GetCardState(&hsd1);
    if (!AtlasSd_IsMediaCurrent()) return ATLAS_SD_TRANSFER_ERROR;
    if (state == HAL_SD_CARD_TRANSFER) return ATLAS_SD_TRANSFER_OK;
    if ((state == HAL_SD_CARD_SENDING) || (state == HAL_SD_CARD_RECEIVING) ||
        (state == HAL_SD_CARD_PROGRAMMING)) return ATLAS_SD_TRANSFER_BUSY;
    sd_failure(6U, HAL_ERROR);
    return ATLAS_SD_TRANSFER_ERROR;
}

/** @brief Obtain validated geometry. @param info Destination. @return MSD status. */
uint8_t AtlasSd_GetCardInfo(AtlasSdCardInfo *info)
{
    if (!AtlasSd_IsMediaCurrent() || info == NULL) return ATLAS_SD_ERROR;
    const HAL_StatusTypeDef status = HAL_SD_GetCardInfo(&hsd1, info);
    if (status != HAL_OK) sd_failure(7U, status);
    return status == HAL_OK && AtlasSd_IsMediaCurrent() ? ATLAS_SD_OK : ATLAS_SD_ERROR;
}

/** @brief Read J3 DET via PD3/R21 (active low). @return ATLAS_SD_PRESENT or ATLAS_SD_NOT_PRESENT. */
uint8_t AtlasSd_IsDetected(void)
{
    return HAL_GPIO_ReadPin(SD_DET_GPIO_Port, SD_DET_Pin) == GPIO_PIN_RESET ?
           ATLAS_SD_PRESENT : ATLAS_SD_NOT_PRESENT;
}
/** @brief Detect invalidated/removable-media sessions without an SD bus transaction.
 * @return One for the same initialized and physically present card. */
uint8_t AtlasSd_IsMediaCurrent(void)
{
    return sd_initialized && mounted_generation == media_generation && AtlasSd_IsDetected() == ATLAS_SD_PRESENT;
}
