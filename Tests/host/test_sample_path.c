/**
 * @file test_sample_path.c
 * @brief Host tests for the estimation data path: rings, clock mapping, FIFO decoding,
 *        and split-phase magnetometer/barometer sampling.
 *
 * Major functions:
 * - test_ring_*(): SPSC ordering, full-ring refusal, and free-running index wrap.
 * - test_clock_map_*(): nominal conversion, period calibration, resync, and wrap.
 * - test_lsm6_fifo_*(): flight register encoding, tag decoding in both word orders,
 *   gaps, saturation, split bursts, status decoding, clock pairs, and polled bursts.
 * - test_mmc_split_phase(): trigger/poll/collect, stale-flag guard, and timeout.
 * - test_ms5611_split_phase(): D2/D1 sequencing, early-read guard, and zero rejection.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "atlas_clock_map.h"
#include "atlas_lsm6dsv16b.h"
#include "atlas_mmc5983ma.h"
#include "atlas_ms5611.h"
#include "atlas_sample_ring.h"
#include "atlas_time.h"

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #condition); \
        exit(1); \
    } \
} while (0)

#define CHECK_NEAR(actual, expected, tolerance) \
    CHECK(fabs((double)(actual) - (double)(expected)) <= (double)(tolerance))

/* ------------------------------------------------------------------ SPI model */

static SPI_HandleTypeDef test_lsm_spi;
static SPI_HandleTypeDef test_mmc_spi;
static uint8_t test_lsm_registers[128];
static uint8_t test_mmc_registers[64];
static uint8_t test_fifo[64 * ATLAS_LSM6DSV16B_FIFO_WORD_BYTES];
static size_t test_fifo_bytes;
static size_t test_fifo_offset;
static unsigned test_fifo_transactions;
static bool test_mmc_done_on_trigger;
static unsigned test_mmc_status_reads;

/**
 * @brief Register-array SPI model plus an LSM6 FIFO_DATA_OUT stream at 0x78.
 */
static HAL_StatusTypeDef test_spi_transfer(SPI_HandleTypeDef *spi,
                                           uint8_t *tx,
                                           uint8_t *rx,
                                           uint16_t length,
                                           uint32_t timeout_ms)
{
    const bool read = (tx[0] & 0x80U) != 0U;
    uint8_t *registers;
    uint8_t mask;
    uint8_t address;
    uint16_t index;

    (void)timeout_ms;
    if (spi == &test_lsm_spi)
    {
        registers = test_lsm_registers;
        mask = 0x7FU;
    }
    else if (spi == &test_mmc_spi)
    {
        registers = test_mmc_registers;
        mask = 0x3FU;
    }
    else
    {
        return HAL_ERROR;
    }
    address = (uint8_t)(tx[0] & mask);
    if (read)
    {
        if ((spi == &test_lsm_spi) && (address == 0x78U))
        {
            /* FIFO_DATA_OUT: consecutive words, as the device's 0x7E->0x78 wrap gives. */
            ++test_fifo_transactions;
            for (index = 1U; index < length; ++index)
            {
                rx[index] = (test_fifo_offset < test_fifo_bytes) ?
                            test_fifo[test_fifo_offset++] : 0U;
            }
            return HAL_OK;
        }
        if ((spi == &test_mmc_spi) && (address == 0x08U))
        {
            ++test_mmc_status_reads;
        }
        for (index = 1U; index < length; ++index)
        {
            rx[index] = registers[(uint8_t)(address + index - 1U) & mask];
        }
        return HAL_OK;
    }
    registers[address] = tx[1];
    if ((spi == &test_lsm_spi) && (address == 0x12U) && ((tx[1] & 0x01U) != 0U))
    {
        registers[address] = 0U; /* SW_RESET self-clears. */
    }
    if ((spi == &test_mmc_spi) && (address == 0x0AU) && ((tx[1] & 0x80U) != 0U))
    {
        const uint8_t product_id = registers[0x2FU];
        memset(registers, 0, sizeof(test_mmc_registers));
        registers[0x2FU] = product_id;
    }
    if ((spi == &test_mmc_spi) && (address == 0x09U) && ((tx[1] & 0x01U) != 0U))
    {
        if (test_mmc_done_on_trigger) { registers[0x08U] |= 0x01U; }
        else { registers[0x08U] &= (uint8_t)~0x01U; }
    }
    return HAL_OK;
}

/* ------------------------------------------------------------------ helpers */

static TIM_TypeDef test_tim2_instance;
static TIM_HandleTypeDef test_tim2;

/** @brief Register a controllable 1 MHz counter as the shared timebase. */
static void test_use_microseconds(uint32_t now_us)
{
    memset(&test_tim2_instance, 0, sizeof(test_tim2_instance));
    memset(&test_tim2, 0, sizeof(test_tim2));
    test_tim2.Instance = &test_tim2_instance;
    test_tim2.State = HAL_TIM_STATE_READY;
    CHECK(AtlasTime_UseMicrosecondCounter(&test_tim2) == ATLAS_OK);
    test_tim2.counter = now_us;
}

/** @brief Append one tagged FIFO word. */
static void fifo_word(uint8_t *bytes, size_t *words, uint8_t tag, uint8_t tag_count,
                      int16_t x, int16_t y, int16_t z)
{
    uint8_t *const word = &bytes[*words * ATLAS_LSM6DSV16B_FIFO_WORD_BYTES];
    word[0] = (uint8_t)((tag << 3) | ((tag_count & 3U) << 1));
    word[1] = (uint8_t)x; word[2] = (uint8_t)((uint16_t)x >> 8);
    word[3] = (uint8_t)y; word[4] = (uint8_t)((uint16_t)y >> 8);
    word[5] = (uint8_t)z; word[6] = (uint8_t)((uint16_t)z >> 8);
    ++*words;
}

/** @brief Append one FIFO timestamp word. */
static void fifo_timestamp(uint8_t *bytes, size_t *words, uint8_t tag_count, uint32_t ticks)
{
    uint8_t *const word = &bytes[*words * ATLAS_LSM6DSV16B_FIFO_WORD_BYTES];
    memset(word, 0, ATLAS_LSM6DSV16B_FIFO_WORD_BYTES);
    word[0] = (uint8_t)((0x04U << 3) | ((tag_count & 3U) << 1));
    word[1] = (uint8_t)ticks; word[2] = (uint8_t)(ticks >> 8);
    word[3] = (uint8_t)(ticks >> 16); word[4] = (uint8_t)(ticks >> 24);
    ++*words;
}

/** @brief Initialize a flight-configured LSM6 on the SPI model. */
static void init_flight_lsm(AtlasLsm6dsv16b *sensor)
{
    const AtlasLsm6dsv16bConfig config = AtlasLsm6dsv16b_FlightConfig();

    memset(&test_lsm_spi, 0, sizeof(test_lsm_spi));
    memset(test_lsm_registers, 0, sizeof(test_lsm_registers));
    test_lsm_registers[0x0FU] = ATLAS_LSM6DSV16B_WHO_AM_I;
    AtlasTest_SetSpiTransferHook(test_spi_transfer);
    CHECK(AtlasLsm6dsv16b_Init(sensor, &test_lsm_spi, &atlas_test_gpio_g, GPIO_PIN_5,
                               &config) == ATLAS_OK);
}

/* Nominal timestamp ticks per 480 Hz sample: (1e6 / 480) / 21.75. */
#define TICKS_PER_SAMPLE (96U)

/* ------------------------------------------------------------------ ring */

static void test_ring_order_and_overflow(void)
{
    AtlasSampleRing ring;
    uint32_t storage[4];
    uint32_t out[8] = {0U};
    uint32_t value;

    CHECK(!AtlasSampleRing_Init(&ring, storage, sizeof(storage[0]), 3U));
    CHECK(!AtlasSampleRing_Init(&ring, storage, 0U, 4U));
    CHECK(AtlasSampleRing_Init(&ring, storage, sizeof(storage[0]), 4U));
    for (value = 1U; value <= 4U; ++value)
    {
        CHECK(AtlasSampleRing_Push(&ring, &value));
    }
    value = 5U;
    CHECK(!AtlasSampleRing_Push(&ring, &value));      /* full: newest refused */
    CHECK(ring.overflows == 1U);
    CHECK(AtlasSampleRing_Count(&ring) == 4U);
    CHECK(AtlasSampleRing_Pop(&ring, out, 3U) == 3U);
    CHECK((out[0] == 1U) && (out[1] == 2U) && (out[2] == 3U));
    value = 6U;
    CHECK(AtlasSampleRing_Push(&ring, &value));
    CHECK(AtlasSampleRing_Pop(&ring, out, 8U) == 2U);
    CHECK((out[0] == 4U) && (out[1] == 6U));
    CHECK(AtlasSampleRing_Pop(&ring, out, 8U) == 0U);
}

static void test_ring_index_wrap(void)
{
    AtlasSampleRing ring;
    uint16_t storage[4];
    uint16_t out[4] = {0U};
    uint16_t value;

    CHECK(AtlasSampleRing_Init(&ring, storage, sizeof(storage[0]), 4U));
    ring.head = ring.tail = UINT32_MAX - 1U;          /* free-running counters wrap */
    for (value = 10U; value < 13U; ++value)
    {
        CHECK(AtlasSampleRing_Push(&ring, &value));
    }
    CHECK(AtlasSampleRing_Count(&ring) == 3U);
    CHECK(AtlasSampleRing_Pop(&ring, out, 4U) == 3U);
    CHECK((out[0] == 10U) && (out[1] == 11U) && (out[2] == 12U));
}

/* ------------------------------------------------------------------ clock map */

static void test_clock_map_nominal_and_calibration(void)
{
    AtlasClockMap map;
    uint32_t us = 0U;
    uint32_t ticks;
    uint32_t mcu;
    int step;

    AtlasClockMap_Init(&map, 21.75F);
    CHECK(!AtlasClockMap_ToMcuUs(&map, 100U, &us));
    AtlasClockMap_AddPair(&map, 1000U, 5000000U);
    CHECK(AtlasClockMap_ToMcuUs(&map, 1100U, &us));
    CHECK(us == 5000000U + 2175U);
    CHECK(AtlasClockMap_ToMcuUs(&map, 900U, &us));     /* before the anchor */
    CHECK(us == 5000000U - 2175U);

    /* The sensor actually ticks every 22.0 us (1.1% slow): pairs every 0.25 s. */
    ticks = 1000U;
    mcu = 5000000U;
    for (step = 0; step < 12; ++step)
    {
        ticks += 11364U;
        mcu += (uint32_t)(11364.0 * 22.0 + 0.5);
        AtlasClockMap_AddPair(&map, ticks, mcu);
    }
    CHECK(map.period_measured);
    CHECK(map.resyncs == 0U);
    CHECK_NEAR(map.us_per_tick, 22.0F, 0.001F);
    CHECK(AtlasClockMap_ToMcuUs(&map, ticks - 1000U, &us));
    CHECK((int32_t)(us - (mcu - 22000U)) <= 2 && (int32_t)(us - (mcu - 22000U)) >= -2);
}

static void test_clock_map_resync_and_wrap(void)
{
    AtlasClockMap map;
    uint32_t us = 0U;

    AtlasClockMap_Init(&map, 21.75F);
    AtlasClockMap_AddPair(&map, UINT32_MAX - 50U, UINT32_MAX - 1000U);
    CHECK(AtlasClockMap_ToMcuUs(&map, 49U, &us));      /* both counters wrap */
    CHECK(us == (uint32_t)(UINT32_MAX - 1000U + 2175U));

    /* A sensor reset makes the counter restart; the map re-anchors. */
    AtlasClockMap_AddPair(&map, 10U, 2000000U);
    CHECK(map.resyncs == 1U);
    CHECK(AtlasClockMap_ToMcuUs(&map, 110U, &us));
    CHECK(us == 2000000U + 2175U);
}

/* ------------------------------------------------------------------ LSM6 FIFO */

static void test_lsm6_fifo_flight_registers(void)
{
    AtlasLsm6dsv16b sensor;
    AtlasLsm6dsv16bConfig bad = AtlasLsm6dsv16b_FlightConfig();

    init_flight_lsm(&sensor);
    CHECK(test_lsm_registers[0x10U] == 0x08U);          /* accel 480 Hz */
    CHECK(test_lsm_registers[0x11U] == 0x08U);          /* gyro 480 Hz */
    CHECK(test_lsm_registers[0x15U] == 0x0CU);          /* +/-4000 dps */
    CHECK(test_lsm_registers[0x17U] == 0x03U);          /* +/-16 g */
    CHECK(test_lsm_registers[0x0DU] == 0x00U);          /* no DRDY on INT1 */
    CHECK(test_lsm_registers[0x09U] == 0x88U);          /* BDR gyro|accel 480 Hz */
    CHECK(test_lsm_registers[0x0AU] == 0x56U);          /* TS every BDR, temp, stream */
    CHECK((test_lsm_registers[0x50U] & 0x40U) != 0U);   /* TIMESTAMP_EN */

    bad.accel_odr = ATLAS_LSM6_ODR_240_HZ;               /* FIFO pairing needs one rate */
    CHECK(AtlasLsm6dsv16b_Init(&sensor, &test_lsm_spi, &atlas_test_gpio_g, GPIO_PIN_5,
                               &bad) == ATLAS_ERROR_ARGUMENT);
    AtlasTest_SetSpiTransferHook(NULL);
}

static void test_lsm6_fifo_decode_orders(void)
{
    AtlasLsm6dsv16b sensor;
    uint8_t bytes[16 * ATLAS_LSM6DSV16B_FIFO_WORD_BYTES];
    AtlasImuSample samples[8];
    size_t words = 0U;
    size_t count;
    const float dps_to_rad = 0.017453292519943295F;

    init_flight_lsm(&sensor);
    AtlasClockMap_AddPair(&sensor.clock, 1000U, 7000000U);

    /* Slot 0: timestamp first (Linux st_lsm6dsx order). */
    fifo_timestamp(bytes, &words, 0U, 1000U);
    fifo_word(bytes, &words, 0x01U, 0U, 100, -200, 300);     /* gyro */
    fifo_word(bytes, &words, 0x02U, 0U, 2049, 0, -2049);     /* accel */
    /* Slot 1: data first, timestamp last. */
    fifo_word(bytes, &words, 0x02U, 1U, 1, 2, 3);
    fifo_word(bytes, &words, 0x01U, 1U, 4, 5, 6);
    fifo_timestamp(bytes, &words, 1U, 1000U + TICKS_PER_SAMPLE);
    /* Temperature words do not disturb slots. */
    fifo_word(bytes, &words, 0x03U, 2U, 512, 0, 0);

    count = AtlasLsm6dsv16b_DecodeFifo(&sensor, bytes, words, samples, 8U);
    CHECK(count == 2U);
    CHECK(samples[0].timestamp_us == 7000000U);
    CHECK(samples[1].timestamp_us == 7000000U + 2088U);  /* 96 * 21.75 us */
    CHECK((samples[0].sequence == 0U) && (samples[1].sequence == 1U));
    CHECK(samples[0].flags == (ATLAS_IMU_FLAG_GYRO_VALID | ATLAS_IMU_FLAG_ACCEL_VALID));
    CHECK(samples[1].flags == (ATLAS_IMU_FLAG_GYRO_VALID | ATLAS_IMU_FLAG_ACCEL_VALID));
    CHECK_NEAR(samples[0].gyro_rad_s[0], 100.0F * 0.140F * dps_to_rad, 1e-6);
    CHECK_NEAR(samples[0].gyro_rad_s[1], -200.0F * 0.140F * dps_to_rad, 1e-6);
    CHECK_NEAR(samples[0].accel_m_s2[0], 2049.0F * 0.000488F * 9.80665F, 1e-4);
    CHECK_NEAR(samples[0].accel_m_s2[2], -2049.0F * 0.000488F * 9.80665F, 1e-4);
    CHECK_NEAR(samples[1].gyro_rad_s[2], 6.0F * 0.140F * dps_to_rad, 1e-6);
    CHECK_NEAR(samples[1].accel_m_s2[1], 2.0F * 0.000488F * 9.80665F, 1e-6);
    CHECK_NEAR(sensor.latest.temperature_c, 27.0F, 1e-6);
    CHECK(sensor.latest.raw_gyro_x == 4);
    CHECK(sensor.health.fifo_samples == 2U);
    CHECK(sensor.health.samples_missed == 0U);
    AtlasTest_SetSpiTransferHook(NULL);
}

static void test_lsm6_fifo_gaps_saturation_and_splits(void)
{
    AtlasLsm6dsv16b sensor;
    uint8_t bytes[16 * ATLAS_LSM6DSV16B_FIFO_WORD_BYTES];
    AtlasImuSample samples[8];
    size_t words = 0U;
    size_t count;

    init_flight_lsm(&sensor);
    AtlasClockMap_AddPair(&sensor.clock, 0U, 1000000U);

    fifo_timestamp(bytes, &words, 0U, 0U);
    fifo_word(bytes, &words, 0x01U, 0U, 0, 0, 0);
    fifo_word(bytes, &words, 0x02U, 0U, 0, 0, 0);
    /* Two samples lost: the next timestamp is three periods later. */
    fifo_timestamp(bytes, &words, 3U, 3U * TICKS_PER_SAMPLE);
    fifo_word(bytes, &words, 0x01U, 3U, INT16_MAX, 0, 0);
    fifo_word(bytes, &words, 0x02U, 3U, 0, INT16_MIN, 0);
    count = AtlasLsm6dsv16b_DecodeFifo(&sensor, bytes, words, samples, 8U);
    CHECK(count == 2U);
    CHECK(samples[1].missed_before == 2U);
    CHECK((samples[1].flags & ATLAS_IMU_FLAG_GAP_BEFORE) != 0U);
    CHECK((samples[1].flags & ATLAS_IMU_FLAG_GYRO_SATURATED) != 0U);
    CHECK((samples[1].flags & ATLAS_IMU_FLAG_ACCEL_SATURATED) != 0U);
    CHECK(sensor.health.samples_missed == 2U);

    /* A slot split across two bursts completes on the second call. */
    words = 0U;
    fifo_timestamp(bytes, &words, 0U, 4U * TICKS_PER_SAMPLE);
    fifo_word(bytes, &words, 0x01U, 0U, 7, 7, 7);
    CHECK(AtlasLsm6dsv16b_DecodeFifo(&sensor, bytes, words, samples, 8U) == 0U);
    words = 0U;
    fifo_word(bytes, &words, 0x02U, 0U, 9, 9, 9);
    CHECK(AtlasLsm6dsv16b_DecodeFifo(&sensor, bytes, words, samples, 8U) == 1U);
    CHECK(samples[0].missed_before == 0U);
    CHECK(samples[0].timestamp_us == 1000000U + (uint32_t)(4U * 2088U));

    /* Data without any timestamp word is extrapolated and flagged. */
    words = 0U;
    fifo_word(bytes, &words, 0x01U, 1U, 1, 1, 1);
    fifo_word(bytes, &words, 0x02U, 1U, 1, 1, 1);
    fifo_word(bytes, &words, 0x01U, 2U, 2, 2, 2);        /* next slot flushes it */
    count = AtlasLsm6dsv16b_DecodeFifo(&sensor, bytes, words, samples, 8U);
    CHECK(count == 1U);
    CHECK((samples[0].flags & ATLAS_IMU_FLAG_TIMESTAMP_ESTIMATED) != 0U);
    CHECK(samples[0].timestamp_us == 1000000U + (uint32_t)(5U * 2088U));

    /* An overrun reported by FIFO_STATUS marks the next sample as following a gap. */
    test_lsm_registers[0x1BU] = 0x34U;
    test_lsm_registers[0x1CU] = 0x41U;                   /* OVR_IA | DIFF_FIFO[8] */
    {
        uint16_t level = 0U;
        bool overrun = false;
        CHECK(AtlasLsm6dsv16b_ReadFifoStatus(&sensor, &level, &overrun) == ATLAS_OK);
        CHECK(level == 0x134U);
        CHECK(overrun);
        CHECK(sensor.health.fifo_overruns == 1U);
    }
    words = 0U;
    fifo_timestamp(bytes, &words, 3U, 7U * TICKS_PER_SAMPLE);
    fifo_word(bytes, &words, 0x01U, 3U, 0, 0, 0);
    fifo_word(bytes, &words, 0x02U, 3U, 0, 0, 0);
    count = AtlasLsm6dsv16b_DecodeFifo(&sensor, bytes, words, samples, 8U);
    CHECK(count == 2U);                                  /* flushed slot 2 + new one */
    /* The first record emitted after the overrun carries the gap marker. */
    CHECK((samples[0].flags & ATLAS_IMU_FLAG_GAP_BEFORE) != 0U);
    CHECK((samples[0].flags & ATLAS_IMU_FLAG_ACCEL_VALID) == 0U);  /* gyro-only slot */
    CHECK((samples[1].flags & ATLAS_IMU_FLAG_GAP_BEFORE) == 0U);

    /* Insufficient caller capacity is counted, never overrun. */
    words = 0U;
    fifo_timestamp(bytes, &words, 0U, 8U * TICKS_PER_SAMPLE);
    fifo_word(bytes, &words, 0x01U, 0U, 0, 0, 0);
    fifo_word(bytes, &words, 0x02U, 0U, 0, 0, 0);
    CHECK(AtlasLsm6dsv16b_DecodeFifo(&sensor, bytes, words, samples, 0U) == 0U);
    CHECK(sensor.health.output_overflows == 1U);
    AtlasTest_SetSpiTransferHook(NULL);
}

static void test_lsm6_fifo_transport(void)
{
    AtlasLsm6dsv16b sensor;
    uint8_t read_back[20 * ATLAS_LSM6DSV16B_FIFO_WORD_BYTES];
    size_t index;

    init_flight_lsm(&sensor);

    /* Clock pair: TIMESTAMP0..3 read between two microsecond samples. */
    test_use_microseconds(4000000U);
    test_lsm_registers[0x40U] = 0x78U;
    test_lsm_registers[0x41U] = 0x56U;
    test_lsm_registers[0x42U] = 0x34U;
    test_lsm_registers[0x43U] = 0x12U;
    CHECK(AtlasLsm6dsv16b_SyncClock(&sensor) == ATLAS_OK);
    CHECK(sensor.clock.anchored);
    CHECK(sensor.clock.anchor_ticks == 0x12345678UL);
    CHECK(sensor.clock.anchor_us == 4000000U);

    /* 20 words need three polled chunks of at most eight words. */
    for (index = 0U; index < sizeof(read_back); ++index)
    {
        test_fifo[index] = (uint8_t)(index * 7U + 1U);
    }
    test_fifo_bytes = sizeof(read_back);
    test_fifo_offset = 0U;
    test_fifo_transactions = 0U;
    CHECK(AtlasLsm6dsv16b_ReadFifoWords(&sensor, read_back, 20U) == ATLAS_OK);
    CHECK(test_fifo_transactions == 3U);
    CHECK(memcmp(read_back, test_fifo, sizeof(read_back)) == 0);
    CHECK(AtlasLsm6dsv16b_ReadFifoWords(&sensor, read_back,
                                        ATLAS_LSM6DSV16B_FIFO_MAX_WORDS + 1U) ==
          ATLAS_ERROR_ARGUMENT);
    AtlasTest_SetSpiTransferHook(NULL);
}

/* ------------------------------------------------------------------ MMC5983MA */

static void test_mmc_split_phase(void)
{
    AtlasMmc5983ma sensor;
    AtlasMmc5983maField field;
    bool complete = true;

    memset(&test_mmc_spi, 0, sizeof(test_mmc_spi));
    memset(test_mmc_registers, 0, sizeof(test_mmc_registers));
    test_mmc_registers[0x2FU] = ATLAS_MMC5983MA_PRODUCT_ID;
    AtlasTest_SetSpiTransferHook(test_spi_transfer);
    CHECK(AtlasMmc5983ma_Init(&sensor, &test_mmc_spi, &atlas_test_gpio_g, GPIO_PIN_5,
                              ATLAS_MMC5983_BW_200_HZ_4_MS) == ATLAS_OK);
    test_use_microseconds(1000U);

    CHECK(AtlasMmc5983ma_PollMeasurement(&sensor, &field, 10U, &complete) ==
          ATLAS_ERROR_STATE);
    test_mmc_done_on_trigger = true;                    /* stale-looking flag at once */
    memset(&test_mmc_registers[0x00U], 0x80U, 6U);       /* near mid-scale field */
    CHECK(AtlasMmc5983ma_StartMeasurement(&sensor) == ATLAS_OK);
    CHECK(AtlasMmc5983ma_StartMeasurement(&sensor) == ATLAS_ERROR_STATE);

    /* Before the 4 ms conversion time the flag is not trusted or even read. */
    test_mmc_status_reads = 0U;
    test_tim2.counter = 1000U + 3999U;
    CHECK(AtlasMmc5983ma_PollMeasurement(&sensor, &field, 10U, &complete) == ATLAS_OK);
    CHECK(!complete);
    CHECK(test_mmc_status_reads == 0U);

    test_tim2.counter = 1000U + 4000U;
    CHECK(AtlasMmc5983ma_PollMeasurement(&sensor, &field, 10U, &complete) == ATLAS_OK);
    CHECK(complete);
    CHECK(!sensor.measurement_pending);
    CHECK(field.timestamp_us == 1000U + 2000U);          /* conversion midpoint */
    CHECK(sensor.health.measurements == 1U);

    /* A conversion that never completes times out and clears the pending state. */
    test_mmc_done_on_trigger = false;
    CHECK(AtlasMmc5983ma_StartMeasurement(&sensor) == ATLAS_OK);
    test_tim2.counter += 5000U;
    CHECK(AtlasMmc5983ma_PollMeasurement(&sensor, &field, 10U, &complete) == ATLAS_OK);
    CHECK(!complete);
    HAL_Delay(10U);
    CHECK(AtlasMmc5983ma_PollMeasurement(&sensor, &field, 10U, &complete) ==
          ATLAS_ERROR_TIMEOUT);
    CHECK(!sensor.measurement_pending);
    CHECK(sensor.health.measurement_timeouts == 1U);
    AtlasTest_SetSpiTransferHook(NULL);
}

/* ------------------------------------------------------------------ MS5611 */

static I2C_HandleTypeDef test_baro_i2c;
static uint16_t test_baro_prom[8] = {0x1234U, 40127U, 36924U, 23317U,
                                     23282U, 33464U, 28312U, 6U};
static uint8_t test_baro_command;
static uint8_t test_baro_conversion;
static bool test_baro_return_zero;
static unsigned test_baro_adc_reads;

static HAL_StatusTypeDef test_baro_transmit(I2C_HandleTypeDef *i2c, uint16_t address,
                                            uint8_t *data, uint16_t length,
                                            uint32_t timeout_ms)
{
    (void)timeout_ms;
    if ((i2c != &test_baro_i2c) || (address != (0x77U << 1)) || (length != 1U))
    {
        return HAL_ERROR;
    }
    test_baro_command = data[0];
    if ((data[0] & 0xF0U) == 0x40U) { test_baro_conversion = 1U; }
    if ((data[0] & 0xF0U) == 0x50U) { test_baro_conversion = 2U; }
    return HAL_OK;
}

static HAL_StatusTypeDef test_baro_receive(I2C_HandleTypeDef *i2c, uint16_t address,
                                           uint8_t *data, uint16_t length,
                                           uint32_t timeout_ms)
{
    (void)timeout_ms;
    if ((i2c != &test_baro_i2c) || (address != (0x77U << 1)))
    {
        return HAL_ERROR;
    }
    if ((test_baro_command >= 0xA0U) && (test_baro_command <= 0xAEU) && (length == 2U))
    {
        const uint16_t word = test_baro_prom[(test_baro_command - 0xA0U) / 2U];
        data[0] = (uint8_t)(word >> 8);
        data[1] = (uint8_t)word;
        return HAL_OK;
    }
    if ((test_baro_command == 0x00U) && (length == 3U))
    {
        const uint32_t raw = test_baro_return_zero ? 0UL :
                             ((test_baro_conversion == 1U) ? 9085466UL : 8569150UL);
        ++test_baro_adc_reads;
        data[0] = (uint8_t)(raw >> 16);
        data[1] = (uint8_t)(raw >> 8);
        data[2] = (uint8_t)raw;
        return HAL_OK;
    }
    return HAL_ERROR;
}

static void test_ms5611_split_phase(void)
{
    AtlasMs5611 sensor;
    AtlasMs5611Sample sample;
    bool complete = true;

    memset(&test_baro_i2c, 0, sizeof(test_baro_i2c));
    AtlasTest_SetI2cHooks(test_baro_transmit, test_baro_receive);
    CHECK(AtlasMs5611_Init(&sensor, &test_baro_i2c) == ATLAS_OK);
    test_use_microseconds(0U);

    CHECK(AtlasMs5611_PollSample(&sensor, &sample, &complete) == ATLAS_ERROR_STATE);
    CHECK(AtlasMs5611_StartSample(&sensor, ATLAS_MS5611_OSR_1024) == ATLAS_OK);
    CHECK(test_baro_command == 0x54U);                   /* D2 at OSR 1024 */
    CHECK(AtlasMs5611_StartSample(&sensor, ATLAS_MS5611_OSR_1024) == ATLAS_ERROR_STATE);

    /* No ADC read before the rounded-up 3 ms conversion time. */
    test_baro_adc_reads = 0U;
    test_tim2.counter = 2999U;
    CHECK(AtlasMs5611_PollSample(&sensor, &sample, &complete) == ATLAS_OK);
    CHECK(!complete && (test_baro_adc_reads == 0U));
    test_tim2.counter = 3000U;
    CHECK(AtlasMs5611_PollSample(&sensor, &sample, &complete) == ATLAS_OK);
    CHECK(!complete && (test_baro_adc_reads == 1U));
    CHECK(test_baro_command == 0x44U);                   /* D1 started */
    CHECK(sensor.phase == ATLAS_MS5611_PHASE_PRESSURE);
    test_tim2.counter = 6000U;
    CHECK(AtlasMs5611_PollSample(&sensor, &sample, &complete) == ATLAS_OK);
    CHECK(complete);
    CHECK(sample.temperature_centi_c == 2007);
    CHECK(sample.pressure_pa == 100009);
    CHECK(sample.timestamp_us == 3000U + 1500U);
    CHECK(sensor.phase == ATLAS_MS5611_PHASE_IDLE);

    /* A zero ADC result (read without a finished conversion) is rejected. */
    CHECK(AtlasMs5611_StartSample(&sensor, ATLAS_MS5611_OSR_1024) == ATLAS_OK);
    test_baro_return_zero = true;
    test_tim2.counter = 9000U;
    CHECK(AtlasMs5611_PollSample(&sensor, &sample, &complete) == ATLAS_ERROR_PROTOCOL);
    CHECK(sensor.phase == ATLAS_MS5611_PHASE_IDLE);
    test_baro_return_zero = false;
    AtlasTest_SetI2cHooks(NULL, NULL);
}

int main(void)
{
    test_ring_order_and_overflow();
    test_ring_index_wrap();
    test_clock_map_nominal_and_calibration();
    test_clock_map_resync_and_wrap();
    test_lsm6_fifo_flight_registers();
    test_lsm6_fifo_decode_orders();
    test_lsm6_fifo_gaps_saturation_and_splits();
    test_lsm6_fifo_transport();
    test_mmc_split_phase();
    test_ms5611_split_phase();
    puts("All Atlas sample-path tests passed.");
    return 0;
}
