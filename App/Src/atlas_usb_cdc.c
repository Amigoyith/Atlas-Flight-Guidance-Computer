/**
 * @file atlas_usb_cdc.c
 * @brief Atlas CDC class adapter: validated control requests and retained buffers.
 *
 * Major functions:
 * - AtlasUsbDevice_Init(): initializes the library but DOES NOT connect the pull-up.
 * - AtlasUsbCdc_Init()/DeInit(): reset session and buffer state.
 * - AtlasUsbCdc_Control(): validated seven-byte line coding and DTR/RTS.
 * - AtlasUsbCdc_Receive(): retain RX before rearming the endpoint.
 * - AtlasUsbCdc_Transmit(): copy TX into private storage until the completion callback.
 *
 * The AtlasUsb task owns start/stop and gates connection using the PA9 VBUS divider.
 */
#include "atlas_usb_cdc.h"
#include "atlas_usb.h"
#include "main.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include <string.h>

/* Fails if the stock usbd_def.h shadows the patched copy in App/CubeOverrides. */
_Static_assert(sizeof(((USBD_EndpointTypeDef *)0)->status) == 2U,
               "patched usbd_def.h (16-bit endpoint status) is not on the include path");

extern USBD_HandleTypeDef hUsbDeviceFS;
static uint8_t rx_buffer[CDC_DATA_FS_MAX_PACKET_SIZE];
static uint8_t tx_buffer[ATLAS_USB_PACKET_CAPACITY];
static uint8_t line_coding[7] = {0x00, 0xC2, 0x01, 0x00, 0, 0, 8}; /* 115200 8N1. */

/** @brief Prepare CDC while disconnected; optional USB failure is nonfatal.
 * @return USB status; caller must tear down on error and separately gate USBD_Start. */
uint8_t AtlasUsbDevice_Init(void)
{
    HAL_PWREx_EnableUSBVoltageDetector();
    if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK ||
        USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK ||
        USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK)
        return USBD_FAIL;
    return USBD_OK;
}

/** @brief Allocate no memory and initialize the class's retained buffers. @return USB result. */
int8_t AtlasUsbCdc_Init(void)
{
    if (USBD_CDC_SetTxBuffer(&hUsbDeviceFS, tx_buffer, 0U) != USBD_OK ||
        USBD_CDC_SetRxBuffer(&hUsbDeviceFS, rx_buffer) != USBD_OK) return USBD_FAIL;
    AtlasUsb_OnClassState(true);
    return USBD_OK;
}

/** @brief Invalidate all old-session work. @return USB result. */
int8_t AtlasUsbCdc_DeInit(void)
{
    AtlasUsb_OnClassState(false);
    return USBD_OK;
}

/** @brief Validate CDC requests; baud is metadata, never another UART's setting.
 * @param cmd Request. @param data Payload or setup structure for length zero.
 * @param length Payload size. @return USBD_FAIL requests an EP0 stall. */
int8_t AtlasUsbCdc_Control(uint8_t cmd, uint8_t *data, uint16_t length)
{
    if (data == NULL) return USBD_FAIL;
    switch (cmd)
    {
        case CDC_SET_LINE_CODING:
            if (length != 7U || data[4] > 2U || data[5] > 4U ||
                !((data[6] >= 5U && data[6] <= 8U) || data[6] == 16U)) return USBD_FAIL;
            memcpy(line_coding, data, sizeof(line_coding));
            return USBD_OK;
        case CDC_GET_LINE_CODING:
            if (length != 7U) return USBD_FAIL;
            memcpy(data, line_coding, sizeof(line_coding));
            return USBD_OK;
        case CDC_SET_CONTROL_LINE_STATE:
            if (length != 0U) return USBD_FAIL;
            AtlasUsb_OnControlLines(((USBD_SetupReqTypedef *)data)->wValue);
            return USBD_OK;
        case CDC_SEND_BREAK: /* Accepted as metadata; it does not drive a physical pin. */
            return length == 0U ? USBD_OK : USBD_FAIL;
        default: return USBD_FAIL;
    }
}

/** @brief Retain an OUT packet before allowing another. @param data Received bytes.
 * @param length Count pointer. @return USB result. */
int8_t AtlasUsbCdc_Receive(uint8_t *data, uint32_t *length)
{
    if (data == NULL || length == NULL || *length > sizeof(rx_buffer)) return USBD_FAIL;
    AtlasUsb_OnReceive(data, *length);
    if (USBD_CDC_SetRxBuffer(&hUsbDeviceFS, rx_buffer) != USBD_OK) return USBD_FAIL;
    return (int8_t)USBD_CDC_ReceivePacket(&hUsbDeviceFS);
}

/** @brief Safely query current class TX state. @return true only while configured and busy. */
bool AtlasUsbCdc_TransmitBusy(void)
{
    const USBD_CDC_HandleTypeDef *cdc = hUsbDeviceFS.pClassDataCmsit[hUsbDeviceFS.classId];
    return hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && cdc != NULL && cdc->TxState != 0U;
}

/** @brief Copy and start a TX; caller bytes can be reused on return.
 * @param data Source. @param length 1-64 bytes. @return USB status.
 * @note The AtlasUsb owner calls this; endpoint lifetime is held in tx_buffer. */
uint8_t AtlasUsbCdc_Transmit(const uint8_t *data, uint16_t length)
{
    const uint32_t mask = __get_PRIMASK();
    uint8_t result = USBD_FAIL;
    __disable_irq();
    USBD_CDC_HandleTypeDef *cdc = hUsbDeviceFS.pClassDataCmsit[hUsbDeviceFS.classId];
    if (data != NULL && length != 0U && length <= sizeof(tx_buffer) &&
        hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && cdc != NULL)
    {
        if (cdc->TxState != 0U) result = USBD_BUSY;
        else
        {
            memcpy(tx_buffer, data, length);
            result = USBD_CDC_SetTxBuffer(&hUsbDeviceFS, tx_buffer, length);
            if (result == USBD_OK) result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
        }
    }
    __DMB();
    __set_PRIMASK(mask);
    return result;
}

/** @brief Record completion after the final data/ZLP handshake. @param data Retained buffer.
 * @param length Completed bytes. @param ep Endpoint. @return USB result. */
int8_t AtlasUsbCdc_TransmitComplete(uint8_t *data, uint32_t *length, uint8_t ep)
{
    (void)data; (void)ep;
    if (length != NULL) AtlasUsb_OnTransmit(*length);
    return USBD_OK;
}
