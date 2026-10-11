/**
 * @file atlas_usb_cdc.h
 * @brief Atlas CDC class adapter behind the CubeMX-generated usbd_cdc_if.c hooks.
 *
 * Major functions:
 * - AtlasUsbDevice_Init(): prepares the library and CDC class without connecting.
 * - AtlasUsbCdc_Init()/DeInit(): reset session and buffer state.
 * - AtlasUsbCdc_Control(): validated seven-byte line coding and DTR/RTS.
 * - AtlasUsbCdc_Receive(): retain RX before rearming the endpoint.
 * - AtlasUsbCdc_Transmit()/TransmitBusy(): copied TX until the completion callback.
 *
 * The generated usbd_cdc_if.c only forwards each CDC callback here from its USER
 * CODE blocks, so CubeMX can regenerate it freely.
 */
#ifndef ATLAS_USB_CDC_H
#define ATLAS_USB_CDC_H
#include <stdbool.h>
#include <stdint.h>

/** @brief Prepare CDC while disconnected; optional USB failure is nonfatal.
 * @return USB status; caller must tear down on error and separately gate USBD_Start. */
uint8_t AtlasUsbDevice_Init(void);
/** @brief Initialize the class's retained buffers. @return USB result. */
int8_t AtlasUsbCdc_Init(void);
/** @brief Invalidate all old-session work. @return USB result. */
int8_t AtlasUsbCdc_DeInit(void);
/** @brief Validate a CDC request; baud is metadata, never another UART's setting.
 * @param cmd Request. @param data Payload or setup structure for length zero.
 * @param length Payload size. @return USBD_FAIL requests an EP0 stall. */
int8_t AtlasUsbCdc_Control(uint8_t cmd, uint8_t *data, uint16_t length);
/** @brief Retain an OUT packet before allowing another. @param data Received bytes.
 * @param length Count pointer. @return USB result. */
int8_t AtlasUsbCdc_Receive(uint8_t *data, uint32_t *length);
/** @brief Copy and start a TX; caller bytes can be reused on return.
 * @param data Source. @param length 1-64 bytes. @return USB status.
 * @note The AtlasUsb owner calls this; endpoint lifetime is held in the adapter. */
uint8_t AtlasUsbCdc_Transmit(const uint8_t *data, uint16_t length);
/** @brief Read configured TX-busy state; USB owner only. @return Busy indication. */
bool AtlasUsbCdc_TransmitBusy(void);
/** @brief Record completion after the final data/ZLP handshake. @param data Retained buffer.
 * @param length Completed bytes. @param ep Endpoint. @return USB result. */
int8_t AtlasUsbCdc_TransmitComplete(uint8_t *data, uint32_t *length, uint8_t ep);
#endif
