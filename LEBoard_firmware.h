#ifndef _AUTOMATION_IO_H_
#define _AUTOMATION_IO_H_

#include "bleprofile.h"

#define AIO_HANDLE_NUM_MAX     5 //non notification/indication handle
#define AIO_NOT_HANDLE_NUM_MAX 5 //notification/indication handle

enum aio_trigger
{
    AIO_NO_OPERATION            = 0x00,
    AIO_NO_IND_TIME             = 0x01,
    AIO_NO_IND_INTERVAL         = 0x02,
    AIO_IND_CHANGED             = 0x03,
    AIO_LESS_THAN               = 0x04,
    AIO_LESS_THAN_EQUAL         = 0x05,
    AIO_GREATER_THAN            = 0x06,
    AIO_GREATER_THAN_EQUAL      = 0x07,
    AIO_EQUAL                   = 0x08,
    AIO_NOT_EQUAL               = 0x09,
    AIO_CHANGED_MORE_THAN       = 0x0a,
    AIO_CHANGED_MORE_OFTEN_THAN = 0x0b,
    AIO_MASK                    = 0x0c,
};

typedef PACKED struct
{
    UINT16 hdl[AIO_HANDLE_NUM_MAX];   // GATT HANDLE number
    UINT16 serv[AIO_HANDLE_NUM_MAX];  // GATT service UUID
    UINT16 cha[AIO_HANDLE_NUM_MAX];   // GATT characteristic UUID
} BLE_AIO_GATT_CFG;

typedef PACKED struct
{
    UINT32 tick   [AIO_NOT_HANDLE_NUM_MAX];
    UINT32 timeout[AIO_NOT_HANDLE_NUM_MAX];
    UINT16 count  [AIO_NOT_HANDLE_NUM_MAX];
    UINT16 hdl    [AIO_NOT_HANDLE_NUM_MAX]; //GATT HANDLE number
    UINT16 cl_hdl [AIO_NOT_HANDLE_NUM_MAX]; //GATT HANDLE number
    UINT16 tr_hdl [AIO_NOT_HANDLE_NUM_MAX]; //GATT HANDLE number
    UINT16 tr2_hdl[AIO_NOT_HANDLE_NUM_MAX]; //GATT HANDLE number
    UINT16 serv   [AIO_NOT_HANDLE_NUM_MAX]; //GATT service UUID
    UINT16 cha    [AIO_NOT_HANDLE_NUM_MAX]; //GATT characteristic UUID
} BLE_AIO_NOT_GATT_CFG;

//host information for NVRAM
typedef PACKED struct
//typedef struct
{
    // BD address of the bonded host
    BD_ADDR bdAddr;
    UINT16  serv[AIO_NOT_HANDLE_NUM_MAX];
    UINT16  cha [AIO_NOT_HANDLE_NUM_MAX];
    UINT16  cli_cha_desc[AIO_NOT_HANDLE_NUM_MAX];
}  BLEAIO_HOSTINFO;


void bleaio_Create(void);

extern const UINT8                 bleaio_db_data[];
extern const UINT16                bleaio_db_size;
extern const BLE_PROFILE_CFG       bleaio_cfg;
extern const BLE_PROFILE_PUART_CFG bleaio_puart_cfg;
extern const BLE_PROFILE_GPIO_CFG  bleaio_gpio_cfg;

#endif // end of #ifndef _BLEAIO_H_
