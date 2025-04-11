#ifndef UBX_M8_MSG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define UBX_HEADER_0 0xB5
#define UBX_HEADER_1 0x62

#define UBX_HNR_CLASS 0x28
#define UBX_HNR_PVT 0x00

#define UBX_NAV_CLASS 0x01
#define UBX_NAV_PVT 0x07

#define UBX_CFG_CLASS 0x06
#define UBX_CFG_PRT 0x00
#define UBX_CFG_HNR 0x5C

#define UBX_ACK_CLASS 0x05
#define UBX_ACK_ACK 0x01

#pragma pack(push, 1)
typedef struct
{
  uint32_t iTOW;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t valid;
  int32_t nano;
  uint8_t gnssFix;
  uint8_t flags;
  uint8_t reserved1[2];
  int32_t lon;
  int32_t lat;
  int32_t height;
  int32_t hMSL;
  int32_t gspeed;
  int32_t speed;
  int32_t headMot;
  int32_t headVeh;
  uint32_t hAcc;
  uint32_t vAcc;
  uint32_t sAcc;
  uint32_t headAcc;
  uint8_t reserved2[4];
} ubx_hnr_pvt_t;

typedef struct
{
  uint32_t iTOW;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t valid;
  uint32_t tAcc;
  int32_t nano;
  uint8_t gnssFix;
  uint8_t flags;
  uint8_t flags2;
  uint8_t numSV;
  int32_t lon;
  int32_t lat;
  int32_t height;
  int32_t hMSL;
  uint32_t hAcc;
  uint32_t vAcc;
  int32_t velN;
  int32_t velE;
  int32_t velD;
  int32_t gSpeed;
  int32_t headMot;
  uint32_t sAcc;
  uint32_t headAcc;
  uint16_t pDOP;
  uint16_t flags3;
  uint8_t reserved1[4];
  int32_t headVeh;
  int16_t magDec;
  uint16_t magAcc;
} ubx_nav_pvt_t;

typedef struct
{
  uint8_t portID;
  uint8_t reserved1;
  uint16_t txReady;
  uint32_t mode;
  uint32_t baudRate;
  uint16_t inProtoMask;
  uint16_t outProtoMask;
  uint16_t flags;
  uint8_t reserved2[2];
} ubx_cfg_prt_t;

typedef struct
{
  uint8_t class_id;
  uint8_t msg_id;
} ubx_ack_ack_t;
#pragma pack(pop)

typedef struct 
{
  uint8_t highNavRate;
  uint8_t reserved1[3];
} ubx_cfg_hnr_t;


#ifdef __cplusplus
}
#endif

#endif // UBX_M8_MSG_H