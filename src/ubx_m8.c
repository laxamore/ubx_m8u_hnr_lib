#include <stdio.h>
#include <string.h>
#include <time.h>
#include <stdbool.h>
#include <stdlib.h>

#include "ubx_m8.h"
#include "ubx_m8_msg.h"

#define UBX_LOCK_MUTEX() \
  if (s_mutex_lock)      \
  s_mutex_lock()
#define UBX_UNLOCK_MUTEX() \
  if (s_mutex_unlock)      \
  s_mutex_unlock()
#define htons(x) ((uint16_t)(((x) << 8) | ((x) >> 8)))
#define DEFAULT_TIMEOUT 1000

static bool s_is_initialized = false;
static uint8_t *sp_buffer = NULL;
static ubx_m8_gnss_t *sp_ubx_gnss = NULL;
static ubx_if_t *sp_ubx_if = NULL;
static mutex_lock_fn s_mutex_lock = NULL;
static mutex_unlock_fn s_mutex_unlock = NULL;
static ubx_m8_port_t s_ubx_m8_port = UBX_M8_PORT_UART1;

// Message pointers
static ubx_cfg_prt_t *sp_ubx_cfg_prt = NULL;
static ubx_hnr_pvt_t *sp_ubx_hnr_pvt = NULL;
static ubx_nav_pvt_t *sp_ubx_nav_pvt = NULL;
static ubx_ack_ack_t *sp_ubx_ack_ack = NULL;
static ubx_cfg_hnr_t *sp_ubx_cfg_hnr = NULL;

static void print_hex(uint8_t *data, uint32_t data_size)
{
  char *hex_str = calloc(1, sizeof(char) * (data_size * 3 + 1));
  if (hex_str == NULL)
    return; // Memory allocation failed
  for (uint32_t i = 0; i < data_size; i++)
  {
    sprintf(hex_str + i * 3, "%02X ", data[i]);
  }
  hex_str[data_size * 3] = '\0'; // Null-terminate the string
  printf("%s\n", hex_str);
  free(hex_str);
}

static void *ubx_ensure_allocated(void **ptr, uint32_t size)
{
  if (*ptr == NULL)
  {
    *ptr = calloc(1, size);
  }
  else
  {
    memset(*ptr, 0, size);
  }
  return *ptr;
}

static uint16_t calculate_checksum(uint8_t *data, uint32_t data_size)
{
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;
  for (size_t i = 0; i < data_size; i++)
  {
    if (i > 1)
    {
      ck_a += data[i];
      ck_b += ck_a;
    }
  }
  return (ck_a << 8) | ck_b;
}

static uint64_t to_timestamp(uint16_t year, uint8_t month, uint8_t day,
                             uint8_t hour, uint8_t min, uint8_t sec)
{
  struct tm timeinfo = {
      .tm_year = year - 1900,
      .tm_mon = month - 1,
      .tm_mday = day,
      .tm_hour = hour,
      .tm_min = min,
      .tm_sec = sec,
  };

  return mktime(&timeinfo);
}

static int ubx_send_message(uint8_t class_id, uint8_t msg_id, const void *payload, uint16_t payload_size)
{
  int result = -1;

  // Allocate memory for the message
  uint8_t *buf = calloc(1, sizeof(uint8_t) * (2 + 2 + 2 + payload_size + 2));
  if (!buf)
  {
    return -1; // Memory allocation failed
  }

  // Construct message
  memcpy(buf, (uint8_t[]){UBX_HEADER_0, UBX_HEADER_1}, 2);
  buf[2] = class_id;
  buf[3] = msg_id;
  memcpy(buf + 4, &payload_size, 2);

  if (payload && payload_size > 0)
  {
    memcpy(buf + 6, payload, payload_size);
  }

  // Add checksum
  uint16_t crc = htons(calculate_checksum(buf, 6 + payload_size));
  memcpy(buf + 6 + payload_size, &crc, 2);

  // Send message
  result = sp_ubx_if->send(buf, 8 + payload_size);
  free(buf);

  if (result < 0)
  {
    return -1;
  }

  return 0;
}

static int ubx_parse_message(const uint8_t *buffer, int len, uint8_t expected_class, uint8_t expected_id, void *output, uint32_t output_size)
{
  if (len < 8 + output_size) // Header + class & id + length + minimum payload + checksum
    return -1;

  // Check header
  if (memcmp(buffer, (uint8_t[]){UBX_HEADER_0, UBX_HEADER_1}, 2) != 0)
    return -1;

  // Check class and id if specified
  if (expected_class != 0xFF && buffer[2] != expected_class)
    return -1;
  if (expected_id != 0xFF && buffer[3] != expected_id)
    return -1;

  // Extract payload length
  uint16_t payload_len = *(uint16_t *)(buffer + 4);
  if (payload_len != output_size)
    return -1;

  // Verify checksum
  uint16_t crc = htons(calculate_checksum(buffer, 6 + payload_len));
  if (memcmp(buffer + 6 + payload_len, &crc, 2) != 0)
    return -1;

  // Copy payload to output
  if (output)
    memcpy(output, buffer + 6, payload_len);

  return 0;
}

static int ubx_send_and_receive(uint8_t class_id, uint8_t msg_id,
                                const void *payload, uint16_t payload_size,
                                uint8_t expected_resp_class, uint8_t expected_resp_id,
                                void *response, uint16_t response_size)
{
  if (ubx_send_message(class_id, msg_id, payload, payload_size) != 0)
  {
    return -1;
  }

  // Wait for response with timeout
  int total_bytes = 0;
  int time_elapsed = 0;
  const int polling_interval = 10; // ms
  uint8_t *p = sp_buffer;

  while (time_elapsed < DEFAULT_TIMEOUT)
  {
    // Check for available data
    uint8_t buffer_size = (UBX_M8_BUFFER_SIZE - total_bytes) > 128 ? 128 : (UBX_M8_BUFFER_SIZE - total_bytes);
    uint8_t *temp_buffer = calloc(1, buffer_size);
    int bytes_read = sp_ubx_if->recv(temp_buffer, buffer_size);

    if (bytes_read > 0 && buffer_size != 0)
    {
      memcpy(sp_buffer + total_bytes, temp_buffer, bytes_read);
      total_bytes += bytes_read;
    }
    free(temp_buffer);

    uint8_t *ps = memchr(p, UBX_HEADER_0, total_bytes);
    if (ps != NULL)
    {
      // Check if we have enough data for a complete message
      int len = total_bytes - (ps - p);
      if (len >= 8 + response_size)
      {
        // Parse the message
        int result = ubx_parse_message(ps, len, expected_resp_class, expected_resp_id, response, response_size);
        if (result == 0)
        {
          return 0; // Success
        }
      }
    }

    // Shift the buffer
    if (ps != NULL)
    {
      total_bytes -= (ps - p);
      memmove(sp_buffer, ps, total_bytes);
      p = sp_buffer;
    }
    else
    {
      total_bytes = 0;
      p = sp_buffer;
    }

    // Update elapsed time
    time_elapsed += polling_interval;
    sp_ubx_if->delay_ms(polling_interval);
  }

  return -1; // Failed to receive expected response
}

static int ubx_set_cfg_prt(ubx_cfg_prt_t *cfg_prt)
{
  if (cfg_prt == NULL)
    return -1;

  UBX_LOCK_MUTEX();

  if (!ubx_ensure_allocated((void **)&sp_ubx_ack_ack, sizeof(ubx_ack_ack_t)))
  {
    UBX_UNLOCK_MUTEX();
    return -1;
  }

  int result = ubx_send_and_receive(UBX_CFG_CLASS, UBX_CFG_PRT,
                                    cfg_prt, sizeof(ubx_cfg_prt_t),
                                    UBX_ACK_CLASS, UBX_ACK_ACK,
                                    sp_ubx_ack_ack, sizeof(ubx_ack_ack_t));

  if (result == 0)
  {
    // Verify that ACK is for the right message
    if (sp_ubx_ack_ack->class_id != UBX_CFG_CLASS || sp_ubx_ack_ack->msg_id != UBX_CFG_PRT)
    {
      result = -1;
    }
  }

  UBX_UNLOCK_MUTEX();
  return result;
}

static int ubx_get_cfg_hnr(void)
{
  UBX_LOCK_MUTEX();

  if (!ubx_ensure_allocated((void **)&sp_ubx_cfg_hnr, sizeof(ubx_cfg_hnr_t)))
  {
    UBX_UNLOCK_MUTEX();
    return -1;
  }

  int result = ubx_send_and_receive(UBX_CFG_CLASS, UBX_CFG_HNR,
                                    NULL, 0,
                                    UBX_CFG_CLASS, UBX_CFG_HNR,
                                    sp_ubx_cfg_hnr, sizeof(ubx_cfg_hnr_t));

  UBX_UNLOCK_MUTEX();
  return result;
}

static int ubx_get_cfg_prt(void)
{
  UBX_LOCK_MUTEX();

  if (!ubx_ensure_allocated((void **)&sp_ubx_cfg_prt, sizeof(ubx_cfg_prt_t)))
  {
    UBX_UNLOCK_MUTEX();
    return -1;
  }

  int result = ubx_send_and_receive(UBX_CFG_CLASS, UBX_CFG_PRT,
                                    NULL, 0,
                                    UBX_CFG_CLASS, UBX_CFG_PRT,
                                    sp_ubx_cfg_prt, sizeof(ubx_cfg_prt_t));

  UBX_UNLOCK_MUTEX();
  return result;
}

static int ubx_get_hnr_pvt(void)
{
  UBX_LOCK_MUTEX();
  if (!ubx_ensure_allocated((void **)&sp_ubx_hnr_pvt, sizeof(ubx_hnr_pvt_t)))
  {
    UBX_UNLOCK_MUTEX();
    return -1;
  }

  int result = ubx_send_and_receive(UBX_HNR_CLASS, UBX_HNR_PVT,
                                    NULL, 0,
                                    UBX_HNR_CLASS, UBX_HNR_PVT,
                                    sp_ubx_hnr_pvt, sizeof(ubx_hnr_pvt_t));

  if (result == 0)
  {
    // Copy data to GNSS structure
    sp_ubx_gnss->fix_quality = sp_ubx_hnr_pvt->gnssFix;
    sp_ubx_gnss->longitude = sp_ubx_hnr_pvt->lon;
    sp_ubx_gnss->latitude = sp_ubx_hnr_pvt->lat;
    sp_ubx_gnss->altitude = sp_ubx_hnr_pvt->hMSL;         // In millimeters
    sp_ubx_gnss->speed = sp_ubx_hnr_pvt->gspeed / 1000;   // Convert to m/s
    sp_ubx_gnss->heading = sp_ubx_hnr_pvt->headVeh / 1e4; // In tenths of degrees
    sp_ubx_gnss->timestamp = to_timestamp(sp_ubx_hnr_pvt->year, sp_ubx_hnr_pvt->month,
                                          sp_ubx_hnr_pvt->day, sp_ubx_hnr_pvt->hour,
                                          sp_ubx_hnr_pvt->min, sp_ubx_hnr_pvt->sec);
  }

  UBX_UNLOCK_MUTEX();
  return result;
}

static int ubx_get_nav_pvt(void)
{
  UBX_LOCK_MUTEX();
  if (!ubx_ensure_allocated((void **)&sp_ubx_nav_pvt, sizeof(ubx_nav_pvt_t)))
  {
    UBX_UNLOCK_MUTEX();
    return -1;
  }

  int result = ubx_send_and_receive(UBX_NAV_CLASS, UBX_NAV_PVT,
                                    NULL, 0,
                                    UBX_NAV_CLASS, UBX_NAV_PVT,
                                    sp_ubx_nav_pvt, sizeof(ubx_nav_pvt_t));

  if (result == 0)
  {
    sp_ubx_gnss->fix_quality = sp_ubx_nav_pvt->gnssFix;
    sp_ubx_gnss->longitude = sp_ubx_nav_pvt->lon;
    sp_ubx_gnss->latitude = sp_ubx_nav_pvt->lat;
    sp_ubx_gnss->altitude = sp_ubx_nav_pvt->hMSL;         // In millimeters
    sp_ubx_gnss->speed = sp_ubx_nav_pvt->gSpeed / 1000;   // Convert to m/s
    sp_ubx_gnss->heading = sp_ubx_nav_pvt->headVeh / 1e4; // In tenths of degrees
    sp_ubx_gnss->num_satellites = sp_ubx_nav_pvt->numSV;
    sp_ubx_gnss->timestamp = to_timestamp(sp_ubx_nav_pvt->year, sp_ubx_nav_pvt->month,
                                          sp_ubx_nav_pvt->day, sp_ubx_nav_pvt->hour,
                                          sp_ubx_nav_pvt->min, sp_ubx_nav_pvt->sec);
  }
  UBX_UNLOCK_MUTEX();

  return result;
}

int ubx_m8_init(ubx_m8_port_t ubx_m8_port, ubx_if_t ubx_if)
{
  return ubx_m8_init_with_mutex(ubx_m8_port, ubx_if, NULL, NULL);
}

int ubx_m8_init_with_mutex(ubx_m8_port_t ubx_m8_port, ubx_if_t ubx_if, mutex_lock_fn lock, mutex_unlock_fn unlock)
{
  // Verify mutex consistency
  if (s_mutex_lock != NULL && s_mutex_unlock != NULL &&
      (s_mutex_lock != lock || s_mutex_unlock != unlock))
    return -1;

  // Both must be set or both must be NULL
  if ((lock != NULL) != (unlock != NULL))
    return -1;

  if (lock != NULL && unlock != NULL)
  {
    s_mutex_lock = lock;
    s_mutex_unlock = unlock;
  }

  UBX_LOCK_MUTEX();

  if (s_is_initialized)
  {
    UBX_UNLOCK_MUTEX();
    return 0; // Already initialized
  }

  s_ubx_m8_port = ubx_m8_port;

  // Allocate required memory structures
  if (!ubx_ensure_allocated((void **)&sp_buffer, UBX_M8_BUFFER_SIZE) ||
      !ubx_ensure_allocated((void **)&sp_ubx_gnss, sizeof(ubx_m8_gnss_t)) ||
      !ubx_ensure_allocated((void **)&sp_ubx_if, sizeof(ubx_if_t)))
  {
    UBX_UNLOCK_MUTEX();
    ubx_m8_deinit();
    return -1;
  }

  // Copy interface functions
  memcpy(sp_ubx_if, &ubx_if, sizeof(ubx_if_t));

  // Get initial configuration
  if (ubx_get_cfg_prt() != 0)
  {
    UBX_UNLOCK_MUTEX();
    ubx_m8_deinit();
    return -1;
  }

  // Get HNR configuration
  if (ubx_get_cfg_hnr() != 0)
  {
    UBX_UNLOCK_MUTEX();
    ubx_m8_deinit();
    return -1;
  }

  s_is_initialized = true;
  UBX_UNLOCK_MUTEX();
  return 0;
}

int ubx_m8_deinit()
{
  if (!s_is_initialized)
    return -1; // Not initialized

  UBX_LOCK_MUTEX();

  if (sp_buffer != NULL)
  {
    free(sp_buffer);
    sp_buffer = NULL;
  }

  if (sp_ubx_gnss != NULL)
  {
    free(sp_ubx_gnss);
    sp_ubx_gnss = NULL;
  }

  if (sp_ubx_if != NULL)
  {
    free(sp_ubx_if);
    sp_ubx_if = NULL;
  }

  if (s_mutex_lock != NULL)
  {
    s_mutex_lock = NULL;
  }

  if (sp_ubx_cfg_prt != NULL)
  {
    free(sp_ubx_cfg_prt);
    sp_ubx_cfg_prt = NULL;
  }

  if (sp_ubx_hnr_pvt != NULL)
  {
    free(sp_ubx_hnr_pvt);
    sp_ubx_hnr_pvt = NULL;
  }

  if (sp_ubx_nav_pvt != NULL)
  {
    free(sp_ubx_nav_pvt);
    sp_ubx_nav_pvt = NULL;
  }

  s_ubx_m8_port = UBX_M8_PORT_UART1;

  UBX_UNLOCK_MUTEX();

  if (s_mutex_unlock != NULL)
  {
    s_mutex_unlock = NULL;
  }

  s_is_initialized = false;
  return 0; // Success
}

ubx_m8_gnss_t *ubx_m8_get_gnss_data()
{
  if (!s_is_initialized)
    return NULL; // Not initialized

  // ubx_get_nav_pvt();
  ubx_get_hnr_pvt();

  return sp_ubx_gnss;
}

int ubx_m8_disable_nmea_output(void)
{
  if (!s_is_initialized)
    return -1; // Not initialized

  sp_ubx_cfg_prt->outProtoMask = 0x01; // Disable NMEA output, only UBX output

  return ubx_set_cfg_prt(sp_ubx_cfg_prt);
}

int ubx_m8_set_hnr_rate(uint8_t rate)
{
  if (!s_is_initialized)
    return -1; // Not initialized

  if (rate < 1 || rate > 30)
    return -1; // Invalid rate

  UBX_LOCK_MUTEX();

  sp_ubx_cfg_hnr->highNavRate = rate;

  int result = ubx_send_and_receive(UBX_CFG_CLASS, UBX_CFG_HNR,
                                    sp_ubx_cfg_hnr, sizeof(ubx_cfg_hnr_t),
                                    UBX_ACK_CLASS, UBX_ACK_ACK,
                                    sp_ubx_ack_ack, sizeof(ubx_ack_ack_t));

  UBX_UNLOCK_MUTEX();

  return result;
}