#include "ublox_m8u_hnr.h"
#include <string.h>
#include <time.h>
#include <stdlib.h>
#include <stdint.h>

// UBX protocol structures
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
  int32_t longitude;
  int32_t latitude;
  int32_t height;
  int32_t hMSL;
  int32_t gspeed;
  int32_t speed;
  int32_t headMOt;
  int32_t headVeh;
  uint32_t hAcc;
  uint32_t vAcc;
  uint32_t sAcc;
  uint32_t headAcc;
} ubx_hnr_pvt;

#define UBLOX_RECEIVE_BUFFER_SIZE 256
#define RING_BUFFER_SIZE 800

// Static variables
static ublox_m8u_gnss *ugnss_s;
static uart_interface uart_interface_s;
static uint8_t ring_buffer[RING_BUFFER_SIZE];
static size_t ring_buffer_len = 0;
static int is_initialized = 0;

// Forward declarations
static uint16_t calculate_checksum(uint8_t *data, size_t data_size);
static uint64_t to_timestamp(uint16_t year, uint8_t month, uint8_t day,
                             uint8_t hour, uint8_t min, uint8_t sec);
static void parse_ubx_hnr_pvt();
static int process_received_data();
static int send_request_message();
static int disable_nmea_output();

/**
 * @brief Calculate UBX checksum
 */
static uint16_t calculate_checksum(uint8_t *data, size_t data_size)
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

/**
 * @brief Convert date and time to timestamp
 */
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

/**
 * @brief Parse UBX HNR PVT message from the ring buffer
 */
static void parse_ubx_hnr_pvt()
{
  // Search for a complete UBX HNR PVT message in the ring buffer
  for (size_t i = 0; i <= ring_buffer_len - 80; i++)
  {
    if (ring_buffer[i] == 0xB5 && ring_buffer[i + 1] == 0x62 &&
        ring_buffer[i + 2] == 0x28 && ring_buffer[i + 3] == 0x00 &&
        ring_buffer[i + 4] == 0x48 && ring_buffer[i + 5] == 0x00)
    {
      // Validate checksum
      size_t message_len = 80;
      uint16_t expected_crc = calculate_checksum(ring_buffer + i, message_len - 2);
      uint16_t actual_crc = (ring_buffer[i + 78] << 8) | ring_buffer[i + 79];

      if (expected_crc != actual_crc)
      {
        // Log: "Checksum mismatch for HNR PVT message"
        continue;
      }

      // Extract GNSS data
      ubx_hnr_pvt pvt_data;
      memcpy(&pvt_data, &ring_buffer[i + 6], sizeof(ubx_hnr_pvt));

      if (uart_interface_s.lock != NULL)
      {
        uart_interface_s.lock();
      }

      ugnss_s->fix_quality = pvt_data.gnssFix;

      // Update global GNSS data
      if (ugnss_s->fix_quality == TIME_ONLY_FIX)
      {
        ugnss_s->timestamp =
            to_timestamp(pvt_data.year, pvt_data.month, pvt_data.day,
                         pvt_data.hour, pvt_data.min, pvt_data.sec);
      }
      else if (ugnss_s->fix_quality >= 1)
      {
        ugnss_s->longitude = pvt_data.longitude;
        ugnss_s->latitude = pvt_data.latitude;
        ugnss_s->altitude = pvt_data.hMSL;         // In millimeters
        ugnss_s->speed = pvt_data.gspeed / 1000;   // Convert to m/s
        ugnss_s->heading = pvt_data.headVeh / 1e4; // In tenths of degrees
        ugnss_s->num_satellites = 0;               // This info is not in HNR PVT message
        ugnss_s->timestamp =
            to_timestamp(pvt_data.year, pvt_data.month, pvt_data.day,
                         pvt_data.hour, pvt_data.min, pvt_data.sec);
      }

      if (uart_interface_s.unlock != NULL)
      {
        uart_interface_s.unlock();
      }

      // Remove the processed message from the rolling buffer
      size_t remaining_bytes = ring_buffer_len - i - message_len;
      if (remaining_bytes > 0)
      {
        memmove(ring_buffer + i, ring_buffer + i + message_len, remaining_bytes);
      }
      ring_buffer_len -= message_len;
    }
  }
}

/**
 * @brief Process received data from UART
 */
static int process_received_data()
{
  uint8_t receive_buffer[UBLOX_RECEIVE_BUFFER_SIZE];
  int len = uart_interface_s.recv(receive_buffer, UBLOX_RECEIVE_BUFFER_SIZE);

  if (len <= 0)
  {
    return -1;
  }

  // Append received data to the ring buffer
  if (ring_buffer_len + len > RING_BUFFER_SIZE)
  {
    ring_buffer_len = 0;
  }

  memcpy(ring_buffer + ring_buffer_len, receive_buffer, len);
  ring_buffer_len += len;

  // Parse data in the ring buffer
  parse_ubx_hnr_pvt();

  return 0;
}

/**
 * @brief Send UBX HNR PVT request message
 */
static int send_request_message()
{
  // UBX-HNR-PVT message request
  uint8_t request[] = {0xB5, 0x62, 0x28, 0x00, 0x00, 0x00};
  uint16_t crc = calculate_checksum(request, sizeof(request));
  uint8_t request_with_crc[sizeof(request) + 2];

  memcpy(request_with_crc, request, sizeof(request));
  request_with_crc[sizeof(request)] = (crc >> 8) & 0xFF;
  request_with_crc[sizeof(request) + 1] = crc & 0xFF;

  return uart_interface_s.send(request_with_crc, sizeof(request_with_crc));
}

/**
 * @brief Disable NMEA output to only get UBX messages
 */
static int disable_nmea_output()
{
  uint8_t disable_nmea_cmd[] = {
      0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00,
      0xC0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00,
      0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x90, 0xA9};

  return uart_interface_s.send(disable_nmea_cmd, sizeof(disable_nmea_cmd));
}

/**
 * @brief Initialize the ublox M8U HNR module
 * @param uart_interface UART interface structure
 * @return 0 on success, -1 on failure
 */
int ublox_m8u_hnr_init(uart_interface uart_if)
{
  if (is_initialized)
  {
    return 0; // Already initialized
  }

  // Copy interface functions
  uart_interface_s = uart_if;

  // Initialize GNSS data structure
  ugnss_s = (ublox_m8u_gnss *)calloc(1, sizeof(ublox_m8u_gnss));

  // Disable NMEA output to only receive UBX messages
  if (disable_nmea_output() < 0)
  {
    return -1;
  }

  // Initialize ring buffer
  ring_buffer_len = 0;

  is_initialized = 1;
  return 0;
}

/**
 * @brief Deinitialize the ublox M8U HNR module
 * @return 0 on success, -1 on failure
 */
int ublox_m8u_hnr_deinit()
{
  if (!is_initialized)
  {
    return -1;
  }

  free(ugnss_s);
  ugnss_s = NULL;
  is_initialized = 0;
  return 0;
}

/**
 * @brief Get the latest GNSS data
 * @return Pointer to the latest GNSS data
 */
ublox_m8u_gnss *ublox_m8u_hnr_get_gnss_data()
{
  // First, request new data from the module
  send_request_message();

  // Process any received data
  process_received_data();

  // Wait a bit to ensure we received a response
  if (uart_interface_s.delay_ms != NULL)
  {
    uart_interface_s.delay_ms(25);
  }

  // Process any more received data
  process_received_data();

  return ugnss_s;
}
