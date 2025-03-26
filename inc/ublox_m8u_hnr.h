#ifndef UBLOX_M8U_HNR_H

#include "stdint.h"

#ifdef __cplusplus
extern "C"
{
#endif
  /**
   * @brief GNSS fix type
   */
  typedef enum
  {
    NO_FIX = 0,
    DEAD_RECKONING_ONLY = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_GPS_DEAD_RECKONING = 4,
    TIME_ONLY_FIX = 5
  } ublox_m8u_fix_type_t;

  /**
   * @brief Function pointer for sending data over UART
   * @param data Pointer to data to be sent
   * @param data_length Length of data to be sent
   * @return Number of bytes sent, negative value on failure
   */
  typedef int (*uart_send)(uint8_t *data, uint32_t data_length);

  /**
   * @brief Function pointer for receiving data over UART
   * @param data Pointer to buffer for received data
   * @param data_length Maximum length of data to receive
   * @return Number of bytes received, negative value on failure
   */
  typedef int (*uart_recv)(uint8_t *data, uint32_t data_length);

  /**
   * @brief Function pointer for delaying execution
   * @param ms Number of milliseconds to delay
   */
  typedef void (*delay_ms)(uint32_t ms);

  /**
   * @brief Function pointer for locking a mutex
   */
  typedef void (*mutex_lock)();

  /**
   * @brief Function pointer for unlocking a mutex
   *
   */
  typedef void (*mutex_unlock)();

  /**
   * @brief Function pointer for unlocking a mutex
   */
  typedef void (*mutex_unlock)();

  /**
   * @brief UART interface structure
   */
  typedef struct
  {
    uart_send send;      // Function pointer for sending data
    uart_recv recv;      // Function pointer for receiving data
    delay_ms delay_ms;   // Function pointer for delaying execution
    mutex_lock lock;     // Optional Function pointer for locking a mutex / semaphore
    mutex_unlock unlock; // Optional Function pointer for unlocking a mutex / semaphore
  } uart_interface;

  /**
   * @brief ublox m8u location / GNSS data structure
   */
  typedef struct
  {
    int32_t longitude; // Longitude in microdegrees
    int32_t latitude;  // Latitude in microdegrees
    ublox_m8u_fix_type_t fix_quality;
    uint8_t num_satellites;
    uint8_t hdop;       // Horizontal Dilution of Precision in tenths
    int32_t altitude;   // Altitude in millimeters
    int32_t speed;      // Ground Speed in m/h
    uint16_t heading;   // Heading in tenths of degrees
    uint64_t timestamp; // Timestamp in seconds
  } ublox_m8u_gnss;

  /**
   * @brief Initialize the ublox M8U HNR module
   * @param uart_interface UART interface structure
   * @return 0 on success, -1 on failure
   */
  int ublox_m8u_hnr_init(uart_interface uart_interface);

  /**
   * @brief Deinitialize the ublox M8U HNR module
   * @return 0 on success, -1 on failure
   */
  int ublox_m8u_hnr_deinit();

  /**
   * @brief Get the latest GNSS data
   * @return Pointer to the latest GNSS data
   */
  ublox_m8u_gnss *ublox_m8u_hnr_get_gnss_data();
#ifdef __cplusplus
}
#endif

#endif // UBLOX_M8U_HNR_H