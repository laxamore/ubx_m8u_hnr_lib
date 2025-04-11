#ifndef UBX_M8_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

  #define UBX_M8_BUFFER_SIZE 1024 // Size of the ring buffer for incoming data

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
  } ubx_m8_fix_t;

  /**
   * @brief UBX Port Interface
   */
  typedef enum
  {
    UBX_M8_PORT_DDC = 0,
    UBX_M8_PORT_UART1 = 1,
    UBX_M8_PORT_USB = 3,
    UBX_M8_PORT_SPI = 4
  } ubx_m8_port_t;

  /**
   * @brief location / GNSS data structure
   */
  typedef struct
  {
    int32_t longitude; // Longitude in microdegrees
    int32_t latitude;  // Latitude in microdegrees
    ubx_m8_fix_t fix_quality;
    uint8_t num_satellites;
    int32_t altitude;   // Altitude in millimeters
    int32_t speed;      // Ground Speed in m/h
    uint16_t heading;   // Heading in tenths of degrees
    uint64_t timestamp; // Timestamp in seconds
  } ubx_m8_gnss_t;

  /**
   * @brief Function pointer for sending data over interface
   *
   * @param data Pointer to data to be sent
   * @param data_length Length of data to be sent
   *
   * @return Number of bytes sent, negative value on failure
   */
  typedef int (*if_send_fn)(uint8_t *data, uint32_t data_length);

  /**
   * @brief Function pointer for receiving data over interface
   *
   * @param data Pointer to buffer for received data
   * @param data_length Maximum length of data to receive
   *
   * @return Number of bytes received, negative value on failure
   */
  typedef int (*if_recv_fn)(uint8_t *data, uint32_t data_length);

  typedef void (*if_delay_ms_fn)(uint32_t delay_ms);

  /**
   * @brief Function pointer for locking a mutex
   */
  typedef void (*mutex_lock_fn)(void);

  /**
   * @brief Function pointer for unlocking a mutex
   */
  typedef void (*mutex_unlock_fn)(void);

  /**
   * @brief Interface struct implemented by user
   */
  typedef struct
  {
    if_send_fn send;      // Function pointer for sending data to interface
    if_recv_fn recv;      // Function pointer for receiving data to interface
    if_delay_ms_fn delay_ms; // Function pointer for delay in milliseconds
  } ubx_if_t;

  /**
   * @brief Initialize the UBX M8 module
   *
   * @param ubx_m8_port Port to use for communication @enum ubx_m8_port_t
   * @param ubx_if interface structure
   *
   * @return 0 on success, -1 on failure
   */
  int ubx_m8_init(ubx_m8_port_t ubx_m8_port, ubx_if_t ubx_if);

  /**
   * @brief Initialize the UBX M8 Module with optional mutex lock and unlock
   *
   * @param ubx_m8_port Port to use for communication @enum ubx_m8_port_t
   * @param ubx_if interface structure
   * @param lock pointer to function implementing mutex_lock_fn
   * @param unlock pointer to function implementing mutex_unlock_fn
   *
   * @return 0 on success, -1 on failure
   */
  int ubx_m8_init_with_mutex(ubx_m8_port_t ubx_m8_port, ubx_if_t ubx_if, mutex_lock_fn lock, mutex_unlock_fn unlock);

  /**
   * @brief Deinitialize the UBX M8 module
   *
   * @return 0 on success, -1 on failure
   */
  int ubx_m8_deinit();

  /**
   * @brief Get the latest GNSS data
   *
   * @return Pointer to the latest GNSS data
   */
  ubx_m8_gnss_t *ubx_m8_get_gnss_data();

  /**
   * @brief Disable NMEA output
   *
   * @return 0 on success, -1 on failure
   */
  int ubx_m8_disable_nmea_output(void);

  /**
   * @brief Set the HNR rate
   *
   * @param rate Rate in hz (1-30)
   *
   * @return 0 on success, -1 on failure
   */
  int ubx_m8_set_hnr_rate(uint8_t rate);

#ifdef __cplusplus
}
#endif

#endif // UBX_M8_H