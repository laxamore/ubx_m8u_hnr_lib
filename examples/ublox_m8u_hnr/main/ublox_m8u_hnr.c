#include "ublox_m8u_hnr.h"

void app_main(void) {
  ublox_m8u_hnr_setup(UART_NUM_1, 17, 16, 9600);

  ublox_m8u_hnr_init();
  ublox_m8u_hnr_run();

  while (1) {
    ublox_m8u_gnss gnss_data = ublox_m8u_hnr_get_gnss_data();
    double latitude = (double)gnss_data.latitude / 1e7;
    double longitude = (double)gnss_data.longitude / 1e7;
    ESP_LOGI("GNSS",
             "Timestamp: %lu, Latitude: %f, Longitude: %f, Altitude: %d",
             gnss_data.timestamp, latitude, longitude, gnss_data.altitude);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}