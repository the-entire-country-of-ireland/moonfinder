#include "utils.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void printMatrix3d(const Eigen::Matrix3d& mat) {
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      Serial.print(mat(row, col), 6);
      Serial.print('\t');
    }
    Serial.println();
  }
}

void printVector3d(const Eigen::Vector3d& vec) {
  for (int index = 0; index < 3; ++index) {
    Serial.println(vec(index), 6);
  }
}

const char* resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXT_RESET";
    case ESP_RST_SW: return "SOFTWARE_RESET";
    case ESP_RST_PANIC: return "PANIC_EXCEPTION";
    case ESP_RST_INT_WDT: return "INTERRUPT_WATCHDOG";
    case ESP_RST_TASK_WDT: return "TASK_WATCHDOG";
    case ESP_RST_WDT: return "OTHER_WATCHDOG";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
    default: return "UNKNOWN";
  }
}

void dumpRuntime(const char* where, int sectorSampleCount, int collectedSectors) {
  Serial.printf("[%s] t=%lu heap=%u min_heap=%u stack_hwm=%u sector=%d sectors=%d\n",
                where, millis(), ESP.getFreeHeap(), ESP.getMinFreeHeap(),
                uxTaskGetStackHighWaterMark(NULL), sectorSampleCount,
                collectedSectors);
}