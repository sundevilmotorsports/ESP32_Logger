#include "esp_log.h"
#include "module_core.h"
#include "logger.h"

using namespace std;

static const char *TAG = "main";

#define BLINK_PIN   GPIO_NUM_2
#define CAN_TX_PIN  GPIO_NUM_21
#define CAN_RX_PIN  GPIO_NUM_22

#define MODULE_TYPE 0x01
#define FW_VERSION  0x01

static ModuleCore g_module;

static void on_uart_rx(const uint8_t *data, size_t len) {
    ESP_LOGI(TAG, "Unhandled UART data: len=%d", len);
}

extern "C" void app_main() {
    ModuleInfo info;
    info.module_type = MODULE_TYPE;
    info.fw_version  = FW_VERSION;
    info.blink_pin   = BLINK_PIN;

    Logger logger;

    ModuleCore::Config cfg;
    cfg.can_tx     = CAN_TX_PIN;
    cfg.can_rx     = CAN_RX_PIN;
    cfg.uart_port  = UART_NUM_0;
    cfg.uart_baud  = 115200;
    cfg.on_can_rx  = [&logger](const CanFrame *frame) { logger.on_can_frame(frame); };
    cfg.on_uart_rx = on_uart_rx;
    cfg.app_main   = [&logger]() { return logger.main(); };

    ESP_ERROR_CHECK(g_module.init(info, cfg));
}
