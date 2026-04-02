#include "esp_log.h"
#include "module_core.h"
#include "logger.h"
#include "esp_random.h"

using namespace std;

static const char *TAG = "main";

#define BLINK_PIN   GPIO_NUM_2
#define CAN_TX_PIN  GPIO_NUM_21
#define CAN_RX_PIN  GPIO_NUM_20

#define MODULE_TYPE 0x00
#define FW_VERSION  0x01

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
    cfg.on_uart_rx = [&logger](const uint8_t* data, size_t len) { logger.on_uart_rx(data, len); };
    cfg.app_main   = [&logger]() { return logger.main(); };

    ESP_ERROR_CHECK(g_module.init(info, cfg));

    AdcDriver::Config adc_cfg = {
        .bus = {
            .host_id = SPI3_HOST,
            .dma_chan = SPI_DMA_CH_AUTO,
            .sclk_io_num = GPIO_NUM_5,
            .mosi_io_num = GPIO_NUM_7,
            .miso_io_num = GPIO_NUM_6,
            .max_transfer_sz = 4092,
            .initialize_bus = true,
        },
        .device = {
            .cs_io_num = GPIO_NUM_4,
            .clock_speed_hz = SPI_MASTER_FREQ_10M,
            .spi_mode = 3,
            .queue_size = 10,
        },
        .protocol = {
            .channel_count = 8, // 8 Available channels on the current ADC
            .channel_command_shift = 3, // Command Bits start on Bit 3
            .sample_mask = 0x0FFF, // For our ADC first 4 bits are 0 anws but this masks them to be 0 regardless of ADC output
        },
    };
    ESP_ERROR_CHECK(logger.init_adc(adc_cfg));

    for (;;) {
        // for (uint8_t id : { 0x01, 0x02 }) {
        //     CanFrame frame{};
        //     frame.header.ide = 1;
        //     frame.header.id  = build_arb_id(0x00, CMD_DATA, id);
        //     frame.len        = 8;
        //     for (auto &b : frame.data) b = esp_random() & 0xFF;
        //     logger.on_can_frame(&frame);
        // }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
