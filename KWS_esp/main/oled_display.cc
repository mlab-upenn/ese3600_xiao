
// SSD1306 OLED driver accessed via I2C and font8x8 to render response text and a progress bar.

#include "oled_display.h"

#include <string.h>

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "font8x8_basic.h"

#define I2C_PORT I2C_NUM_0
#define TAG "OLED"

// SSD1306 control bytes
#define OLED_CONTROL_CMD  0x00
#define OLED_CONTROL_DATA 0x40

// 1KB framebuffer for 128x64 display (8 pages x 128 columns)
static uint8_t s_framebuffer[OLED_WIDTH * (OLED_HEIGHT / 8)] = {0};
static bool s_oled_ready = false;

// Builds and sends I2C command
static esp_err_t i2c_send_bytes(uint8_t control, const uint8_t* data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, control, true);
    if (len) {
        i2c_master_write(cmd, (uint8_t*)data, len, true);
    }
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static inline esp_err_t oled_cmd(uint8_t cmd) {
    //Send in single command byte
    return i2c_send_bytes(OLED_CONTROL_CMD, &cmd, 1);
}

static esp_err_t oled_cmd_list(const uint8_t* cmds, size_t len) {
    //Send in a list of command bytes
    return i2c_send_bytes(OLED_CONTROL_CMD, cmds, len);

}

static esp_err_t oled_data(const uint8_t* data, size_t len) {
    //Send in chunks to keep I2C transaction size reasonable
    const size_t chunk = 16;
    for (size_t i = 0; i < len; i += chunk) {
        size_t n = (len - i > chunk) ? chunk : (len - i);
        esp_err_t ret = i2c_send_bytes(OLED_CONTROL_DATA, data + i, n);
        if (ret != ESP_OK) return ret;
    }
    return ESP_OK;
}

// Setting up oled primitives
static esp_err_t oled_set_addressing_full(void) {
    // Column range: 0..127, Page range: 0..7
    uint8_t cmds[] = {0x21, 0x00, 0x7F, 0x22, 0x00, 0x07};
    return oled_cmd_list(cmds, sizeof(cmds));
}

static esp_err_t oled_init_hw(void) {
    // Initializing I2C for communication
    i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = OLED_SDA_PIN,
            .scl_io_num = OLED_SCL_PIN,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master = {.clk_speed = 400000},
            .clk_flags = 0,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    esp_err_t ret = i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }

    // SSD1306 init sequence for 128x64
    esp_err_t err = ESP_OK;
    err |= oled_cmd(0xAE);                            // Display OFF
    err |= oled_cmd_list((uint8_t[]){0xD5, 0x80}, 2); // Set display clock divide
    err |= oled_cmd_list((uint8_t[]){0xA8, 0x3F}, 2); // Multiplex 1/64
    err |= oled_cmd_list((uint8_t[]){0xD3, 0x00}, 2); // Display offset
    err |= oled_cmd(0x40);                            // Start line = 0
    err |= oled_cmd_list((uint8_t[]){0x8D, 0x14}, 2); // Charge pump ON
    err |= oled_cmd_list((uint8_t[]){0x20, 0x00}, 2); // Horizontal addressing mode
    err |= oled_cmd(0xA1);                            // Segment remap
    err |= oled_cmd(0xC8);                            // COM scan direction remap
    err |= oled_cmd_list((uint8_t[]){0xDA, 0x12}, 2); // COM pins
    err |= oled_cmd_list((uint8_t[]){0x81, 0xCF}, 2); // Contrast
    err |= oled_cmd_list((uint8_t[]){0xD9, 0xF1}, 2); // Precharge
    err |= oled_cmd_list((uint8_t[]){0xDB, 0x40}, 2); // VCOM detect
    err |= oled_cmd(0xA4);                            // Resume to RAM content
    err |= oled_cmd(0xA6);                            // Normal display
    err |= oled_cmd(0x2E);                            // Deactivate scroll
    err |= oled_cmd(0xAF);                            // Display ON
    return err;
}

//Framebuffer pixel manipulation
static inline void set_pixel(int x, int y, bool on) {
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) return;
    int page = y / 8;
    int bit = y % 8;
    uint8_t* cell = &s_framebuffer[page * OLED_WIDTH + x];
    if (on) {
        *cell |= (1u << bit);
    } else {
        *cell &= ~(1u << bit);
    }
}

// Displays character using font8x8_basic
static void draw_char(int x, int y, char c) {
    if ((unsigned char)c > 127) c = '?';
    const uint8_t* glyph = (const uint8_t*)font8x8_basic[(int)(unsigned char)c];
    for (int dy = 0; dy < 8; ++dy) {
        uint8_t row = glyph[dy];
        for (int dx = 0; dx < 8; ++dx) {
            bool on = (row >> dx) & 0x01;
            set_pixel(x + dx, y + dy, on);
        }
    }
}

// Displays text string at position (x,y)
static void draw_text(int x, int y, const char* text) {
    int cx = x;
    for (const char* p = text; *p; ++p) {
        if (*p == '\n') {
            y += 8; cx = x; continue;
        }
        draw_char(cx, y, *p);
        cx += 8;
        if (cx + 8 >= OLED_WIDTH) { y += 8; cx = x; }
        if (y >= OLED_HEIGHT) break;
    }
}

// Function to initialize the OLED display using the functions above and log status
esp_err_t oled_display_init(void) {
    if (s_oled_ready) return ESP_OK;
    esp_err_t ret = oled_init_hw();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OLED init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    memset(s_framebuffer, 0x00, sizeof(s_framebuffer));
    s_oled_ready = true;
    ESP_LOGI(TAG, "OLED initialized: I2C SDA=%d SCL=%d addr=0x%02X", (int)OLED_SDA_PIN, (int)OLED_SCL_PIN, OLED_I2C_ADDR);
    return oled_display_update();
}

void oled_display_clear(void) {
    memset(s_framebuffer, 0x00, sizeof(s_framebuffer));
}

void oled_display_text(uint8_t x, uint8_t y, const char* text) {
    if (!text) return;
    draw_text(x, y, text);
}

void oled_display_command(const char* command, uint8_t score) {
    // Title and a progress bar 
    oled_display_clear();

    char line[32];
    snprintf(line, sizeof(line), "CMD: %s", (command ? command : "?"));
    draw_text(0, 0, line);

    // Progress bar (0..100) indicating the confidence score
    int bar_w = (score > 100 ? 100 : score) * (OLED_WIDTH - 2) / 100;
    int y0 = OLED_HEIGHT - 12;
    draw_text(0, y0 - 8, "CONF:");
    for (int x = 0; x < bar_w; ++x) {
        for (int y = 0; y < 8; ++y) {
            set_pixel(40 + x, y0 + y, true);
        }
    }
    oled_display_update();
}

esp_err_t oled_display_update(void) {
    if (!s_oled_ready) return ESP_ERR_INVALID_STATE;
    esp_err_t err = oled_set_addressing_full();
    if (err != ESP_OK) return err;
    return oled_data(s_framebuffer, sizeof(s_framebuffer));
}
