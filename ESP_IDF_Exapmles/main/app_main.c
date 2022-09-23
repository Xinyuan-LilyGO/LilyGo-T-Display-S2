#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "image.h"
#include "font.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "esp_err.h"
#include "esp_log.h"
#include <sys/unistd.h>
#include <sys/stat.h>
#include "driver/adc.h"
#include <sys/time.h>
#include "esp_sleep.h"

static const char *TAG = "demo";
#define SPIFIFOSIZE                                 16
#define SWAPBYTES(i)                                ((i>>8) | (i<<8))

#define ST7789_SLPIN                                0x10
#define ST7789_SLPOUT                               0x11
#define ST7789_NORON                                0x13
#define ST7789_MADCTL                               0x36      // Memory data access control
#define TFT_MAD_RGB                                 0x08
#define ST7789_COLMOD                               0x3A
#define ST7789_PORCTRL                              0xB2      // Porch control
#define ST7789_GCTRL                                0xB7      // Gate control
#define ST7789_VCOMS                                0xBB      // VCOMS setting
#define ST7789_LCMCTRL                              0xC0      // LCM control
#define ST7789_VDVVRHEN                             0xC2      // VDV and VRH command enable
#define ST7789_VRHS                                 0xC3      // VRH set
#define ST7789_VDVSET                               0xC4      // VDV setting
#define ST7789_FRCTR2                               0xC6      // FR Control 2
#define ST7789_PWCTRL1                              0xD0      // Power control 1
#define ST7789_PVGAMCTRL                            0xE0      // Positive voltage gamma control
#define ST7789_NVGAMCTRL                            0xE1      // Negative voltage gamma control
#define ST7789_INVON                                0x21
#define ST7789_CASET                                0x2A
#define ST7789_RASET                                0x2B
#define ST7789_RAMWR                                0x2C
#define ST7789_DISPOFF                              0x28
#define ST7789_DISPON                               0x29
#define TFT_MAD_COLOR_ORDER                         TFT_MAD_RGB
#define TFT_MAD_MY                                  0x80
#define TFT_MAD_MX                                  0x40
#define TFT_MAD_MV                                  0x20
#define TFT_MAD_ML                                  0x10

#define TFT_BLACK                                   0x0000      /*   0,   0,   0 */
#define TFT_NAVY                                    0x000F      /*   0,   0, 128 */
#define TFT_DARKGREEN                               0x03E0      /*   0, 128,   0 */
#define TFT_DARKCYAN                                0x03EF      /*   0, 128, 128 */
#define TFT_MAROON                                  0x7800      /* 128,   0,   0 */
#define TFT_PURPLE                                  0x780F      /* 128,   0, 128 */
#define TFT_OLIVE                                   0x7BE0      /* 128, 128,   0 */
#define TFT_LIGHTGREY                               0xD69A      /* 211, 211, 211 */
#define TFT_DARKGREY                                0x7BEF      /* 128, 128, 128 */
#define TFT_BLUE                                    0x001F      /*   0,   0, 255 */
#define TFT_GREEN                                   0x07E0      /*   0, 255,   0 */
#define TFT_CYAN                                    0x07FF      /*   0, 255, 255 */
#define TFT_RED                                     0xF800      /* 255,   0,   0 */
#define TFT_MAGENTA                                 0xF81F      /* 255,   0, 255 */
#define TFT_YELLOW                                  0xFFE0      /* 255, 255,   0 */
#define TFT_WHITE                                   0xFFFF      /* 255, 255, 255 */
#define TFT_ORANGE                                  0xFDA0      /* 255, 180,   0 */
#define TFT_GREENYELLOW                             0xB7E0      /* 180, 255,   0 */
#define TFT_PINK                                    0xFE19      /* 255, 192, 203 */ //Lighter pink, was 0xFC9F      
#define TFT_BROWN                                   0x9A60      /* 150,  75,   0 */
#define TFT_GOLD                                    0xFEA0      /* 255, 215,   0 */
#define TFT_SILVER                                  0xC618      /* 192, 192, 192 */
#define TFT_SKYBLUE                                 0x867D      /* 135, 206, 235 */
#define TFT_VIOLET                                  0x915C      /* 180,  46, 226 */

#define LCD_HOST                                    SPI2_HOST
#define DMA_CHAN                                    LCD_HOST

#define PIN_NUM_MISO                                4
#define PIN_NUM_MOSI                                35
#define PIN_NUM_CLK                                 36
#define PIN_NUM_CS                                  34

#define PIN_NUM_DC                                  37
#define PIN_NUM_RST                                 38
#define PIN_NUM_BCKL                                33

#define NO_OF_SAMPLES                               64          //Multisampling

/**         SPI SD CARD
  */
#define MOUNT_POINT "/sdcard"

// on ESP32-S2, DMA channel must be the same as host id
#define SPI_DMA_CHAN    host.slot

// DMA channel to be used by the SPI peripheral
#ifndef SPI_DMA_CHAN
#define SPI_DMA_CHAN    1
#endif //SPI_DMA_CHAN

#define SD_PIN_NUM_MISO 13
#define SD_PIN_NUM_MOSI 11
#define SD_PIN_NUM_CLK  12
#define SD_PIN_NUM_CS   10

// IO14 is connected to the SD card of the board, the power control of the LED is IO pin
#define POWER_PIN  14

uint16_t colstart = 52;
uint16_t rowstart = 40;
uint16_t _init_height = 240;
uint16_t _init_width = 135;
uint16_t _width = 135;
uint16_t _height = 240;

static spi_device_handle_t spi;
static RTC_DATA_ATTR struct timeval sleep_enter_time;

// The ADC correction function is still under test,
// so here the default reference voltage is set to 1100,
// which is consistent with esp32, this is only temporary
#define DEFAULT_VERF                            1100

void drawString(uint16_t x, uint16_t y, const char *p, uint16_t color);



void sdcard_init()
{
    esp_err_t ret;
    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI3_HOST;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_PIN_NUM_MOSI,
        .miso_io_num = SD_PIN_NUM_MISO,
        .sclk_io_num = SD_PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CHAN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        drawString(0,  0, "Failed to mount sdcard.", TFT_RED);
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }

    drawString(0,  0, "Mount sdcard success.", TFT_GREEN);

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

    // All done, unmount partition and disable SDMMC or SPI peripheral
    esp_vfs_fat_sdcard_unmount(mount_point, card);
    ESP_LOGI(TAG, "Card unmounted");

    //deinitialize the bus after all devices are removed
    spi_bus_free(host.slot);
}


/* Send a command to the LCD. Uses spi_device_polling_transmit, which waits
 * until the transfer is complete.
 *
 * Since command transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_cmd( const uint8_t cmd)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));                   //Zero out the transaction
    t.length = 8;                               //Command is 8 bits
    t.tx_buffer = &cmd;                         //The data is the cmd itself
    t.user = (void *)0;                         //D/C needs to be set to 0
    ret = spi_device_polling_transmit(spi, &t); //Transmit!
    assert(ret == ESP_OK);                      //Should have had no issues.
}

/* Send data to the LCD. Uses spi_device_polling_transmit, which waits until the
 * transfer is complete.
 *
 * Since data transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_data( const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len == 0) return;                       //no need to send anything
    memset(&t, 0, sizeof(t));                   //Zero out the transaction
    t.length = len * 8;                         //Len is in bytes, transaction length is in bits.
    t.tx_buffer = data;                         //Data
    t.user = (void *)1;                         //D/C needs to be set to 1
    ret = spi_device_polling_transmit(spi, &t); //Transmit!
    assert(ret == ESP_OK);                      //Should have had no issues.
}

//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc = (int)t->user;
    gpio_set_level(PIN_NUM_DC, dc);
}

void lcd_write_u8( const uint8_t data)
{
    lcd_data( &data, 1);
}

//Initialize the display
void lcd_init(spi_device_handle_t spi)
{
    //Initialize non-SPI GPIOs
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);

    //Reset the display
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);


    lcd_cmd( ST7789_SLPOUT);                // Sleep out
    vTaskDelay(120 / portTICK_RATE_MS);

    lcd_cmd( ST7789_NORON);                 // Normal display mode on

    //------------------------------display and color format setting--------------------------------//
    lcd_cmd( ST7789_MADCTL);
    lcd_write_u8( TFT_MAD_RGB);

    // JLX240 display datasheet
    lcd_cmd( 0xB6);
    lcd_write_u8( 0x0A);
    lcd_write_u8( 0x82);

    lcd_cmd( ST7789_COLMOD);
    lcd_write_u8( 0x55);
    vTaskDelay(10 / portTICK_RATE_MS);

    //--------------------------------ST7789V Frame rate setting----------------------------------//
    lcd_cmd( ST7789_PORCTRL);
    lcd_write_u8( 0x0c);
    lcd_write_u8( 0x0c);
    lcd_write_u8( 0x00);
    lcd_write_u8( 0x33);
    lcd_write_u8( 0x33);

    lcd_cmd( ST7789_GCTRL);                 // Voltages: VGH / VGL
    lcd_write_u8( 0x35);

    //---------------------------------ST7789V Power setting--------------------------------------//
    lcd_cmd( ST7789_VCOMS);
    lcd_write_u8( 0x28);                    // JLX240 display datasheet

    lcd_cmd( ST7789_LCMCTRL);
    lcd_write_u8( 0x0C);

    lcd_cmd( ST7789_VDVVRHEN);
    lcd_write_u8( 0x01);
    lcd_write_u8( 0xFF);

    lcd_cmd( ST7789_VRHS);                  // voltage VRHS
    lcd_write_u8( 0x10);

    lcd_cmd( ST7789_VDVSET);
    lcd_write_u8( 0x20);

    lcd_cmd( ST7789_FRCTR2);
    lcd_write_u8( 0x0f);

    lcd_cmd( ST7789_PWCTRL1);
    lcd_write_u8( 0xa4);
    lcd_write_u8( 0xa1);

    //--------------------------------ST7789V gamma setting---------------------------------------//
    lcd_cmd( ST7789_PVGAMCTRL);
    lcd_write_u8( 0xd0);
    lcd_write_u8( 0x00);
    lcd_write_u8( 0x02);
    lcd_write_u8( 0x07);
    lcd_write_u8( 0x0a);
    lcd_write_u8( 0x28);
    lcd_write_u8( 0x32);
    lcd_write_u8( 0x44);
    lcd_write_u8( 0x42);
    lcd_write_u8( 0x06);
    lcd_write_u8( 0x0e);
    lcd_write_u8( 0x12);
    lcd_write_u8( 0x14);
    lcd_write_u8( 0x17);

    lcd_cmd( ST7789_NVGAMCTRL);
    lcd_write_u8( 0xd0);
    lcd_write_u8( 0x00);
    lcd_write_u8( 0x02);
    lcd_write_u8( 0x07);
    lcd_write_u8( 0x0a);
    lcd_write_u8( 0x28);
    lcd_write_u8( 0x31);
    lcd_write_u8( 0x54);
    lcd_write_u8( 0x47);
    lcd_write_u8( 0x0e);
    lcd_write_u8( 0x1c);
    lcd_write_u8( 0x17);
    lcd_write_u8( 0x1b);
    lcd_write_u8( 0x1e);

    lcd_cmd(ST7789_INVON);

    lcd_cmd( ST7789_DISPON);   //Display on
    vTaskDelay(120 / portTICK_RATE_MS);

    ///Enable backlight
    gpio_set_level(PIN_NUM_BCKL, 1);
}

void lcd_write_byte( const uint16_t data)
{
    uint8_t val;
    val = data >> 8 ;
    lcd_data( &val, 1);
    val = data;
    lcd_data( &val, 1);
}

void setAddress( uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    lcd_cmd( ST7789_CASET);
    lcd_write_byte( x1 + colstart);
    lcd_write_byte( x2 + colstart);
    lcd_cmd( ST7789_RASET);
    lcd_write_byte( y1 + rowstart);
    lcd_write_byte( y2 + rowstart);
    lcd_cmd( ST7789_RAMWR);
}

void setRotation( uint8_t m)
{
    uint8_t rotation = m % 4;
    lcd_cmd( ST7789_MADCTL);
    switch (rotation) {
    case 0:
        colstart = 52;
        rowstart = 40;
        _width  = _init_width;
        _height = _init_height;
        lcd_write_u8( TFT_MAD_COLOR_ORDER);
        break;

    case 1:
        colstart = 40;
        rowstart = 53;
        _width  = _init_height;
        _height = _init_width;
        lcd_write_u8( TFT_MAD_MX | TFT_MAD_MV | TFT_MAD_COLOR_ORDER);
        break;
    case 2:
        colstart = 53;
        rowstart = 40;
        _width  = _init_width;
        _height = _init_height;
        lcd_write_u8( TFT_MAD_MX | TFT_MAD_MY | TFT_MAD_COLOR_ORDER);
        break;
    case 3:
        colstart = 40;
        rowstart = 52;
        _width  = _init_height;
        _height = _init_width;
        lcd_write_u8( TFT_MAD_MV | TFT_MAD_MY | TFT_MAD_COLOR_ORDER);
        break;
    }
}



void lcd_send_uint16_r(const uint16_t data, int32_t repeats)
{
    uint32_t i;
    uint32_t word = data << 16 | data;
    uint32_t word_tmp[16];
    spi_transaction_t t;

    while (repeats > 0) {
        uint16_t bytes_to_transfer = MIN(repeats * sizeof(uint16_t), SPIFIFOSIZE * sizeof(uint32_t));
        for (i = 0; i < (bytes_to_transfer + 3) / 4; i++) {
            word_tmp[i] = word;
        }
        memset(&t, 0, sizeof(t));           //Zero out the transaction
        t.length = bytes_to_transfer * 8;   //Len is in bytes, transaction length is in bits.
        t.tx_buffer = word_tmp;             //Data
        t.user = (void *) 1;                //D/C needs to be set to 1
        spi_device_transmit(spi, &t);        //Transmit!
        repeats -= bytes_to_transfer / 2;
    }
}


void fillRect( int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color)
{
    // Clipping
    if ((x >= _width) || (y >= _height)) return;

    if (x < 0) {
        w += x;
        x = 0;
    }
    if (y < 0) {
        h += y;
        y = 0;
    }

    if ((x + w) > _width)  w = _width  - x;
    if ((y + h) > _height) h = _height - y;

    if ((w < 1) || (h < 1)) return;


    setAddress( x, y, x + w - 1, y + h - 1);

    lcd_send_uint16_r(SWAPBYTES(color), h * w);
}

void fillScreen( uint32_t color)
{
    fillRect( 0, 0, _width, _height, color);
}

void drawPixel(int32_t x, int32_t y, uint32_t color)
{
    setAddress(x, y, x, y);
    lcd_write_byte(color);
}

void drawChar(uint16_t x, uint16_t y, uint8_t num, uint8_t mode, uint16_t color)
{
    uint8_t temp;
    uint8_t pos, t;
    uint16_t x0 = x;
    if (x > _width - 16 || y > _height - 16)return;
    num = num - ' ';
    setAddress(x, y, x + 8 - 1, y + 16 - 1);
    if (!mode) {
        for (pos = 0; pos < 16; pos++) {
            temp = asc2_1608[(uint16_t)num * 16 + pos];
            for (t = 0; t < 8; t++) {
                if (temp & 0x01)
                    lcd_write_byte(color);
                else
                    lcd_write_byte(TFT_BLACK);
                temp >>= 1;
                x++;
            }
            x = x0;
            y++;
        }
    } else {
        for (pos = 0; pos < 16; pos++) {
            temp = asc2_1608[(uint16_t)num * 16 + pos];
            for (t = 0; t < 8; t++) {
                if (temp & 0x01)
                    drawPixel(x + t, y + pos, color);
                temp >>= 1;
            }
        }
    }
}

void drawString(uint16_t x, uint16_t y, const char *p, uint16_t color)
{
    while (*p != '\0') {
        if (x > _width - 16) {
            x = 0;
            y += 16;
        }
        if (y > _height - 16) {
            y = x = 0;
            fillScreen(TFT_RED);
        }
        drawChar(x, y, *p, 0, color);
        x += 8;
        p++;
    }
}


uint32_t read_adc_raw()
{
    uint32_t adc_reading = 0;
    //Multisampling
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        adc_reading += adc1_get_raw((adc1_channel_t)ADC_CHANNEL_8);
    }
    adc_reading /= NO_OF_SAMPLES;
    return adc_reading;
}


void app_main(void)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

    switch (esp_sleep_get_wakeup_cause()) {

    case ESP_SLEEP_WAKEUP_TIMER:
        printf("Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
        break;
    case ESP_SLEEP_WAKEUP_UNDEFINED:
    default:
        printf("Not a deep sleep reset\n");

    }
    //Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_13);

    // GPIO9 ADC1 CHANNEL 8
    adc1_config_channel_atten(ADC_CHANNEL_8, ADC_ATTEN_DB_11);


    gpio_pad_select_gpio(POWER_PIN);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(POWER_PIN, GPIO_MODE_OUTPUT);

    /* Blink off (output low) */
    printf("Turning on the peripherals power\n");
    gpio_set_level(POWER_PIN, 1);

    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SPIFIFOSIZE * 240 * 2 + 8
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 26 * 1000 * 1000,     //Clock out at 26 MHz
        .mode = 0,                              //SPI mode 0
        .spics_io_num = PIN_NUM_CS,             //CS pin
        .queue_size = 7,                        //We want to be able to queue 7 transactions at a time
        .pre_cb = lcd_spi_pre_transfer_callback, //Specify pre-transfer callback to handle D/C line
    };
    //Initialize the SPI bus
    ret = spi_bus_initialize(LCD_HOST, &buscfg, DMA_CHAN);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret = spi_bus_add_device(LCD_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    //Initialize the LCD
    lcd_init(spi);

    setRotation(0);

    setAddress( 0, 0,  _width - 1, _height - 1);

    for (uint32_t i = 0; i < sizeof(gImage_1) ; i++) {
        lcd_data( &gImage_1[i], 1);
    }

    vTaskDelay(3000 / portTICK_RATE_MS);

    setRotation(1);

    sdcard_init();

    // The current ADC calibration function is temporarily unavailable,
    // so the ADC voltage is not accurate
    /*
        for (int i = 0; i < 10; i++) {
            uint32_t raw = read_adc_raw();
            char buff[128];
            float battery_voltage = ((float)raw / 8191.0) * 2.0 * 3.3 * (DEFAULT_VERF / 1000.0);
            snprintf(buff, 128, "%.2fV raw:%u", battery_voltage, raw);
            fillScreen(TFT_BLACK);
            drawString(0, 0, buff, TFT_GREEN);
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
        fillScreen(TFT_BLACK);
    */
    drawString(0,  18, "Now goto deepsleep", TFT_GREEN);
    vTaskDelay(3000 / portTICK_RATE_MS);

    lcd_cmd(ST7789_DISPOFF);
    lcd_cmd(ST7789_SLPIN);

    const int wakeup_time_sec = 60;
    printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
    esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);

    printf("Entering deep sleep\n");
    gettimeofday(&sleep_enter_time, NULL);

    esp_deep_sleep_start();
}
