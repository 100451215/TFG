#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "driver/uart.h"
//#include "soc/uart_struct.h"

//#include "esp_dsp.h"
#include "esp_timer.h"

// mis includes----
#include <unistd.h>


#include "esp_log.h"
#include "esp_clk_tree.h"
//#include "esp_cpu.h"
//#include "esp32-hal-cpu.h"
#include "esp_pm.h"

 

#include "esp_system.h"
//#include "esp_clk.h"
//esp_cpu_get_cycle_count()
//esp_cpu_set_cycle_count()
#include "driver/temperature_sensor.h"
#include "driver/i2c.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "lwip/err.h" //light weight ip packets error handling
#include "lwip/sys.h" //system applications for light weight ip apps
#include "nvs_flash.h"
//#include "time.h"
#include "esp_sntp.h"
#include "esp_netif_sntp.h"

static const char *TAG = "TFG";

//----- parte nueva micro sd
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "sd_test_io.h"
#if SOC_SDMMC_IO_POWER_EXTERNAL
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#endif

#define EXAMPLE_MAX_CHAR_SIZE    64

#define MOUNT_POINT "/sdcard"

#ifdef CONFIG_EXAMPLE_DEBUG_PIN_CONNECTIONS
const char* names[] = {"CLK ", "MOSI", "MISO", "CS  "};
const int pins[] = {CONFIG_EXAMPLE_PIN_CLK,
                    CONFIG_EXAMPLE_PIN_MOSI,
                    CONFIG_EXAMPLE_PIN_MISO,
                    CONFIG_EXAMPLE_PIN_CS};

const int pin_count = sizeof(pins)/sizeof(pins[0]);
#if CONFIG_EXAMPLE_ENABLE_ADC_FEATURE
const int adc_channels[] = {CONFIG_EXAMPLE_ADC_PIN_CLK,
                            CONFIG_EXAMPLE_ADC_PIN_MOSI,
                            CONFIG_EXAMPLE_ADC_PIN_MISO,
                            CONFIG_EXAMPLE_ADC_PIN_CS};
#endif //CONFIG_EXAMPLE_ENABLE_ADC_FEATURE

pin_configuration_t config = {
    .names = names,
    .pins = pins,
#if CONFIG_EXAMPLE_ENABLE_ADC_FEATURE
    .adc_channels = adc_channels,
#endif
};
#endif //CONFIG_EXAMPLE_DEBUG_PIN_CONNECTIONS

// Pin assignments can be set in menuconfig, see "SD SPI Example Configuration" menu.
// You can also change the pin assignments here by changing the following 4 lines.
#define PIN_NUM_MISO  CONFIG_EXAMPLE_PIN_MISO
#define PIN_NUM_MOSI  CONFIG_EXAMPLE_PIN_MOSI
#define PIN_NUM_CLK   CONFIG_EXAMPLE_PIN_CLK
#define PIN_NUM_CS    CONFIG_EXAMPLE_PIN_CS

static esp_err_t s_example_write_file(const char *path, char *data)
{
    ESP_LOGI(TAG, "Opening file %s", path);
    FILE *f = fopen(path, "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fprintf(f, data);
    fclose(f);
    ESP_LOGI(TAG, "File written");

    return ESP_OK;
}

static esp_err_t s_example_read_file(const char *path)
{
    ESP_LOGI(TAG, "Reading file %s", path);
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    char line[EXAMPLE_MAX_CHAR_SIZE];
    fgets(line, sizeof(line), f);
    fclose(f);

    // strip newline
    char *pos = strchr(line, '\n');
    if (pos) {
        *pos = '\0';
    }
    ESP_LOGI(TAG, "Read from file: '%s'", line);

    return ESP_OK;
}
//-----



//uint32_t frec_MH;
float temp;

temperature_sensor_handle_t temp_handle = NULL;
temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(20, 50);

//settings
esp_pm_config_t mi_conf;
i2c_config_t config_luz;

//lectura luz
uint8_t r[2];
//comando sensor luz
uint8_t read_command = 0x10;
uint8_t w = 0x01;

esp_sntp_config_t config_tiempo = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");

//wifi 0
//const char* ntpServer = "pool.ntp.org";
//const long  gmtOffset_sec = 0;
//const int   daylightOffset_sec = 3600;


time_t now;
char strftime_buf[64];
struct tm timeinfo;


//cosas wifi 1
const char *ssid = "Nombre de red";
const char *pass = "Contrasena";
int retry_num=0;

static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id,void *event_data){
    if(event_id == WIFI_EVENT_STA_START)
    {
    printf("WIFI CONNECTING....\n");
    }
    else if (event_id == WIFI_EVENT_STA_CONNECTED)
    {
    printf("WiFi CONNECTED\n");
    }
    else if (event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
    printf("WiFi lost connection\n");
    if(retry_num<5){esp_wifi_connect();retry_num++;printf("Retrying to Connect...\n");}
    }
    else if (event_id == IP_EVENT_STA_GOT_IP)
    {
    printf("Wifi got IP...\n\n");
    }
}

void wifi_connection()
{
     //                          s1.4
    // 2 - Wi-Fi Configuration Phase
    esp_netif_init();
    esp_event_loop_create_default();     // event loop                    s1.2
    esp_netif_create_default_wifi_sta(); // WiFi station                      s1.3
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation); //     
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    wifi_config_t wifi_configuration = {
        .sta = {
            .ssid = "",
            .password = "",
           }
        };
    strcpy((char*)wifi_configuration.sta.ssid, ssid);
    strcpy((char*)wifi_configuration.sta.password, pass);    
    esp_log_write(ESP_LOG_INFO, "Kconfig", "SSID=%s, PASS=%s", ssid, pass);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
    // 3 - Wi-Fi Start Phase
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    vTaskDelay(pdMS_TO_TICKS(1000));
    // 4- Wi-Fi Connect Phase
    esp_wifi_connect();
    printf( "wifi_init_softap finished. SSID:%s  password:%s",ssid,pass);
    
}

void printLocalTime(){
    //gettimeofday();
    time(&now);
    setenv("TZ", "CST-0", 1);
    tzset();

    //sntp_sync_time();

    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time in Shanghai is: %s", strftime_buf);
  /*struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    printf("Failed to obtain time");
    return;
  }
  printf(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  printf("Day of week: ");
  printf(&timeinfo, "%A");
  printf("Month: ");
  printf(&timeinfo, "%B");
  printf("Day of Month: ");
  printf(&timeinfo, "%d");
  printf("Year: ");
  printf(&timeinfo, "%Y");
  printf("Hour: ");
  printf(&timeinfo, "%H");
  printf("Hour (12 hour format): ");
  printf(&timeinfo, "%I");
  printf("Minute: ");
  printf(&timeinfo, "%M");
  printf("Second: ");
  printf(&timeinfo, "%S");

  printf("Time variables");
  char timeHour[3];
  strftime(timeHour,3, "%H", &timeinfo);
  printf(timeHour);
  char timeWeekDay[10];
  strftime(timeWeekDay,10, "%A", &timeinfo);
  printf(timeWeekDay);*/



}

//cambios en skconfig:
//CONFIG_PM_ENABLE
void app_main(void)
{

    //### INIT ###
    mi_conf.max_freq_mhz = 80; //160 max fisico y default
    mi_conf.min_freq_mhz = 80;
    mi_conf.light_sleep_enable = false;
    esp_pm_configure(&mi_conf);

    config_luz.mode = I2C_MODE_MASTER;
    config_luz.scl_io_num = GPIO_NUM_22;
    config_luz.sda_io_num = GPIO_NUM_21;
    config_luz.master.clk_speed = 100000;
    config_luz.scl_pullup_en = 1;
    config_luz.sda_pullup_en = 1;
    config_luz.clk_flags = 0;
    i2c_param_config(I2C_NUM_0, &config_luz);
    i2c_driver_install(I2C_NUM_0,I2C_MODE_MASTER,0,0,0);
    

    i2c_master_write_to_device(I2C_NUM_0, 0x23, &w, 1, pdMS_TO_TICKS(180));// encendido

    //configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    //gettimeofday();

    //wifi2
    //nvs_flash_init();
    //wifi_connection();

    

    //esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    //esp_sntp_setservername(0, "pool.ntp.org");
    //esp_sntp_init();

    esp_netif_sntp_init(&config_tiempo);//

    if (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(1000)) != ESP_OK) {
    printf("Failed to update system time within 1s timeout");
    }

    struct tm tm;
    tm.tm_year = 2025 - 1900;
    tm.tm_mon = 6;
    tm.tm_mday = 2;
    tm.tm_hour = 20;
    tm.tm_min = 35;
    tm.tm_sec = 30;
    time_t t = mktime(&tm);
    printf("Setting time: %s", asctime(&tm));
    struct timeval now2 = { .tv_sec = t };
    settimeofday(&now2, NULL);

    // init del sensor de temperatura interno
    temperature_sensor_install(&temp_sensor_config, &temp_handle);
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_handle));

    ESP_LOGI(TAG, "*** COMIENZO ***");
    //comienzo guarreria-------
        esp_err_t ret;

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
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
    ESP_LOGI(TAG, "Using SPI peripheral");

    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    // For SoCs where the SD power can be supplied both via an internal or external (e.g. on-board LDO) power supply.
    // When using specific IO pins (which can be used for ultra high-speed SDMMC) to connect to the SD card
    // and the internal LDO power supply, we need to initialize the power supply first.
#if CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_INTERNAL_IO
    sd_pwr_ctrl_ldo_config_t ldo_config = {
        .ldo_chan_id = CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_IO_ID,
    };
    sd_pwr_ctrl_handle_t pwr_ctrl_handle = NULL;

    ret = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &pwr_ctrl_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create a new on-chip LDO power control driver");
        return;
    }
    host.pwr_ctrl_handle = pwr_ctrl_handle;
#endif

    // datos configuracion bus targeta SD
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
#ifdef CONFIG_EXAMPLE_DEBUG_PIN_CONNECTIONS
            check_sd_card_pins(&config, pin_count);
#endif
        }
        return;
    }
    ESP_LOGI(TAG, "Filesystem mounted");

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

    // Use POSIX and C standard library functions to work with files.

    // First create a file.
    const char *file_hello = MOUNT_POINT"/hello.txt";
    char data[EXAMPLE_MAX_CHAR_SIZE];
    snprintf(data, EXAMPLE_MAX_CHAR_SIZE, "%s %s!\n", "Hello", card->cid.name);
    ret = s_example_write_file(file_hello, data);
    if (ret != ESP_OK) {
        return;
    }

    const char *file_foo = MOUNT_POINT"/foo.txt";

    // Check if destination file exists before renaming
    struct stat st;
    if (stat(file_foo, &st) == 0) {
        // Delete it if it exists
        unlink(file_foo);
    }

    // Rename original file
    ESP_LOGI(TAG, "Renaming file %s to %s", file_hello, file_foo);
    if (rename(file_hello, file_foo) != 0) {
        ESP_LOGE(TAG, "Rename failed");
        return;
    }

    ret = s_example_read_file(file_foo);
    if (ret != ESP_OK) {
        return;
    }

    // Format FATFS
#ifdef CONFIG_EXAMPLE_FORMAT_SD_CARD
    ret = esp_vfs_fat_sdcard_format(mount_point, card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to format FATFS (%s)", esp_err_to_name(ret));
        return;
    }

    if (stat(file_foo, &st) == 0) {
        ESP_LOGI(TAG, "file still exists");
        return;
    } else {
        ESP_LOGI(TAG, "file doesn't exist, formatting done");
    }
#endif // CONFIG_EXAMPLE_FORMAT_SD_CARD

    const char *file_nihao = MOUNT_POINT"/algo.txt";
    memset(data, 0, EXAMPLE_MAX_CHAR_SIZE);
    snprintf(data, EXAMPLE_MAX_CHAR_SIZE, "%s %s!\n", "Nihao", card->cid.name);
    ret = s_example_write_file(file_nihao, data);
    if (ret != ESP_OK) {
        return;
    }

    
    //fin guarreria-------
    //while (1){
        //esp_clk_tree_src_get_freq_hz(SOC_MOD_CLK_CPU, 1, &frec_MH);
        esp_cpu_set_cycle_count(0);

        // leer datos de tiempo al inicio
        uint32_t ciclo_inicio = esp_cpu_get_cycle_count(); // 0
        int64_t t_inicio = esp_timer_get_time();
        
        i2c_master_write_read_device(I2C_NUM_0, 0x23, &read_command, 1, r, 2, pdMS_TO_TICKS(180));// lectura continua

        //float j = 10; comentado el primer bucle
        //for(int i = 1; i<9951; i++){
        //    j = j / i + temp;
            //printf("%d %d %f\n", i, j, temp);
        //}//
        int z;
        for (int i = 0 ; i < 7000 ; i++) {
            for (int j = 0 ; j < 7000 ; j++) {
                z = i + j;
            }
        }

        //sleep(1); // para el cpu
        temperature_sensor_get_celsius(temp_handle, &temp);

        
       

        //frec_MH = esp_cpu_get_cycle_count();
        //dsp_get_cpu_cycle_count()

        // tiempo de fin
        uint32_t ciclo_final = esp_cpu_get_cycle_count();
        int64_t t_final = esp_timer_get_time();

        //transformacion de datos
        int32_t ciclos_total = ciclo_final - ciclo_inicio;
        float t_total = (t_final - t_inicio)/1e6;
        
        ESP_LOGI(TAG, "La frecuencia es %f H, se han hecho %"PRId32" ciclos, el tiempo de %f, la temperatura es de %f ºC la luminosidad de %d %u %u\n", (float)(ciclos_total/t_total), (int32_t)(ciclos_total), (float)(t_total), (float)(temp), (int)(r[0]*256 + r[1]), (unsigned int)(r[0]), (unsigned int)(r[1]));
        
        printLocalTime(); // recoge el tiempo
        //sleep(1);

    

    //snprintf(data, EXAMPLE_MAX_CHAR_SIZE, "%f %ld %f %f %d %u %u\n", (float)(ciclos_total/t_total), (int32_t)(ciclos_total), (float)(t_total), (float)(temp), (int)(r[0]*256 + r[1]), (unsigned int)(r[0]), (unsigned int)(r[1])); 
    //}
    temperature_sensor_disable(temp_handle);

    //----

    const char *file_mediciones = MOUNT_POINT"/medcns.txt";
    memset(data, 0, EXAMPLE_MAX_CHAR_SIZE);
    snprintf(data, EXAMPLE_MAX_CHAR_SIZE, "%s", strftime_buf); 
    ret = s_example_write_file(file_mediciones, data);
    if (ret != ESP_OK) {
        return;
    }

    memset(data, 0, EXAMPLE_MAX_CHAR_SIZE);
    snprintf(data, EXAMPLE_MAX_CHAR_SIZE, " %f %ld %f %f %d %u %u\n", (float)(ciclos_total/t_total), (int32_t)(ciclos_total), (float)(t_total), (float)(temp), (int)(r[0]*256 + r[1]), (unsigned int)(r[0]), (unsigned int)(r[1])); 
    ret = s_example_write_file(file_mediciones, data);
    if (ret != ESP_OK) {
        return;
    }
    
    //Open file for reading
    ret = s_example_read_file(file_nihao);
    if (ret != ESP_OK) {
        return;
    }

    // All done, unmount partition and disable SPI peripheral
    esp_vfs_fat_sdcard_unmount(mount_point, card);
    ESP_LOGI(TAG, "Card unmounted");

    //deinitialize the bus after all devices are removed
    spi_bus_free(host.slot);

    // Deinitialize the power control driver if it was used
#if CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_INTERNAL_IO
    ret = sd_pwr_ctrl_del_on_chip_ldo(pwr_ctrl_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete the on-chip LDO power control driver");
        return;
    }
#endif

    //----
    ESP_LOGI(TAG, "*** FIN ***");

}


