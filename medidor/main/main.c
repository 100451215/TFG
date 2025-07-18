#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"

#include "esp_dsp.h"
#include "esp_timer.h"

// mis includes----
#include <unistd.h>


#include "esp_log.h"
#include "esp_clk_tree.h"
#include "esp_pm.h"


 

#include "esp_system.h"
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

#define EXAMPLE_MAX_CHAR_SIZE    128 // buffer de paso de datos a la microsd

#define MOUNT_POINT "/sdcard" // fichero raiz microsd

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
#define DHT_PIN 18  

#define TIMEOUT_US 1000

//-----
// para el benchmark
#define N_SAMPLES 5000// 1024 //32768
int N = N_SAMPLES;
// Input test array
__attribute__((aligned(16)))
float x1[N_SAMPLES];
// Window coefficients
__attribute__((aligned(16)))
float wind[N_SAMPLES];
// working complex array
__attribute__((aligned(16)))
float y_cf[N_SAMPLES * 2];
// Pointers to result arrays
float *y1_cf = &y_cf[0];


//uint32_t frec_MH;
// parametros medidas
int numero_medidas;
int max_medidas = 6*2*12; //12 * 12; //30
int delay_medidas = 5 * 60; //segundos


// variables percepcion
uint64_t ciclo_final;
uint64_t ciclo_inicio;
uint64_t ciclos_total;
int64_t t_final;
int64_t t_inicio;
float t_total;

float humedad;
float temperatura;
float temp_cpu;

bool end_dht = false;
bool end_benchmark = false;
bool end_ccl_tracker = false;
bool flag_ccl_tracker = false;


//uint16_t rawHumidity = 0;
//uint16_t rawTemperature = 0;
//uint16_t data_temp = 0;
//int expected;
//int DHT_GPIO = 9;
//int pin = 9;

// --settings--
// sensor temperatura cpu
temperature_sensor_handle_t temp_handle = NULL;
temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(20, 50);

//sensor luz
esp_pm_config_t mi_conf;
i2c_config_t config_luz;
uint8_t r[2]; //lectura luz
//comandos sensor luz
uint8_t read_command = 0x10;
uint8_t w = 0x01;

// tiempo
//esp_sntp_config_t config_tiempo = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
time_t now;
char strftime_buf[64];
struct tm timeinfo;
//wifi 0
//const char* ntpServer = "pool.ntp.org";
//const long  gmtOffset_sec = 0;
//const int   daylightOffset_sec = 3600;

//cosas wifi 1
const char *ssid = "Nombre de red";
const char *pass = "Contrasena";
int retry_num=0;

//microsd
esp_err_t ret;

// operaciones del benchmark
static void process_and_show(float *data, int length)
{
    dsps_fft2r_fc32(data, length);
    // Bit reverse
    dsps_bit_rev_fc32(data, length);
    // Convert one complex vector to two complex vectors
    dsps_cplx2reC_fc32(data, length);

    for (int i = 0 ; i < length / 2 ; i++) {
        data[i] = 10 * log10f((data[i * 2 + 0] * data[i * 2 + 0] + data[i * 2 + 1] * data[i * 2 + 1]) / N);
    }

    // Show power spectrum in 64x10 window from -100 to 0 dB from 0..N/4 samples
    //dsps_view(data, length / 2, 64, 10,  -120, 40, '|');
}

// escritura
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

// lectura (desuso)
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

// usado por dht_task
int wait_for_level(int level, int timeout_us) {
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(DHT_PIN) != level) {
        if ((esp_timer_get_time() - start) > timeout_us) return -1;
    }
    return 0;
}

void dht_task(void *pvParameter) {
    uint8_t data[5];

    // configuracion para el sensor de temperatura ambiente
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << DHT_PIN,
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    while (!end_dht) {
        memset(data, 0, sizeof(data));

        // Preparar el pin en estado HIGH (reposo)
        gpio_set_direction(DHT_PIN, GPIO_MODE_OUTPUT_OD);
        gpio_set_level(DHT_PIN, 1);              // Estado de reposo
        esp_rom_delay_us(10);                    // Esperar un poco

        // Señal de inicio: línea baja 1ms
        gpio_set_level(DHT_PIN, 0);
        esp_rom_delay_us(1000);

        // Subir a HIGH (liberar el bus) por unos 30us
        gpio_set_level(DHT_PIN, 1);
        esp_rom_delay_us(30);

        // Pasar a entrada para leer al sensor
        gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT);

        // Esperar respuesta del DHT22
        if (wait_for_level(0, TIMEOUT_US) < 0) {
            printf("Timeout esperando respuesta LOW\n");
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        if (wait_for_level(1, TIMEOUT_US) < 0) {
            printf("Timeout esperando respuesta HIGH\n");
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        // Leer los 40 bits (5 bytes)
        for (int i = 0; i < 40; i++) {
            if (wait_for_level(0, TIMEOUT_US) < 0) break;
            if (wait_for_level(1, TIMEOUT_US) < 0) break;

            int64_t start = esp_timer_get_time();
            if (wait_for_level(0, TIMEOUT_US) < 0) break;
            int64_t duration = esp_timer_get_time() - start;

            int byte_index = i / 8;
            data[byte_index] <<= 1;
            if (duration > 40) {
                data[byte_index] |= 1;  // Bit 1
            }
        }

        // Verificar el checksum
        uint8_t checksum = data[0] + data[1] + data[2] + data[3];
        if (checksum != data[4]) {
            printf("Checksum inválido\n");
        } else {
            humedad = ((data[0] << 8) | data[1]) * 0.1;
            temperatura = (((data[2] & 0x7F) << 8) | data[3]) * 0.1;
            if (data[2] & 0x80) temperatura *= -1;

            printf("Humedad: %.1f %%\tTemperatura: %.1f °C\n", humedad, temperatura);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));  // Esperar 2 segundos
    }
    ESP_LOGI(TAG, "Fin de la lectura del sensor de temperatura externo");
    vTaskDelete(NULL);
}

void benchmark(void *pvParameter){
    float x, y = 1;
    while (!end_benchmark){

                // Generate input signal
            dsps_tone_gen_f32(x1, N, 1., 0.2, 0);
            // Convert two input vectors to one complex vector
            for (int i = 0 ; i < N ; i++) {
                y_cf[i * 2 + 0] = x1[i] * wind[i];
                y_cf[i * 2 + 1] = 0;
            }
            process_and_show(y_cf, N);

            ESP_LOGI(TAG, "*** Multiply tone signal with Hann window by esp-dsp basic math functions. ***");
            // Convert two input vectors to one complex vector with basic functions
            dsps_mul_f32(x1, wind, y_cf, N, 1, 1, 2); // Multiply input array with window and store as real part
            dsps_mulc_f32(&y_cf[1], &y_cf[1], N, 0, 2, 2); // Clear imaginary part of the complex signal
            process_and_show(y_cf, N);
            for (size_t i = 0; i < 100; i++)
            {
                x = x + y;
                y = y + x/2;
                x = (x + 1)/y;
            }
            vTaskDelay(pdMS_TO_TICKS(3));
    }
    ESP_LOGI(TAG, "Fin de la ejecucion del benchmark");
    vTaskDelete(NULL);
}

void ccl_tracker(void *pvParameter){
    while (!end_ccl_tracker){
        if (!flag_ccl_tracker){
            ciclo_final = ciclo_final + esp_cpu_get_cycle_count();
            esp_cpu_set_cycle_count(0);
            vTaskDelay(pdMS_TO_TICKS(5000));
        } else { 
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
    ESP_LOGI(TAG, "Fin de la ejecucion del ccl_tracker");
    vTaskDelete(NULL);
}

void configuracion_frecuencia(int cmax){
    mi_conf.max_freq_mhz = cmax; //160 max fisico y default
    mi_conf.min_freq_mhz = 80;
    mi_conf.light_sleep_enable = false;
    esp_pm_configure(&mi_conf);
}

void tomar_medidas(){
    i2c_master_write_read_device(I2C_NUM_0, 0x23, &read_command, 1, r, 2, pdMS_TO_TICKS(180));// lectura continua luminosidad

    temperature_sensor_get_celsius(temp_handle, &temp_cpu); //lectura temperatura cpu

    //ESP_LOGI(TAG, "aaa %f, %lld, %lld", t_total, t_inicio, t_final);

    // tiempo de fin
    ciclo_final = ciclo_final + esp_cpu_get_cycle_count();
    t_final = esp_timer_get_time();

    //transformacion de datos
    ciclos_total = ciclo_final - ciclo_inicio;
    t_total = (t_final - t_inicio)/1e6;

    printLocalTime(); // muestra la hora
    ESP_LOGI(TAG, "%lld - %lld", ciclo_final, ciclo_inicio);
    ESP_LOGI(TAG, "La frecuencia es %f H, se han hecho %lld ciclos, el tiempo de %f, la temperatura es de %f ºC la luminosidad de %d %u %u\n", (float)(ciclos_total/t_total), ciclos_total, (float)(t_total), (float)(temp_cpu), (int)(r[0]*256 + r[1]), (unsigned int)(r[0]), (unsigned int)(r[1]));
    const char *file_mediciones = MOUNT_POINT"/datos.txt"; //(!)
    char data[EXAMPLE_MAX_CHAR_SIZE];
    snprintf(data, EXAMPLE_MAX_CHAR_SIZE, "%s: %.0fHz, %lldCcl, %.3fseg, %.2fC cpu, %dlm, %.1fHum, %.1fC\n", strftime_buf, (float)(ciclos_total/t_total), ciclos_total, (float)(t_total), (float)(temp_cpu), (int)(r[0]*256 + r[1]), humedad, temperatura);  
    ret = s_example_write_file(file_mediciones, data);
    if (ret != ESP_OK) {
        return;
    }
}

void w_separador(char *mensaje){
    const char *file_mediciones = MOUNT_POINT"/datos.txt"; //(!)
    char data[EXAMPLE_MAX_CHAR_SIZE];
    //memset(data, 0, EXAMPLE_MAX_CHAR_SIZE);
    //snprintf(data, EXAMPLE_MAX_CHAR_SIZE, " %fHz, %ldCcl, %fseg, %fºC, cpu, %dlm, %u %u\n", (float)(ciclos_total/t_total), (int32_t)(ciclos_total), (float)(t_total), (float)(temp_cpu), (int)(r[0]*256 + r[1]), (unsigned int)(r[0]), (unsigned int)(r[1]));
    snprintf(data, EXAMPLE_MAX_CHAR_SIZE, "\n%s\n", mensaje);  
    ret = s_example_write_file(file_mediciones, data);
    if (ret != ESP_OK) {
        return;
    }
}

/*static void monitoreo(void *arg)
{
    esp_err_t ret = arg;//a

    while (numero_medidas < max_medidas){
    
        // realiza las operaciones necesarias para tomar los datos
        esp_cpu_set_cycle_count(0);

        // leer datos de tiempo al inicio
        uint32_t ciclo_inicio = esp_cpu_get_cycle_count(); // 0
        int64_t t_inicio = esp_timer_get_time();

        vTaskDelay(pdMS_TO_TICKS(10000)); //15 segundos

        i2c_master_write_read_device(I2C_NUM_0, 0x23, &read_command, 1, r, 2, pdMS_TO_TICKS(180));// lectura continua luminosidad

        temperature_sensor_get_celsius(temp_handle, &temp_cpu); //lectura temperatura cpu

        // tiempo de fin
        ciclo_final = esp_cpu_get_cycle_count();
        int64_t t_final = esp_timer_get_time();

        //transformacion de datos
        int32_t ciclos_total = ciclo_final - ciclo_inicio;
        float t_total = (t_final - t_inicio)/1e6;

        printLocalTime(); // muestra la hora
        ESP_LOGI(TAG, "La frecuencia es %f H, se han hecho %"PRId32" ciclos, el tiempo de %f, la temperatura es de %f ºC la luminosidad de %d %u %u\n", (float)(ciclos_total/t_total), (int32_t)(ciclos_total), (float)(t_total), (float)(temp_cpu), (int)(r[0]*256 + r[1]), (unsigned int)(r[0]), (unsigned int)(r[1]));
        ESP_LOGI(TAG, "%"PRId32" %"PRId32"",(int32_t)ciclo_inicio,(int32_t)ciclos_total );
        ESP_LOGI("EXTERNO: Humedad: %.1f %%\tTemperatura: %.1f °C\n", humedad, temperatura);
        const char *file_mediciones = MOUNT_POINT"/medcn1.txt";
        char data[EXAMPLE_MAX_CHAR_SIZE];
        snprintf(data, EXAMPLE_MAX_CHAR_SIZE, "%s", strftime_buf); 
        ret = s_example_write_file(file_mediciones, data);
        if (ret != ESP_OK) {
            return;
        }

        memset(data, 0, EXAMPLE_MAX_CHAR_SIZE);
        snprintf(data, EXAMPLE_MAX_CHAR_SIZE, " %f %ld %f %f %d %u %u\n", (float)(ciclos_total/t_total), (int32_t)(ciclos_total), (float)(t_total), (float)(temp_cpu), (int)(r[0]*256 + r[1]), (unsigned int)(r[0]), (unsigned int)(r[1])); 
        ret = s_example_write_file(file_mediciones, data);
        if (ret != ESP_OK) {
            return;
        }

        numero_medidas++;
    }
    vTaskDelete(NULL);
}*/

//cambios hechos en skconfig:
//  CONFIG_PM_ENABLE
void app_main(void)
{
    

    
    /*// inicializacion temperatura
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0); 
    vTaskDelay(pdMS_TO_TICKS(18));

    gpio_set_direction(pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);

    int age;
    int startTime =  esp_timer_get_time();

    do {
      //ESP_LOGI(TAG, "loop");
      age = (int)(esp_timer_get_time() - startTime);
      
      ESP_LOGI(TAG, "init low us: %d", age);
    }
    while (gpio_get_level(pin) == 0);

    do {
      //ESP_LOGI(TAG, "loop");
      age = (int)(esp_timer_get_time() - startTime);
      
      ESP_LOGI(TAG, "init low us: %d", age);
    }
    while (gpio_get_level(pin) == 1);

    for ( int8_t i = -3 ; i < 2 * 40; i++ ) { //-3

    //esperando senal 
    do {
      //ESP_LOGI(TAG, "loop");
      age = (int)(esp_timer_get_time() - startTime);
      if ( age > 90 ) {
        ESP_LOGI(TAG, "UuuuuuuuuuuuSps %d", age);
        return;
      }
      int expected = (i & 1) ? 1 : 0; //lee el ultimo bit de i
      ESP_LOGI(TAG, "%d comparado con %d %d %d", gpio_get_level(pin), expected, gpio_get_level(pin) == expected, age);
    }
    while (gpio_get_level(pin) == expected);

    if ( i >= 0 && (i & 1) ) {
      // Now we are being fed our 40 bits
      //0
      ESP_LOGI(TAG, "age: %d", age);
      data_temp <<= 1;

      // A zero max 30 usecs, a one at least 68 usecs.
      if ( age > 30 ) {
        ESP_LOGI(TAG, "1");
        data_temp |= 1; // we got a one
      }
    }

    switch ( i ) {
      case 31:
        rawHumidity = data_temp;
        ESP_LOGI(TAG, "humedad: %d", rawHumidity);
        break;
      case 63:
        rawTemperature = data_temp;
        ESP_LOGI(TAG, "temperatura: %d", rawTemperature);
        data_temp = 0;
        break;
    }
  }*/
    

    

    //### INIT ###
    // sensor temperatura
    xTaskCreate(&dht_task, "dht_task", 2048, NULL, 5, NULL);
    // establecer los valores base del cpu
    //(!)
    configuracion_frecuencia(160);
    end_benchmark = true;

    // configurar el sensor de luz
    config_luz.mode = I2C_MODE_MASTER;
    config_luz.scl_io_num = GPIO_NUM_22;
    config_luz.sda_io_num = GPIO_NUM_21;
    config_luz.master.clk_speed = 100000;
    config_luz.scl_pullup_en = 1;
    config_luz.sda_pullup_en = 1;
    config_luz.clk_flags = 0;
    i2c_param_config(I2C_NUM_0, &config_luz);
    i2c_driver_install(I2C_NUM_0,I2C_MODE_MASTER,0,0,0);
    
    i2c_master_write_to_device(I2C_NUM_0, 0x23, &w, 1, pdMS_TO_TICKS(180));// encendido sensor luz

    //- intento tiemp
    //configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    //gettimeofday();

    //- intento wifi
    //wifi2
    //nvs_flash_init();
    //wifi_connection();

    //esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    //esp_sntp_setservername(0, "pool.ntp.org");
    //esp_sntp_init();

    //esp_netif_sntp_init(&config_tiempo);

    //if (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(1000)) != ESP_OK) {
    //printf("Failed to update system time within 1s timeout");
    //}

    // tiempo (establecido de forma manual)
    //(!)
    struct tm tm;
    tm.tm_year = 2025 - 1900;
    tm.tm_mon = 6;
    tm.tm_mday = 18;
    tm.tm_hour = 9;
    tm.tm_min = 30;
    tm.tm_sec = 0;
    time_t t = mktime(&tm);
    printf("Setting time: %s", asctime(&tm));
    struct timeval now2 = { .tv_sec = t };
    settimeofday(&now2, NULL);

    // init del sensor de temperatura interno
    temperature_sensor_install(&temp_sensor_config, &temp_handle);
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_handle));

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

    
    //fin guarreria-------

    // Genera una ventana de Hann - el benchmark
    dsps_wind_hann_f32(wind, N);

    ESP_LOGI(TAG, "*** COMIENZO ***");

    //xTaskCreatePinnedToCore(monitoreo, "tarea_monitoreo", 4096, (void*)ret, 2, NULL, tskNO_AFFINITY);

    // tomas multiples casos - 3 iteraciones
    /*for (int i = 0; i < 3; i++) //repeticiones proceso
    {
        for (int j = 0; j < 3; j++) // cada una de las pruebas 
        {
            xTaskCreate(&benchmark, "carga", 2048, NULL, 1, NULL);
    
            while (numero_medidas < max_medidas){

                // realiza las operaciones necesarias para tomar los datos
                esp_cpu_set_cycle_count(0);

                // leer datos de tiempo al inicio
                ciclo_inicio = esp_cpu_get_cycle_count(); // 0
                t_inicio = esp_timer_get_time();
                t_total = 0;
                

                //wait
                while (t_total < delay_medidas){
                    t_final = esp_timer_get_time();
                    t_total = (t_final - t_inicio)/1e6;
                }
                tomar_medidas();

                numero_medidas++;

            }

            end_benchmark = true;
            numero_medidas = 0;
            w_separador();

            switch (j)
            {
            case 0:
                //prepara caso 2
                end_benchmark = false;
                configuracion_frecuencia(80);
                ESP_LOGI(TAG, "prepara caso 2");
                break;
            
            case 1:
                //prepara caso 3
                end_benchmark = false;
                configuracion_frecuencia(160);
                ESP_LOGI(TAG, "prepara caso 3");
                break;

            case 2:
                //para y prepara caso 1
                vTaskDelay(pdMS_TO_TICKS(20000));//1200000
                end_benchmark = true;
                configuracion_frecuencia(160);
                ESP_LOGI(TAG, "prepara caso 1");
                break;
            
            default:
                ESP_LOGI(TAG, "ALGO VA MAL%d", i);
                break;
            }
        }
    }*/

    /*flag_ccl_tracker = true;
    xTaskCreate(&ccl_tracker, "conteo de ciclos", 2048, NULL, 1, NULL);

    while (numero_medidas < max_medidas){

        // realiza las operaciones necesarias para tomar los datos
        esp_cpu_set_cycle_count(0);

        // leer datos de tiempo al inicio
        ciclo_inicio = esp_cpu_get_cycle_count(); // 0
        t_inicio = esp_timer_get_time();
        t_total = 0;
        
        flag_ccl_tracker = false;
        
        //wait
        while (t_total < delay_medidas){
            t_final = esp_timer_get_time();
            t_total = (t_final - t_inicio)/1e6;
        }
        
        flag_ccl_tracker = true;
        tomar_medidas();
        ciclo_final = 0;
        //w_separador("probando separador");

        numero_medidas++;
        //delay_medidas = delay_medidas + 10;
        //vTaskDelay(pdMS_TO_TICKS(1200000));

    }

    end_ccl_tracker = true;
    end_benchmark = true;
    numero_medidas = 0;
    //w_separador();*/

    /*flag_ccl_tracker = true;
    end_benchmark = false;
    xTaskCreate(&ccl_tracker, "conteo de ciclos", 2048, NULL, 1, NULL);
    w_separador("caso 1 - 160 MHz con carga");

    for (int i = 0; i < 1; i++) //repeticiones proceso
    {
        for (int j = 0; j < 3; j++) // cada una de las pruebas 
        {
            
            xTaskCreate(&benchmark, "carga", 2048, NULL, 1, NULL);
    
            while (numero_medidas < max_medidas){

                // realiza las operaciones necesarias para tomar los datos
                esp_cpu_set_cycle_count(0);

                // leer datos de tiempo al inicio
                ciclo_inicio = esp_cpu_get_cycle_count(); // 0
                t_inicio = esp_timer_get_time();
                t_total = 0;
                

                //wait
                flag_ccl_tracker = false;
                while (t_total < delay_medidas){
                    t_final = esp_timer_get_time();
                    t_total = (t_final - t_inicio)/1e6;
                }
                flag_ccl_tracker = true;
                tomar_medidas();
                ciclo_final = 0;
                numero_medidas++;

            }

            end_benchmark = true;
            numero_medidas = 0;

            switch (j)
            {
            case 0:
                //prepara caso 2
                w_separador("caso 2 - 80 MHz con carga");
                max_medidas = 6 * 20;
                end_benchmark = false;
                configuracion_frecuencia(80);
                ESP_LOGI(TAG, "prepara caso 2");
                break;
            
            case 1:
                //prepara caso 3
                w_separador("caso 3 - 80 MHz sin carga");
                end_benchmark = true;
                configuracion_frecuencia(80);
                ESP_LOGI(TAG, "prepara caso 3");
                break;

            case 2:
                w_separador("FIN");
                //prepara caso 1
                end_benchmark = false;
                configuracion_frecuencia(160);
                ESP_LOGI(TAG, "prepara caso 1");
                break;
            
            default:
                ESP_LOGI(TAG, "ALGO VA MAL%d", i);
                break;
            }
        }
    }*/

    end_benchmark = false;
    xTaskCreate(&ccl_tracker, "conteo de ciclos", 2048, NULL, 1, NULL);
    w_separador("ANALISIS DIARIO DEL CLIMA: SIN CARGA 160MHz");

    while (numero_medidas < max_medidas){

        // realiza las operaciones necesarias para tomar los datos
        esp_cpu_set_cycle_count(0);

        // leer datos de tiempo al inicio
        ciclo_inicio = esp_cpu_get_cycle_count(); // 0
        t_inicio = esp_timer_get_time();
        t_total = 0;
        

        //wait
        flag_ccl_tracker = false;
        while (t_total < delay_medidas){
            t_final = esp_timer_get_time();
            t_total = (t_final - t_inicio)/1e6;
        }
        //vTaskDelay(pdMS_TO_TICKS(delay_medidas * 1000));
        
        flag_ccl_tracker = true;
        tomar_medidas();
        ciclo_final = 0;
        numero_medidas++;

    }

    end_benchmark = true;
    configuracion_frecuencia(160);
    end_ccl_tracker = true;
    end_dht= true;

    temperature_sensor_disable(temp_handle);

    //----
    
    //Open file for reading
    //ret = s_example_read_file(file_foo);
    //if (ret != ESP_OK) {
    //    return;
    //}

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


