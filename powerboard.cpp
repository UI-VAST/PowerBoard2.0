#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/divider.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include <string>
#include <cstring> 
#include <cmath>

#include "BME280_driver/bme280.h"
#include "BME280_driver/bme280_defs.h" 
#include "BME280_driver/bme280.c"
#include "sic45x_driver/sic45x.h"

#include "ff.h"
#include "sd_card.h"
#include "hw_config.c"

#include "TinyGPSPlus.h"
#include "CRC.h"

#define UART_ID uart1
#define BAUD_RATE 9600

#define UART_TX_PIN 4
#define UART_RX_PIN 5

#define flg0 8
#define flg1 10
#define flg2 12
#define flg3 14

#define en0 9
#define en1 11
#define en2 13
#define en3 15

#define I2C_PORT i2c0
#define I2C_SDA 0
#define I2C_SCL 1

#define sic45xaddr 28
#define bme280addr 119
//Gps setup
#define gps_tx 24
#define gps_rx 25
#define gps_baud 9600
//Lora setup
#define lora_tx 4
#define lora_rx 5
#define lora_baud 57600


TinyGPSPlus gps;

typedef struct packet {
  double lat;
  double lng;
  float alt;
  float speed;
  int sats;
  int packetcount;
  int cutdown_time;
  bool timer_running;
  bool cutdown_status;
  bool parachute_status;
  int lora_bad_packet;
  int rfd_bad_packet;
} packet;
packet myPacket;

/* Structure that contains identifier details used in example */
struct identifier
{
    /* Variable to hold device address */
    uint8_t dev_addr;

    /* Variable that contains file descriptor */
    int8_t fd;
};
struct bme280_dev dev;
struct identifier id;
struct bme280_data comp_data;

struct sic45x_data
{
    double Vin;
    double Vout;
    double Iin;
    double Iout;
    double Pin;
    double Pout;
    double temp;

    uint16_t VOUT_OV_FAULT_LIMIT;
    uint8_t VOUT_OV_FAULT_RESPONSE;
    uint16_t VOUT_OV_WARN_LIMIT;

    uint16_t VOUT_UV_WARN_LIMIT;
    uint16_t VOUT_UV_FAULT_LIMIT;
    uint8_t VOUT_UV_FAULT_RESPONSE;

    uint16_t IOUT_OC_FAULT_LIMIT;
    uint8_t IOUT_OC_FAULT_RESPONSE;
    uint16_t IOUT_OC_WARN_LIMIT;

    uint16_t OT_FAULT_LIMIT;
    uint8_t  OT_FAULT_RESPONSE;
    uint16_t OT_WARN_LIMIT;

    uint16_t VIN_OV_FAULT_LIMIT;
    uint8_t VIN_OV_FAULT_RESPONSE;
    uint16_t VIN_UV_WARN_LIMIT;

    uint16_t IIN_OC_WARN_LIMIT;

    uint8_t WRITE_PROTECT;
    uint8_t status_byte;
};
sic45x_data sic_data;

FRESULT fr;
FATFS fs;
FIL fil;
char filename[] = "log.csv";
unsigned long sec = 0;


void my_sd_init(){

    // Initialize SD card
    if (!sd_init_driver()) {
        printf("ERROR: Could not initialize SD card\r\n");
        while (true);
    }

    // Mount drive
    fr = f_mount(&fs, "0:", 1);
    if (fr != FR_OK) {
        printf("ERROR: Could not mount filesystem (%d)\r\n", fr);
        while (true);
    }

    // Open file for writing ()
    fr = f_open(&fil, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK) {
        printf("ERROR: Could not open file (%d)\r\n", fr);
    }

    // Write Header
    f_printf(&fil, "Presure, Temp, Hum, Volt, Vin, Iin, Vout, SicTemp\r\n");

    // Close file
    fr = f_close(&fil);
    if (fr != FR_OK) {
        printf("ERROR: Could not close file (%d)\r\n", fr);
    }

}

int data_log(struct bme280_data *comp_data, sic45x_data *sic_data) {

    float temp, press, hum;

    temp = comp_data->temperature;
    press = 0.01 * comp_data->pressure;
    hum = comp_data->humidity;

    float Vin = sic_data->Vin;
    float Iin = sic_data->Iin;
    float Vout = sic_data->Vout;
    float sictemp = sic_data->temp;
    // Open file for writing ()
  
    fr = f_open(&fil, filename, FA_WRITE | FA_OPEN_APPEND);
    if (fr != FR_OK) {
        printf("ERROR: Could not open file (%d)\r\n", fr);
    }

    //f_lseek(&fil, curpos);
    f_printf(&fil,"%f", press);
    f_printf(&fil,",");
    f_printf(&fil,"%f", temp);
    f_printf(&fil,",");
    f_printf(&fil,"%f", hum);
    f_printf(&fil,",");
    f_printf(&fil,"%f", Vin);
    f_printf(&fil,",");
    f_printf(&fil,"%f", Iin);
    f_printf(&fil,",");
    f_printf(&fil,"%f", Vout);
    f_printf(&fil,",");
    f_printf(&fil,"%f", sictemp);
    f_printf(&fil,"\r\n");


    
    // Close file
    fr = f_close(&fil);
    if (fr != FR_OK) {
        printf("ERROR: Could not close file (%d)\r\n", fr);
    }
    

    // Unmount drive
    //f_unmount("0:");
    

    return 0;
}

int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr){
    uint8_t ret;
    ret = i2c_write_blocking(i2c0, bme280addr, &reg_addr, 1,true);
    ret = i2c_read_blocking(i2c0, bme280addr, data, len, false);
    return 0;
}

void user_delay_us(uint32_t period, void *intf_ptr){
    sleep_us(period);
}

int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr){
    uint8_t ret;
    uint8_t tx_len = len + 1;
    uint8_t tx_buf[tx_len];
    tx_buf[0]  = reg_addr;
    
    memcpy(&tx_buf[1], data, len);
    ret = i2c_write_blocking(i2c0, bme280addr, tx_buf, tx_len, false);
    return 0;

}

void print_sensor_data(struct bme280_data *comp_data)
{
    float temp, press, hum;

    temp = comp_data->temperature;
    press = 0.01 * comp_data->pressure;
    hum = comp_data->humidity;

    printf("%0.2lf deg C, %0.2lf hPa, %0.2lf%%\n", temp, press, hum);
}

int8_t stream_sensor_data_forced_mode(struct bme280_dev *dev)
{
    uint8_t rslt = BME280_OK;
    struct bme280_data comp_data;

    /* Continuously stream sensor data */
    while (1)
    {
        rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
        if (rslt != BME280_OK)
        {
            fprintf(stderr, "Failed to set sensor mode (code %+d).", rslt);
            break;
        }

        /* Wait for the measurement to complete and print data @25Hz */
        dev->delay_us(40000, dev->intf_ptr);
        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
        if (rslt != BME280_OK)
        {
            fprintf(stderr, "Failed to get sensor data (code %+d).", rslt);
            break;
        }
        
        print_sensor_data(&comp_data);
        dev->delay_us(1000000, dev->intf_ptr);
    }

    return rslt;
}


int64_t alarm_callback(alarm_id_t id, void *user_data) {
    // Put your timeout handler code in here
    return 0;
}

double parseLinear11(uint16_t rawValue) {
	int16_t mantissa = rawValue & 1023;
	uint8_t mantissaSign = (rawValue >> 10) & 1;

	int8_t  exponent = (rawValue >> 11) & 0b1111;
	uint8_t exponentSign = rawValue >> 15;

	if (exponentSign) {
		exponent = -1 * (exponent ^ 0b1111) - 1;
	}

	if (mantissaSign) {
		mantissa = -1 * (mantissa ^ 1023) - 1;
	}

	return pow(2, exponent) * (double) mantissa;
}

double parseLinear16(uint16_t rawValue){
    return pow(2, -9) * double(rawValue);
}

uint16_t encodeLinear11(double rawValue){
    
    
    //1111100000011110
    //1111100000100000
    return 0;
}

uint8_t get8bitreg(uint8_t addr){
    uint8_t rxdata;
    i2c_write_blocking(i2c0, sic45xaddr, &addr, 1, true);
    i2c_read_blocking(i2c0, sic45xaddr, &rxdata, 1, false);
    return rxdata;
}

uint16_t get16bitreg(uint8_t addr){
    uint8_t rxdata[1];
    i2c_write_blocking(i2c0, sic45xaddr, &addr, 1, true);      
    i2c_read_blocking(i2c0, sic45xaddr, &rxdata[0], 2, false);
    uint16_t rawValue = (rxdata[1] << 8) + rxdata[0];
    return rawValue;
}

void write8bitreg(uint8_t addr, uint8_t data){
    i2c_write_blocking(i2c0, sic45xaddr, &addr, 1, true);
    i2c_write_blocking(i2c0, sic45xaddr, &data, 1, false);
}

void write16bitreg(uint8_t addr, uint16_t data){
    uint8_t ret;
    uint8_t tx_buf[2];
    tx_buf[0] = addr;
    tx_buf[1] = uint8_t(data);
    tx_buf[2] = uint8_t(data >> 8);
    ret = i2c_write_blocking(i2c0, sic45xaddr, tx_buf, 3, false);
}

void get_sic_data(sic45x_data* sic_data){
    //Voltage
    sic_data->Vin = parseLinear11(get16bitreg(SIC45X_READ_VIN_ADDR));
    double vout = get16bitreg(SIC45X_READ_VOUT_ADDR);
    vout = pow(2, -9) * vout;
    sic_data->Vout = vout;

    //Current
    sic_data->Iin = parseLinear11(get16bitreg(SIC45X_READ_IIN_ADDR));
    sic_data->Iout = parseLinear11(get16bitreg(SIC45X_READ_IOUT_ADDR));
    //Power
    sic_data->Pin = parseLinear11(get16bitreg(SIC45X_READ_PIN_ADDR));
    sic_data->Pout = parseLinear11(get16bitreg(SIC45X_READ_POUT_ADDR));
    //Temp
    sic_data->temp = parseLinear11(get16bitreg(SIC45X_READ_TEMPERATURE_ADDR));
    //Status Byte
    sic_data->status_byte = get8bitreg(SIC45X_STATUS_BYTE_ADDR);

    //Vin
    sic_data->VIN_OV_FAULT_LIMIT = get16bitreg(SIC45X_VIN_OV_FAULT_LIMIT_ADDR);
    sic_data->VIN_OV_FAULT_RESPONSE = get8bitreg(SIC45X_VIN_OV_FAULT_RESPONSE_ADDR);
    sic_data->VIN_UV_WARN_LIMIT = get16bitreg(SIC45X_VIN_UV_WARN_LIMIT_ADDR);
    
    //Vout OV
    sic_data->VOUT_OV_FAULT_LIMIT = get16bitreg(SIC45X_VOUT_OV_FAULT_LIMIT_ADDR);
    sic_data->VOUT_OV_WARN_LIMIT = get16bitreg(SIC45X_VOUT_OV_WARN_LIMIT_ADDR);
    sic_data->VOUT_OV_FAULT_RESPONSE = get8bitreg(SIC45X_VOUT_OV_FAULT_RESPONSE_ADDR);
    //Vout UV
    sic_data->VOUT_UV_FAULT_LIMIT = get16bitreg(SIC45X_VOUT_UV_FAULT_LIMIT_ADDR);
    sic_data->VOUT_UV_WARN_LIMIT = get16bitreg(SIC45X_VOUT_UV_WARN_LIMIT_ADDR);
    sic_data->VOUT_UV_FAULT_RESPONSE = get8bitreg(SIC45X_VOUT_UV_FAULT_RESPONSE_ADDR);

    sic_data->WRITE_PROTECT = get8bitreg(SIC45X_WRITE_PROTECT_ADDR);
}

void sic_print(sic45x_data* sic_data){
    printf("Vin: %f \n", sic_data->Vin);
    printf("Vout: %f \n", sic_data->Vout);

    printf("Iin: %f \n", sic_data->Iin);
    printf("Iout: %f \n", sic_data->Iout);

    printf("Pin: %f \n", sic_data->Pin);
    printf("Pout: %f \n", sic_data->Pout);

    printf("Temp: %f \n", sic_data->temp);

    printf("Status: %b \n", sic_data->status_byte);
    //Vin
    printf("Vin OV Fault Limit: %f \n", parseLinear11(sic_data->VIN_OV_FAULT_LIMIT));
    printf("Vin OV Fault Response: %b \n", sic_data->VIN_OV_FAULT_RESPONSE);
    printf("Vin UV Warn Limit: %f \n", parseLinear11(sic_data->VIN_UV_WARN_LIMIT));
    //Vout
    printf("Vout OV Fault Limit: %f \n", parseLinear16(sic_data->VIN_OV_FAULT_LIMIT));
    printf("Vout OV Warn Limit: %f \n", parseLinear16(sic_data->VIN_OV_FAULT_LIMIT));
    printf("Vout OV Fault Response: %b \n", sic_data->VIN_OV_FAULT_RESPONSE);

    printf("Write Protet: %b \n", sic_data->WRITE_PROTECT);

    printf("\n");
}

void io_init(){

    stdio_init_all();
    //I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000); 
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    //gpio_pull_up(I2C_SDA);
    //gpio_pull_up(I2C_SCL);
    //bi_decl(bi_2pins_with_func(I2C_SDA, I2C_SCL, GPIO_FUNC_I2C));

    gpio_init(en0);
    gpio_init(en1);
    gpio_init(en2);
    gpio_init(en3);

    gpio_init(flg0);
    gpio_init(flg1);
    gpio_init(flg2);
    gpio_init(flg3);

    gpio_set_dir(en0, GPIO_OUT);
    gpio_set_dir(en1, GPIO_OUT);
    gpio_set_dir(en2, GPIO_OUT);
    gpio_set_dir(en3, GPIO_OUT);

    gpio_pull_up(flg0);
    gpio_pull_up(flg1);
    gpio_pull_up(flg2);
    gpio_pull_up(flg3);

    gpio_set_dir(flg0, GPIO_IN);
    gpio_set_dir(flg1, GPIO_IN);
    gpio_set_dir(flg2, GPIO_IN);
    gpio_set_dir(flg3, GPIO_IN);

    gpio_put(en0, 1);
    sleep_ms(500);
    gpio_put(en1, 1);
    sleep_ms(500);
    gpio_put(en2, 1);
    sleep_ms(500);
    gpio_put(en3, 1);

}

uint8_t my_bme280_init(){

    int8_t rslt = BME280_OK;
    id.dev_addr = BME280_I2C_ADDR_SEC;
    dev.intf = BME280_I2C_INTF;
    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_us = user_delay_us;
    dev.intf_ptr = &id;

    rslt = bme280_init(&dev);

    /* Recommended mode of operation: Indoor navigation */
    dev.settings.osr_h = BME280_OVERSAMPLING_1X;
    dev.settings.osr_p = BME280_OVERSAMPLING_16X;
    dev.settings.osr_t = BME280_OVERSAMPLING_2X;
    dev.settings.filter = BME280_FILTER_COEFF_16;

    uint8_t settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    rslt = bme280_set_sensor_settings(settings_sel, &dev);

    return rslt;
}

void my_sic45x_init(){
    uint16_t data = 0b1111100000101000; //20V
    write16bitreg(SIC45X_VIN_OV_FAULT_LIMIT_ADDR, data);
    
}

bool gps_init(){
    uart_init(uart1, gps_baud);
    gpio_set_function(gps_tx, GPIO_FUNC_UART);
    gpio_set_function(gps_rx, GPIO_FUNC_UART);
    return uart_is_enabled(uart1);
}

bool lora_init(){
    uart_init(uart0, lora_baud);
    gpio_set_function(lora_tx, GPIO_FUNC_UART);
    gpio_set_function(lora_rx, GPIO_FUNC_UART);
    return uart_is_enabled(uart1);
}

void Send_packet(){
    //myPacket.packetcount++;
    uint32_t crc = CRC::Calculate(&myPacket, sizeof(myPacket), CRC::CRC_32());
    uint8_t payload[sizeof(myPacket)+sizeof(crc)];
    memcpy(&payload, &myPacket, sizeof(myPacket));
    memcpy(&payload[sizeof(myPacket)], &crc, sizeof(crc));
    
    /*
    for(int i = 0; i < sizeof(payload); i++){
      Serial.print(i);
      Serial.print(" ");
      Serial.println(payload[i]);
    }
    
    Serial.print("packet size ");
    Serial.println(sizeof(myPacket));
    Serial.print("payload ");
    Serial.println(sizeof(payload));
    */
    //rfd_PacketSerial.send(&payload[0], sizeof(payload));
}

void read_gps(){
    if(uart_is_readable(uart1)){
        if(gps.encode(uart_getc(uart1))){
            printf("decoded gps \n");
            if(gps.location.isValid()){
                printf("GPS location is valid! \n");
            }
        }
        printf("uart recieved \n");
    }
    else{
        printf("no gps connected \n");
    }
}
int main()
{
    

    io_init();

    my_bme280_init();

    my_sd_init();

    my_sic45x_init();
    
    while(true){


        bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);

        get_sic_data(&sic_data);

        sic_print(&sic_data);

        bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);

        //print_sensor_data(&comp_data);

        data_log(&comp_data, &sic_data);
        



        sleep_ms(1000);

    }


    return 0;
}
