#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_log.h"
#include "driver/i2c.h"

#include "i2c_bus.h"

#include "wm8960.h"

/* R25 - Power 1 */
#define WM8960_VMID_MASK 0x180
#define WM8960_VREF      0x40

/* R26 - Power 2 */
#define WM8960_PWR2_LOUT1	0x40
#define WM8960_PWR2_ROUT1	0x20
#define WM8960_PWR2_OUT3	0x02

/* R28 - Anti-pop 1 */
#define WM8960_POBCTRL   0x80
#define WM8960_BUFDCOPEN 0x10
#define WM8960_BUFIOEN   0x08
#define WM8960_SOFT_ST   0x04
#define WM8960_HPSTBY    0x01

/* R29 - Anti-pop 2 */
#define WM8960_DISOP     0x40
#define WM8960_DRES_MASK 0x30

static const char *TAG = "WM8960";

const char* audio_nvs_nm = "audio";
const char* vol_nvs_key = "volume";

/* Forward Declarations for static functions */
/**
  * @brief  Write register of WM8960.
  * @param  reg: The number of register which to be read.
  * @retval The value of register.
  */
static esp_err_t wm8960_write_register(i2c_dev_t *dev, uint8_t reg_addr, uint16_t reg_val);

/**
  * @brief  Read register of WM8960.
  * @param  reg: The number of register which to be read.
  * @retval esp_err_t.
  */
static esp_err_t wm8960_read_register(i2c_dev_t *dev, uint8_t reg_addr);

/*
 * wm8960 register cache
 * We can't read the WM8960 register space when we are
 * using 2 wire for device control, so we cache them instead.
 */
static uint16_t wm8960_reg_val[56] = {
    0x0097, 0x0097, 0x0000, 0x0000, 0x0000, 0x0008, 0x0000, 0x000A,
    0x01C0, 0x0000, 0x00FF, 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x007B, 0x0100, 0x0032, 0x0000, 0x00C3, 0x00C3, 0x01C0,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0100, 0x0100, 0x0050, 0x0050, 0x0050, 0x0050, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0040, 0x0000, 0x0000, 0x0050, 0x0050, 0x0000,
    0x0000, 0x0037, 0x004D, 0x0080, 0x0008, 0x0031, 0x0026, 0x00ED
};

esp_err_t wm8960_init(i2c_dev_t *dev){
    esp_err_t ret = 0;

    // Reset Wm8960
    ret = wm8960_write_register(dev, 0x0F, 0x0000);
    if(ret != ESP_OK)
        return ret;
    else
        ESP_LOGI(TAG, "reset completed");

    ret =  wm8960_write_register(dev, 0x19, 1<<8 | 1<<7 | 1<<6);                //Enable main volTAGe references
    ret |= wm8960_write_register(dev, 0x1A, 1<<8 | 1<<7 | 1<<6 | 1<<5 | 1<<0);  //Enable DAC & output peripherals + PLL
    ret |= wm8960_write_register(dev, 0x2F, 1<<3 | 1<<2);                       //Enable mixer outputs
    if(ret != ESP_OK)  {
        ESP_LOGI(TAG, "power mngr. config failed");
        return ret;
    }
    
    //Configure SYSCLK
    /* SYSCLK = 12.288MHz
     * PLLin = 25/2
     * R = (4 x 2 x 12.288)/12.5MHz = 7.86432
     * PLLN = int(R) = 7
     * PLLK = 2^24 * (R - int(R)) = 2^24 * 0.86432 = 0xDD4413h
     */ 
    ret =  wm8960_write_register(dev, 0x34, 1<<5 | 1<<4 | 0x07); //PLL Fractional Mode, Prescale & PLLN=7
    uint32_t PLLK = 0xDD4413;
    ret |= wm8960_write_register(dev, 0x35, (PLLK >> 16) & 0xFF);
    ret |= wm8960_write_register(dev, 0x36, (PLLK >> 8) & 0xFF);
    ret |= wm8960_write_register(dev, 0x37, (PLLK) & 0xFF);
    if(ret != ESP_OK)  {
        ESP_LOGI(TAG, "SYSCLK config failed");
        return ret;
    }

    //Configure peripheral clocking
    ret = wm8960_write_register(dev, 0x04, 1<<2 | 1<<0);    //Set SYSCLKDIV to 2 & select PLL 
    ret |= wm8960_write_register(dev, 0x08, 1<<2);          //Set BCLKDIV to /4
    if(ret != ESP_OK)  {
        ESP_LOGI(TAG, "clocking config failed");
        return ret;
    }

    //Configure audio interface
    ret = wm8960_write_register(dev, 0x07, 0<<3 | 0<<2 | 1<<1 | 0<<0);   //I2S 16-bit Slave mode
    if(ret != ESP_OK)  {
        ESP_LOGI(TAG, "I2S config failed");
        return ret;
    }

    //Configure output routing
    ret = wm8960_write_register(dev, 0x17, 1<<1 | 1<<0);        //Enable Slowclock for volume update zero-cross detect
    ret |= wm8960_write_register(dev, 0x30, 1<<3 | 0<<2);       //Configure HP Switch
    ret |= wm8960_write_register(dev, 0x18, 1<<6 | 0<<5);       //Enable HP switch
    ret |= wm8960_write_register(dev, 0x22, 1<<8);              //LDAC -> Output Mixer
    ret |= wm8960_write_register(dev, 0x25, 1<<8);              //RDAC -> Output Mixer
    ret |= wm8960_write_register(dev, 0x02, 0x006F | 1<<8);     //LOUT1 (HP) Volume Set: -15dB
    ret |= wm8960_write_register(dev, 0x03, 0x006F | 1<<8);     //ROUT1 (HP) Volume Set: -15dB
    if(ret != ESP_OK)  {
        ESP_LOGI(TAG, "output routing failed");
        return ret;
    }

    //Configure output volume
    ret = wm8960_set_vol(dev, 0, true); //Init volume with value pre-shutdown
    ret |= wm8960_set_mute(dev, false); //Unmute DAC
    if(ret != ESP_OK)  {
        ESP_LOGI(TAG, "volume restoration failed");
        return ret;
    }

    if (ret == ESP_OK)
        ESP_LOGI(TAG, "init complete");
    return ret;
}

esp_err_t wm8960_set_mute(i2c_dev_t *dev, bool mute){
    esp_err_t ret = 0;
    if (mute) {
        ret |= wm8960_write_register(dev, 0x05, 1<<3);
    } else {
        ret |= wm8960_write_register(dev, 0x05, 0<<3);
    }
    return ret;
}

esp_err_t wm8960_enable_soft_mute(i2c_dev_t *dev, bool enable){
    esp_err_t ret = 0;
    if (enable) {
        ret |= wm8960_write_register(dev, 0x06, 1<<3);
    } else {
        ret |= wm8960_write_register(dev, 0x06, 0<<3);
    }
    return ret;
}

esp_err_t wm8960_set_volume(i2c_dev_t *dev, uint8_t vol, bool init){
    esp_err_t ret = 0;
    uint8_t vol_to_set = 0;
    nvs_handle_t audioStore;

    ret = nvs_open(audio_nvs_nm, NVS_READWRITE, &audioStore);

    if(init){
        ret |= nvs_get_u8(audioStore, vol_nvs_key, &vol_to_set);
    } else {
        if (vol == 0) {
            vol_to_set = 0;
        } else {
            ret |= nvs_set_u8(audioStore, vol_nvs_key, vol);
            vol_to_set = (vol/255) * 100;    //(vol / 10) * 5 + 200;
        }
    }
    
    ret |= wm8960_write_register(dev, 0x0A, vol_to_set);
    ret |= wm8960_write_register(dev, 0x0B, 0x100 | vol_to_set);

    ret |= nvs_commit(audioStore);
    nvs_close(audioStore);

    return ret;
}

esp_err_t wm8960_get_volume(uint8_t* vol){
    esp_err_t ret;
    nvs_handle_t audioStore;
    uint8_t volume = 0;

    ret = nvs_open(audio_nvs_nm, NVS_READONLY, &audioStore);
    ret |= nvs_get_u8(audioStore, vol_nvs_key, &volume);
    nvs_close(audioStore);

    *vol = volume;
    return ret;
}

static esp_err_t wm8960_write_register(i2c_dev_t *dev, uint8_t reg_addr, uint16_t reg_val){
    esp_err_t ret;
    uint8_t buff[2];
    buff[0] = (reg_addr << 1) |(uint8_t) ((reg_val >> 8) & 0x01);
    buff[1] = (uint8_t) reg_val & 0xFF;
    ret = i2c_bus_write_data(dev->port, dev->addr, buff, 2);
    if(ret == ESP_OK)
        wm8960_reg_val[reg_addr] = reg_val;
    return ret;
}

static esp_err_t wm8960_read_register(i2c_dev_t *dev, uint8_t reg_addr){
    return wm8960_reg_val[reg_addr];
}