#ifndef _SCALE_A8_DRIVERS_H_
#define _SCALE_A8_DRIVERS_H_

#include <linux/ioctl.h> /* needed for the _IOW etc stuff used later */

// #define _NUVOTON_BOARD_

typedef	unsigned char	KEY_CODE;

typedef struct gpio_cfg
{
    unsigned int pin_num;
    unsigned char pin_state;
} gpio_cfg;

/* magic number */
/* Please use a different 8-bit number in your code */
#define SCALE_MAGIC  251

//FOR AD IOCTL
#define SCALE_ADC_IOC_RESET		_IO(SCALE_MAGIC, 0x40)

//FOR LED IOCTL
#define SCALE_LED_IOC_RESET    	_IOW(SCALE_MAGIC, 0x60, long)

//FOR KEY IOCTL
#define SCALE_KEY_IOC_INIT    _IO(SCALE_MAGIC, 0x80)
#define SCALE_KEY_IOC_ID      _IOR(SCALE_MAGIC, 0x81, int)

//FOR GPIO
#define SCALE_GPIO_IOC_RESET    		_IO(SCALE_MAGIC, 0xa0)
#define SCALE_GPIO_IOC_INPUT      	 	_IOWR(SCALE_MAGIC, 0xa1, unsigned long)
#define SCALE_GPIO_IOC_OUTPUT_LOW       _IOW(SCALE_MAGIC, 0xa2, unsigned long)
#define SCALE_GPIO_IOC_OUTPUT_HIGHT     _IOW(SCALE_MAGIC, 0xa3, unsigned long)

//FOR TRACE
#define SCALE_GPIO_IOC_TRACE_INIT       _IOW(SCALE_MAGIC, 0xb0, unsigned long)
#define SCALE_GPIO_IOC_TRACE_DELAY      _IOW(SCALE_MAGIC, 0xb1, unsigned long)

//FOR DETECT
#define SCALE_GPIO_IOC_DETECT_INIT      _IOW(SCALE_MAGIC, 0xb2, unsigned long)
#define SCALE_GPIO_IOC_DETECT_REINIT    _IOW(SCALE_MAGIC, 0xb3, unsigned long)

//FOR set PD12 from PWM to GPIO 
#define SCALE_GPIO_PD12_INIT      		_IOW(SCALE_MAGIC, 0xb4, unsigned long)

#define SCALE_IOC_MAXNR 0xF0

/* Convert GPIO signal to GPIO pin number */
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

//AD PIN
#ifdef _NUVOTON_BOARD_
#define SCALE_AD_SCLK		GPIO_TO_PIN(3,15)
#define SCALE_AD_SDO		GPIO_TO_PIN(3,13)
#define SCALE_AD_PWR		GPIO_TO_PIN(3,11)
#elif defined _RASPBERRY_PI_
#define SCALE_AD_SCLK		GPIO_TO_PIN(3,15)
#define SCALE_AD_SDO		GPIO_TO_PIN(3,13)
#define SCALE_AD_PWR		GPIO_TO_PIN(3,11)
#else
#define SCALE_AD_SCLK		GPIO_TO_PIN(7,7)	//PH7-231
#define SCALE_AD_SDO		GPIO_TO_PIN(7,6)	//PH6-230
#define SCALE_AD_PWR		GPIO_TO_PIN(7,5)	//PH5-229
#endif

//LED PIN
#ifdef _NUVOTON_BOARD_
#define SCALE_LED_SCL0  GPIO_TO_PIN(2, 0)       
#define SCALE_LED_SCL1  GPIO_TO_PIN(1, 30)      
#define SCALE_LED_SDA0  GPIO_TO_PIN(1, 31)      
#define SCALE_LED_SDA1  GPIO_TO_PIN(2, 1)       
#elif defined _RASPBERRY_PI_
#define SCALE_LED_SCL0  GPIO_TO_PIN(2, 0)       
#define SCALE_LED_SCL1  GPIO_TO_PIN(1, 30)      
#define SCALE_LED_SDA0  GPIO_TO_PIN(1, 31)      
#define SCALE_LED_SDA1  GPIO_TO_PIN(2, 1)       
#else
#define SCALE_LED_SCL0  GPIO_TO_PIN(0, 12)       //PA12
#define SCALE_LED_SCL1  GPIO_TO_PIN(0, 14)       //PA14
#define SCALE_LED_SDA0  GPIO_TO_PIN(0, 13)       //PA13
#define SCALE_LED_SDA1  GPIO_TO_PIN(0, 15)       //PA15
#endif

//KEY PIN
#ifdef _NUVOTON_BOARD_
#define SCALE_KEY_INT  GPIO_TO_PIN(7, 6)        //PH6
#define SCALE_KEY_ST   GPIO_TO_PIN(7, 4)        //PH4
#define SCALE_KEY_SDA  GPIO_TO_PIN(7, 10)       //PH10
#define SCALE_KEY_SCL  GPIO_TO_PIN(7, 8)        //PH8
#elif defined _RASPBERRY_PI_
#define SCALE_KEY_INT  GPIO_TO_PIN(7, 6)        
#define SCALE_KEY_ST   GPIO_TO_PIN(7, 12)       
#define SCALE_KEY_SDA  GPIO_TO_PIN(7, 10)       
#define SCALE_KEY_SCL  GPIO_TO_PIN(7, 8)        
#else
#define SCALE_KEY_INT  GPIO_TO_PIN(8, 1)        //PI1-257
#define SCALE_KEY_ST   GPIO_TO_PIN(8, 2)        //PI2-258
#define SCALE_KEY_SDA  GPIO_TO_PIN(8, 4)        //PI4-260
#define SCALE_KEY_SCL  GPIO_TO_PIN(8, 3)        //PI3-259
#endif

//GPIO PIN
#define GPIO_PIN_SPEAKER             0   //蜂鸣器
#define GPIO_PIN_CALIBRATION         1   //标定
#define GPIO_PIN_PRINTER_STATUS      2   //打印机状态引脚
#define GPIO_PIN_PRINTER_POWER       3   //打印机电源引脚
#define GPIO_PIN_RUN_LED             4   //运行指示灯

#define GPIO_PIN_RFID_RESET          5   //RFID
#define GPIO_PIN_RFID_POWER          6

#define GPIO_PIN_ZIGBEE_RESET        7   //ZIGBEE
#define GPIO_PIN_ZIGBEE_CONFIG       8
#define GPIO_PIN_ZIGBEE_SLEEP        9

#define GPIO_PIN_BLE_POWER           10
#define GPIO_PIN_BLE_STATE           11


#ifdef _NUVOTON_BOARD_
#define SCALE_PIN_SPEAKER             GPIO_TO_PIN(8,7)   //PH14  蜂鸣器
#define SCALE_PIN_CALIBRATION         GPIO_TO_PIN(8,7)   //PD11 标定
#define SCALE_PIN_PRINTER_STATUS      GPIO_TO_PIN(8,7)   //PD13 打印机状态引脚
#define SCALE_PIN_PRINTER_POWER       GPIO_TO_PIN(8,7)   //PD15 打印机电源引脚
#define SCALE_PIN_RUN_LED             GPIO_TO_PIN(8,7)    //PD0  运行指示灯
#elif defined _RASPBERRY_PI_
#define SCALE_PIN_SPEAKER             GPIO_TO_PIN(3,12)   //PD12 蜂鸣器
#define SCALE_PIN_CALIBRATION         GPIO_TO_PIN(5,13)   //PF13 标定
#define SCALE_PIN_PRINTER_STATUS      GPIO_TO_PIN(1,2)    //PB2  打印机状态引脚
#define SCALE_PIN_PRINTER_POWER       GPIO_TO_PIN(8,12)   //PI12 打印机电源引脚
#define SCALE_PIN_RUN_LED             GPIO_TO_PIN(3,0)    //PD0  运行指示灯
#else
// #define SCALE_PIN_SPEAKER             GPIO_TO_PIN(2,14)   //PC14-78  蜂鸣器 V1.0 NAND conflict
#define SCALE_PIN_SPEAKER             GPIO_TO_PIN(6,0)    //PG0 BEEP
#define SCALE_PIN_CALIBRATION         GPIO_TO_PIN(1,9)   //PF13-173 标定
#define SCALE_PIN_PRINTER_STATUS      GPIO_TO_PIN(1,10)    //PB2-34   打印机状态引脚
#define SCALE_PIN_PRINTER_POWER       GPIO_TO_PIN(6,11)    //PI8-264  打印机电源引脚
#define SCALE_PIN_RUN_LED             GPIO_TO_PIN(5,13)   //PA10-10  运行指示灯 NONE
#define SCALE_PIN_BLE_POWER           GPIO_TO_PIN(6,11)    //PG11 BLE Power
#define SCALE_PIN_BLE_STATE           GPIO_TO_PIN(6,12)    //PG12 BLE State
#endif

#define SCALE_PIN_RFID_RESET          GPIO_TO_PIN(1,19)   //RFID
#define SCALE_PIN_RFID_POWER          GPIO_TO_PIN(1,21)

#define SCALE_PIN_ZIGBEE_RESET        GPIO_TO_PIN(3,19)   //ZIGBEE
#define SCALE_PIN_ZIGBEE_CONFIG       GPIO_TO_PIN(1,20)
#define SCALE_PIN_ZIGBEE_SLEEP        GPIO_TO_PIN(0,20)



#endif /* _SCALE_A8_DRIVERS_H_ */
