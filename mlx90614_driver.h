#ifndef MLX90614_H_
#define MLX90614_H_
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/types.h> 
#include <linux/ioctl.h>
#define MLX90614_DEFAULT_ADDRESS 			((0x5a)<<1)
// RAM
#define MLX90614_RAWIR1 					0x04
#define MLX90614_RAWIR2 					0x05
#define MLX90614_TA 						0x06
#define MLX90614_TOBJ1 						0x07
#define MLX90614_TOBJ2 						0x08
// EEPROM
#define MLX90614_TOMAX 						0x00
#define MLX90614_TOMIN 						0x01
#define MLX90614_PWMCTRL 					0x02
#define MLX90614_TARANGE 					0x03
#define MLX90614_EMISS 						0x04
#define MLX90614_CONFIG 					0x05
#define MLX90614_ADDR 						0x0E
#define MLX90614_EMISS_CALIBRATION 				0x0F
#define MLX90614_ID1 						0x1C
#define MLX90614_ID2 						0x1D
#define MLX90614_ID3 						0x1E
#define MLX90614_ID4 						0x1F
#define MLX90614CMD_UNLOCK_EMISS_CALIBRATION 0x60
#define MLX90614_SLEEP_MODE 0xFF
#define MLX90614_DEVICE_NAME      "mlx90614_i2c_device"
#define MLX90614_GET_OBJ_TEMP_RAW_VALUE_ 	_IOR('a','b',int32_t*)
#define MLX90614_GET_AMB_TEMP_RAW_VALUE_ 	_IOR('a','c',int32_t*)
#define MLX90614_SET_PWM 					_IOW('a','d',int32_t*)
typedef enum
{
	MLX90614_OKAY 	= 0,
	MLX90614_ERROR	= 1
}mlx90614_status;
typedef struct {
    uint32_t                        id;                      // Chip ID
    uint16_t                       addr;                    // I2C address
    struct cdev                    cdev;                    // Character device structure
    struct class                   *dev_class;              // Device class
    struct i2c_client              *client;                 // I2C client
	uint16_t 					   object_temperature;
	uint16_t 					   ambient_temperature;
	uint16_t 					   emissivity;
	uint16_t					   df_calib;
	uint16_t 					   default_calib;
} mlx90614_device ;
#endif /* MLX90614_H_ */
