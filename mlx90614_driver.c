#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include<linux/slab.h>                 //kmalloc()
#include<linux/uaccess.h>              //copy_to/from_user()
#include <linux/ioctl.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include "mlx90614_driver.h"
/* Variables for pwm  */
struct pwm_device *pwm0 = NULL;
static unsigned int pwm_period_ns = 1000000; // Thời gian chu kỳ PWM (1 ms)
static unsigned int pwm_duty_ns = 0;
int32_t data_ioctl = 0;
int32_t value = 0;
dev_t dev = 0;
uint8_t *kernel_buffer;
mlx90614_status mlx90614_write_reg_value_8bit(mlx90614_device *mlx90614, uint8_t addr, uint8_t data) {
    uint8_t ret;
    ret = i2c_smbus_write_byte_data(mlx90614->client, addr, data);
    if (ret < 0) {
        return MLX90614_ERROR;  
    }
    return MLX90614_OKAY;  
}
mlx90614_status mlx90614_read_reg_value_16bit(mlx90614_device *mlx90614, uint8_t addr, uint16_t *data) {
    uint16_t ret = i2c_smbus_read_word_data(mlx90614->client, addr);
    if(ret < 0) {
        pr_info("Can not send data 16 bit to reg");
        return MLX90614_ERROR;
    }
    *data = ret;
    return MLX90614_OKAY;  
}
mlx90614_status mlx90614_read_reg_value_16bit_eprom(mlx90614_device *mlx90614, uint8_t addr, uint16_t *data) {
	uint16_t ret;
    addr &= 0x1F;
	addr |= 0x20;
    ret = i2c_smbus_read_word_data(mlx90614->client, addr);
    if(ret < 0) {
        pr_info("Can not send data 16 bit to reg");
        return MLX90614_ERROR;
    }
    *data = ret; 
    return MLX90614_OKAY;  
}
uint8_t cyclic_redundancy_check_8bit(mlx90614_device *mlx90614, uint8_t in_crc, uint8_t in_data) {
	uint8_t i;
	uint8_t Data = in_crc ^= in_data;
	for ( i = 0; i < 8; i++ ) {
		if (( Data & 0x80 ) != 0 ) {
			Data <<= 1;
			Data ^= 0x07;
		}
		else
			Data <<= 1;
	}
	return Data;
}
mlx90614_status mlx90614_write_16bit_crc(mlx90614_device *mlx90614, uint8_t addr, uint16_t value)
{
	uint8_t Crc;
	uint8_t tmp[3];
	uint8_t index;
	addr &= 0x1F;
	addr |= 0x20;
	Crc = cyclic_redundancy_check_8bit(mlx90614, 0, MLX90614_DEFAULT_ADDRESS);
	Crc = cyclic_redundancy_check_8bit(mlx90614, Crc, addr);
	Crc = cyclic_redundancy_check_8bit(mlx90614, Crc, value & 0xFF);
	Crc = cyclic_redundancy_check_8bit(mlx90614, Crc, value>>8);
	tmp[0] = value & 0xFF;
	tmp[1] = value>>8;
	tmp[2] = Crc;
	for(index = 1; index <= 3; index++)
		if (MLX90614_OKAY != mlx90614_write_reg_value_8bit(mlx90614, addr, tmp[index]))
			return MLX90614_ERROR;
	return MLX90614_OKAY;
}
mlx90614_status mlx90614_write_reg_value_16bit_eprom(mlx90614_device *mlx90614, uint8_t addr, uint16_t value) {
	uint16_t verify;
	if(MLX90614_OKAY != mlx90614_write_16bit_crc(mlx90614, addr, 0))
		return MLX90614_ERROR;
	if(MLX90614_OKAY != mlx90614_write_16bit_crc(mlx90614, addr, value))
		return MLX90614_ERROR;
	mlx90614_read_reg_value_16bit_eprom(mlx90614, addr, &verify);
	if(verify != value)
		return MLX90614_ERROR;
	return MLX90614_OKAY;
}
mlx90614_status mlx90614_read_ambient_temperature(mlx90614_device *mlx90614) {
	uint16_t data;
	if(MLX90614_OKAY != mlx90614_read_reg_value_16bit(mlx90614, MLX90614_TA, &data))
		return MLX90614_ERROR;
	mlx90614->ambient_temperature = (data);
	return MLX90614_OKAY;
}
mlx90614_status mlx90614_read_object_temperature(mlx90614_device *mlx90614) {
	uint16_t data;
	if(MLX90614_OKAY != mlx90614_read_reg_value_16bit(mlx90614, MLX90614_TOBJ1, &data))
		return MLX90614_ERROR;
	mlx90614->object_temperature = data;
	return MLX90614_OKAY;
}
mlx90614_status mlx90614_get_emissivity(mlx90614_device *mlx90614) {
	uint16_t data;
	if(MLX90614_OKAY != mlx90614_read_reg_value_16bit_eprom(mlx90614, MLX90614_EMISS, &data))
		return MLX90614_ERROR;
	mlx90614->emissivity = data;
	return MLX90614_OKAY;
}
mlx90614_status mlx90614_set_emissivity(mlx90614_device *mlx90614) {
	uint16_t current_emissivity;
	uint16_t current_calibration;
	uint16_t new_emissivity;
	uint16_t new_calibration;
	
    if (mlx90614->emissivity < 0.1 || mlx90614->emissivity > 1)
		return MLX90614_ERROR;
	mlx90614_write_reg_value_16bit_eprom(mlx90614, MLX90614_EMISS, current_emissivity);
	mlx90614_read_reg_value_16bit_eprom(mlx90614, MLX90614_EMISS_CALIBRATION, &current_calibration);
	new_emissivity = (uint16_t)(65535 * mlx90614->emissivity);
	if(new_emissivity == current_emissivity)
		return MLX90614_OKAY;
	new_calibration = (uint16_t)((current_emissivity / new_emissivity) * current_calibration);
	if(MLX90614_OKAY != mlx90614_write_reg_value_16bit_eprom(mlx90614, MLX90614_EMISS, new_emissivity))
		return MLX90614_ERROR;
	if(MLX90614_OKAY != mlx90614_write_reg_value_16bit_eprom(mlx90614, MLX90614_EMISS_CALIBRATION, new_calibration))
		return MLX90614_ERROR;
	return MLX90614_OKAY;
}
mlx90614_status mlx90614_reset_emissivity(mlx90614_device *mlx90614, uint16_t default_calib) {
	if(MLX90614_OKAY != mlx90614_write_reg_value_16bit_eprom(mlx90614, MLX90614_EMISS, 0xFFFF))
		return MLX90614_ERROR;
	if(MLX90614_OKAY != mlx90614_write_reg_value_16bit_eprom(mlx90614, MLX90614_EMISS_CALIBRATION, default_calib))
		return MLX90614_ERROR;
	return MLX90614_OKAY;
}
static int etx_open(struct inode *inode, struct file *file)
{
    mlx90614_device *mlx90614 = container_of(inode->i_cdev, mlx90614_device, cdev);
    // Ensure bmp280 is valid
    if (!mlx90614) {
        pr_err("mlx90614 not initialized\n");
        return -EFAULT;
    }
    file->private_data = mlx90614;
    pr_info("Device file opened successfully\n");
    return 0;
}
static long etx_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    mlx90614_device *mlx90614 = (mlx90614_device*)file->private_data;
    // struct i2c_client *client = mlx90614->client;
    int32_t rawdata = 0;
    int32_t value;
    if (!mlx90614) {
        pr_err("mlx90614 not initialized\n");
        return -EFAULT;
    }
    switch(cmd) {
        case MLX90614_GET_OBJ_TEMP_RAW_VALUE_:
            mlx90614_read_object_temperature(mlx90614);
            rawdata = mlx90614->object_temperature;
            if(copy_to_user((int32_t*) arg, &rawdata, sizeof(rawdata))) {
                pr_err("Error copying raw data to user space\n");
                return -EFAULT;
            }
            break;
        case MLX90614_GET_AMB_TEMP_RAW_VALUE_:
            mlx90614_read_ambient_temperature(mlx90614);
            rawdata = mlx90614->ambient_temperature;
            if(copy_to_user((int32_t*) arg, &rawdata, sizeof(rawdata))) {
                pr_err("Error copying raw data to user space\n");
                return -EFAULT;
            }
            break;
        case MLX90614_SET_PWM: 
            if( copy_from_user(&value ,(int32_t*) arg, sizeof(value))){
                pr_err("Data Write : Err!\n");
                return -EFAULT;
            }
        	pwm_config(pwm0, value, pwm_period_ns);
            break;
        default:
            pr_info("Default\n");
            break;
        }
        return 0;
}
/*
** File operation sturcture
*/
static struct file_operations fops =
{
        .owner          = THIS_MODULE,
        .open           = etx_open,
        .unlocked_ioctl = etx_ioctl,
};
/*
** Module Init function
*/
static int mlx90614_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct device *dev_num_dr = &client->dev;
    mlx90614_device *mlx90614 = devm_kzalloc(dev_num_dr, sizeof(mlx90614_device), GFP_KERNEL);
    if (!mlx90614) {
        pr_err("Failed to allocate memory using kzalloc\n");
        return -ENOMEM;
    }
    i2c_set_clientdata(client, mlx90614);
    mlx90614->client = client;
    /*Allocating Major number*/
    if((alloc_chrdev_region(&dev, 0, 1, "etx_Dev")) <0){
            pr_err("Cannot allocate major number\n");
            return -1;
    }
    pr_info("Major = %d Minor = %d \n",MAJOR(dev), MINOR(dev));
    /*Creating cdev structure*/
    cdev_init(&mlx90614->cdev,&fops);
    /*Adding character device to the system*/
    if((cdev_add(&mlx90614->cdev,dev,1)) < 0){
        pr_err("Cannot add the device to the system\n");
        goto r_class;
    }
    /*Creating struct class*/
    if(IS_ERR(mlx90614->dev_class = class_create(THIS_MODULE,"mlx90614_class"))){
        pr_err("Cannot create the struct class\n");
        goto r_class;
    }
    /*Creating device*/
    if(IS_ERR(device_create(mlx90614->dev_class ,NULL,dev,NULL,MLX90614_DEVICE_NAME))){
        pr_err("Cannot create the Device 1\n");
        goto r_device;
    }
	
    pwm0 = pwm_request(1, "my-pwm");
	if (pwm0 == NULL)
	{
		printk("Could not get PWM0!\n");
		goto AddError;
	}
	pwm_set_period(pwm0, pwm_period_ns);
	pwm_set_duty_cycle(pwm0, pwm_duty_ns);
	pwm_config(pwm0, 0, pwm_period_ns);
	pwm_enable(pwm0);
    pr_info("Device Driver Insert...Done!!!\n");
    return 0;
AddError:
	    device_destroy(mlx90614->dev_class,dev);
r_device:
        class_destroy(mlx90614->dev_class);
r_class:
        unregister_chrdev_region(dev,1);
        return -1;
}
static int mlx90614_remove(struct i2c_client *client)
{
    mlx90614_device *mlx90614 = i2c_get_clientdata(client);
    pwm_disable(pwm0);
	pwm_free(pwm0);
    cdev_del(&mlx90614->cdev);
    device_destroy(mlx90614->dev_class,dev);
    class_destroy(mlx90614->dev_class);
    unregister_chrdev_region(dev, 1);
    pr_info("Device mlx90614 Driver Remove...Done!!!\n");
    return 0;
}
static struct i2c_device_id mlx90614_i2c_device[] = {
	{MLX90614_DEVICE_NAME, 0},
	{ },
};
MODULE_DEVICE_TABLE(i2c, mlx90614_i2c_device);
static const struct of_device_id mlx90614_dt_match[] =
{
    { .compatible = "thermal_sensor" },
    { }
};
MODULE_DEVICE_TABLE(of, mlx90614_dt_match);
static struct i2c_driver mlx90614_driver =
{
    .driver = 
    {
        .name = MLX90614_DEVICE_NAME,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(mlx90614_dt_match),
    },
    .probe = mlx90614_probe,
    .remove = mlx90614_remove,
	.id_table = mlx90614_i2c_device,
}; 
module_i2c_driver(mlx90614_driver);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Eric");
MODULE_DESCRIPTION("MLX90614 device driver");
MODULE_VERSION("1.5");
