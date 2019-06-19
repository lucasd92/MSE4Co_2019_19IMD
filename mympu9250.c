#include <linux/init.h>           // Macros used to mark up functions e.g. __init __exit
#include <linux/module.h>         // Core header for loading LKMs into the kernel
#include <linux/device.h>         // Header to support the kernel Driver Model
#include <linux/kernel.h>         // Contains types, macros, functions for the kernel
#include <linux/fs.h>             // Header for the Linux file system support
#include <linux/uaccess.h>          // Required for the copy to user function+
#include <linux/i2c.h>
#include <linux/utsname.h>
#include <linux/string.h>
#include <linux/types.h>

#define  DEVICE_NAME "i2c_mse"    ///< The device will appear at /dev/i2c using this value
#define  CLASS_NAME  "i2c"        ///< The device class -- this is a character device driver

MODULE_LICENSE("GPL");            ///< The license type -- this affects available functionality
MODULE_AUTHOR("Lucas Dordolo");    ///< The author -- visible when you use modinfo
MODULE_DESCRIPTION("A simple Linux mpu9250 driver for the BBB");  ///< The description -- see modinfo
MODULE_VERSION("0.1");            ///< A version number to inform users


static int    majorNumber;                  ///< Stores the device number -- determined automatically
static char   message[256] = {0};           ///< Memory for the string that is passed from userspace
static int    numberOpens = 0;              ///< Counts the number of times the device is opened
static struct class*  i2cClass  = NULL; ///< The device-driver class struct pointer
static struct device* i2cDevice = NULL; ///< The device-driver device struct pointer
static struct i2c_client *modClient;


// The prototype functions for the character driver -- must come before the struct definition
static int     dev_open(struct inode *, struct file *);
static int     dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);


static struct file_operations fops =
{
   	.open = dev_open,
   	.read = dev_read,
   	.write = dev_write,
   	.release = dev_release,
};


static const struct i2c_device_id my_mpu9250_i2c_id[] = {
{ "my_mpu9250", 0 },
{ }
};

#ifdef CONFIG_OF
MODULE_DEVICE_TABLE(i2c, my_mpu9250_i2c_id);

static const struct of_device_id my_mpu9250_dt_ids[] = {
    { .compatible = "mse,my_mpu9250" },
    { }
};
MODULE_DEVICE_TABLE(of, my_mpu9250_dt_ids);
#endif



static int ebbchar_init(void){
   printk(KERN_INFO "MPU9250: Initializing ...\n");
 
   // Try to dynamically allocate a major number for the device -- more difficult but worth it
   majorNumber = register_chrdev(0, DEVICE_NAME, &fops);
   if (majorNumber<0){
      printk(KERN_ALERT "MPU9250: failed to register a major number\n");
      return majorNumber;
   }
   printk(KERN_INFO "MPU9250: registered correctly with major number %d\n", majorNumber);
 
   // Register the device class
   i2cClass = class_create(THIS_MODULE, CLASS_NAME);
   if (IS_ERR(i2cClass)){                // Check for error and clean up if there is
      unregister_chrdev(majorNumber, DEVICE_NAME);
      printk(KERN_ALERT "Failed to register device class\n");
      return PTR_ERR(i2cClass);          // Correct way to return an error on a pointer
   }
   printk(KERN_INFO "MPU9250: device class registered correctly\n");
 
   // Register the device driver
   i2cDevice = device_create(i2cClass, NULL, MKDEV(majorNumber, 0), NULL, DEVICE_NAME);
   if (IS_ERR(i2cDevice)){               // Clean up if there is an error
      class_destroy(i2cClass);           // Repeated code but the alternative is goto statements
      unregister_chrdev(majorNumber, DEVICE_NAME);
      printk(KERN_ALERT "Failed to create the device\n");
      return PTR_ERR(i2cDevice);
   }
   printk(KERN_INFO "MPU9250: device class created correctly\n"); // Made it! device was initialized
   return 0;
}

static void ebbchar_exit(void){
   device_destroy(i2cClass, MKDEV(majorNumber, 0));     // remove the device
   class_unregister(i2cClass);                          // unregister the device class
   class_destroy(i2cClass);                             // remove the device class
   unregister_chrdev(majorNumber, DEVICE_NAME);             // unregister the major number
   printk(KERN_INFO "MPU9250: Goodbye cruel world\n");
}

static int dev_open(struct inode *inodep, struct file *filep){
   numberOpens++;
   printk(KERN_INFO "MPU9250: Device has been opened %d time(s)\n", numberOpens);
   return 0;
}


static ssize_t dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset){



	int error_count = 0;
   int Ret;
    pr_info("Leyendo registros de MPU9250\n");
	
    Ret = i2c_master_recv(modClient, message, len);

   // copy_to_user has the format ( * to, *from, size) and returns 0 on success
   error_count = copy_to_user(buffer, message, len);
 
   if (error_count==0){            // if true then have success
      return 0;
   }
   else {
      pr_info("MPU9250: Failed to send %d characters to the user\n", error_count);
      return -EFAULT;              // Failed -- return a bad address message (i.e. -14)
   }
	
}
 
static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset){
	int error_count = 0;
 
   	error_count = copy_from_user(message ,buffer, len);
	error_count = i2c_master_send(modClient,message,len);


   return len;
   
}


static int dev_release(struct inode *inodep, struct file *filep){

   pr_info( "MPU9250: Device successfully closed\n");

   return 0;
}




static int my_mpu9250_probe(struct i2c_client *client,const struct i2c_device_id *id){
/* initialize device */
/* register to a kernel framework */

    ebbchar_init();
    modClient = client;	//guardamos este puntero para el uso de read y write
	pr_info("PROBE:my_mpu9250\n");
    return 0;
}

static int my_mpu9250_remove(struct i2c_client *client)
{
    pr_info("REMOVE:my_mpu9250\n");
    ebbchar_exit();
    return 0;
}

static struct i2c_driver my_mpu9250_i2c_driver = {

	.probe = my_mpu9250_probe,
    .remove = my_mpu9250_remove,
    .id_table = my_mpu9250_i2c_id,
	.driver = {
        .name = "my_mpu9250",
		//.owner = THIS_MODULE,
        .of_match_table = of_match_ptr(my_mpu9250_dt_ids),
    },
};

module_i2c_driver(my_mpu9250_i2c_driver);
MODULE_LICENSE("GPL");





