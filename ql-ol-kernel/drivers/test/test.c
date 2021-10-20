/* Raspberry pi GPIO interrupt processing example
 * Light on LED with BUTTON press
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/device.h>//含有类相关的处理函数
#include <asm/uaccess.h>//含有copy_from_user函数
#include <asm/io.h>        //含有iomap函数iounmap函数

#define SUCCESS (0)
#define ERROR (-1)

static struct fasync_struct *fasync_queue; //异步通知队列

#define TEST_PIN 25    /* GPIO 25 */
//int flag = 0;

/*
struct cdev *test_dev;//使用cdev结构体来描述字符设备
dev_t devNum;//通过其成员dev_t来定义设备号（分为主、次设备号）以确定字符设备的唯一性
unsigned int subDevNum = 1;//请求的连续设备个数
int reg_major = 501;//主设备号
int reg_minor = 0;  //次设备号
*/

static struct class *test_class;
static struct class_device	*test_class_dev;

int major;

static DEFINE_SEMAPHORE(test_queue);     //定义互斥锁

static struct timer_list test_timer;

static DECLARE_WAIT_QUEUE_HEAD(button_waitq);

/* 中断事件标志, 中断服务程序将它置1，sixth_drv_read将它清0 */
static volatile int ev_press = 0;

struct pin_desc{
	unsigned int pin;
	unsigned int key_val;
};


/* 键值: 按下时, 0x01, 0x02, 0x03, 0x04 */
/* 键值: 松开时, 0x81, 0x82, 0x83, 0x84 */
static unsigned char key_val;

struct pin_desc pins_desc[1] = {
	{25, 0x01},  //GPIO1 - GPIO_25
//	{10, 0x02},  //GPIO2 - GPIO_10
//	{42, 0x03},  //GPIO3 - GPIO_42
//	{11, 0x04}, //GPIO4 - GPIO_11
};

static irqreturn_t irq_handler(int irq, void *dev)
{
    //printk("[puck] enter irq_handler...\r\n");
	
	//irq_pd = (struct pin_desc *)dev_id;
	mod_timer(&test_timer, jiffies+10);
	
    return IRQ_HANDLED;
}

static int test_drv_open(struct inode *inode, struct file *file)
{
#if 0	
	if (!atomic_dec_and_test(&canopen))
	{
		atomic_inc(&canopen);
		return -EBUSY;
	}
#endif		

	if (file->f_flags & O_NONBLOCK)
	{
		if (down_trylock(&test_queue))
			return -EBUSY;
	}
	else
	{
		/* 获取信号量 */
		down(&test_queue);
	}

	/* 配置GPF0,2为输入引脚 */
	/* 配置GPG3,11为输入引脚 */
	request_irq(gpio_to_irq(TEST_PIN),  irq_handler, IRQ_TYPE_EDGE_BOTH, "GPIO1", NULL);
//	request_irq(IRQ_EINT2,  irq_handler, IRQT_BOTHEDGE, "GPIO2", &pins_desc[1]);
//	request_irq(IRQ_EINT11, irq_handler, IRQT_BOTHEDGE, "GPIO3", &pins_desc[2]);
//	request_irq(IRQ_EINT19, irq_handler, IRQT_BOTHEDGE, "GPIO4", &pins_desc[3]);	

	return 0;
}

ssize_t test_drv_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	if (size != 1)
		return -EINVAL;

	if (file->f_flags & O_NONBLOCK)
	{
		if (!ev_press)
			return -EAGAIN;
	}
	else
	{
		/* 如果没有按键动作, 休眠 */
		wait_event_interruptible(button_waitq, ev_press);
	}

	/* 如果有按键动作, 返回键值 */
	copy_to_user(buf, &key_val, 1);
	ev_press = 0;
	
	return 1;
}

int test_drv_close(struct inode *inode, struct file *file)
{
	//atomic_inc(&canopen);
	free_irq(gpio_to_irq(TEST_PIN), &pins_desc[0]);
//	free_irq(IRQ_EINT2, &pins_desc[1]);
//	free_irq(IRQ_EINT11, &pins_desc[2]);
//	free_irq(IRQ_EINT19, &pins_desc[3]);
	up(&test_queue);
	return 0;
}

static unsigned test_drv_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;
	poll_wait(file, &button_waitq, wait); // 不会立即休眠

	if (ev_press)
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

static int test_fasync(int fd, struct file * filp, int on)
{
/*  
	int retval;
    retval=fasync_helper(fd,filp,on,&fasync_queue);
    if(retval<0)
      return retval;
    return 0;
*/
	printk("driver: test_fasync\n");
	return fasync_helper (fd, filp, on, &fasync_queue);
}

static struct file_operations dev_fops = {
    .owner = THIS_MODULE,
	.open    =  test_drv_open,     
	.read	 =	test_drv_read,	   
	.release =  test_drv_close,
	.poll    =  test_drv_poll,
    .fasync = test_fasync,
};

int major;

static void test_timer_function(unsigned long data)
{
//	struct pin_desc * pindesc = irq_pd;
	unsigned int pinval;

//	if (!pindesc)
//		return;
	
	pinval = gpio_get_value(TEST_PIN);

	if (pinval)
	{
		/* 松开 */
		printk("headset plugged in ...\r\n");
	}
	else
	{
		/* 按下 */
		printk("headset unplugged ...\r\n");
	}

    ev_press = 1;                  /* 表示中断发生了 */
    wake_up_interruptible(&button_waitq);   /* 唤醒休眠的进程 */
	
	kill_fasync (&fasync_queue, SIGIO, POLL_IN);
}

static int test_init(void)
{
    int err;

	init_timer(&test_timer);
	test_timer.function = test_timer_function;
	//test_timer.expires  = 0;
	add_timer(&test_timer); 

/*
    devNum = MKDEV(reg_major, reg_minor);//通过主次设备号生成设备号
    if(SUCCESS == register_chrdev_region(devNum, subDevNum, "headset")){
        printk(KERN_INFO "register_chrdev_region ok \n");
    }
    else {
        printk(KERN_INFO "register_chrdev_region error n");
        return ERROR;
    }

    printk(KERN_INFO " headset driver init \n");

    test_dev = kzalloc(sizeof(struct cdev), GFP_KERNEL);
    cdev_init(test_dev, &dev_fops);
    cdev_add(test_dev, devNum, 1);
*/
	major = register_chrdev(0, "mytest", &dev_fops); // 注册, 告诉内核
	test_class = class_create(THIS_MODULE, "mytest");

	//led_class_dev = class_device_create(led_class, NULL, MKDEV(major, 0), NULL, "led1"); /* /dev/led1 */
	test_class_dev = device_create(test_class, NULL, MKDEV(major, 0), NULL, "headset"); /* /dev/led1 */
    
	printk(KERN_INFO "test init\n");
    err = gpio_request_one(TEST_PIN, GPIOF_IN, "test");
    if (err) 
		return err;

/*
    enable_irq(gpio_to_irq(TEST_PIN));
    err = request_irq(gpio_to_irq(TEST_PIN), irq_handler,
            IRQ_TYPE_EDGE_BOTH, "test", NULL);
    if (err < 0) {
        printk("irq_request failed!\n");
        return err;
    }
	*/
 //   flag = 1;
	
    return 0;
}

static void test_exit(void)
{
    printk(KERN_INFO "LED exit\n");
  //  if(flag) free_irq(gpio_to_irq(TEST_PIN), NULL);
  //  gpio_free(TEST_PIN);
	
	unregister_chrdev(major, "mytest");
	device_unregister(test_class_dev);
	class_destroy(test_class);
	//iounmap(gpfcon);
	//iounmap(gpgcon);
}

module_init(test_init);
module_exit(test_exit);

MODULE_LICENSE("Dual BSD/GPL");

