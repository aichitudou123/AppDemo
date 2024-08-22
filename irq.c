#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/irq.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/platform_device.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/mempool.h>
#include <linux/mm.h>
#include <linux/highmem.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/kasan.h>
#include <linux/kmemleak.h>
#include <linux/workqueue.h>

#define keyDEV_CNT		1				/* 设备号长度 	*/
#define keyDEV_NAME		"mykey"	/* 设备名字 	*/
#define keyOFF 			0x10
#define keyON 			0x12
#define keyCmdMAGIC		0xEF
#define KEYCMDW         _IOR(keyCmdMAGIC,0,unsigned int)
#define KEYCMDR 		_IOR(keyCmdMAGIC,1,unsigned int)
char ioctlBuf[20] = {0};
char RW_buf[20] = {0};

unsigned int ioctldata = 0x68;
// #define MALLOC
//#define SLAB_CACHE
//#define VMALLOC
#define MEMPOOL

// #define TASKLET
#define WORKQUEUE
/* 中断IO描述结构体 */
struct irq_keydesc {
	int hw_irqnum;								/* gpio */
	int soft_irqnum;						/* 中断号     */
	unsigned char value;					/* 按键对应的键值 */
	char name[10];							/* 名字 */
	irqreturn_t (*handler)(int, void *);	/* 中断服务函数 */
};

/* keydev设备结构体 */
struct key_dev{
	dev_t devid;				/* 设备号	*/
	struct cdev cdev;			/* cdev		*/
	struct class *class;		/* 类 		*/
	struct device *device;		/* 设备		*/
	int major;					/* 主设备号	*/	
	struct device_node *node;	/* key设备节点 */
	int irq_gpio;					/* key灯GPIO标号,硬件中断号 */
	atomic_t dataRCondition;  /*0为没准备好，1为准备好了*/
	atomic_t dataBufisFull; /*0为缓冲区满，1为缓冲区中空*/
    atomic_t key0value;		/* 有效的按键键值 */
	struct timer_list timer;/* 定义一个定时器*/
    struct irq_keydesc *irq_key0; /*key0的中断描述符,在堆上分配*/
	struct kmem_cache *slab_desc; /*slab_cache描述符*/
	mempool_t *key_mempool;    /*内存池*/ 
	wait_queue_head_t r_wait,w_wait;	/* 等待队列头 */
	
};

struct key_dev keydev; 		/* key设备 */
int count = 0;

#ifdef TASKLET
static void tasklet_func(unsigned long data)
{
	// struct key_dev *dev = (struct key_dev *)arg;
	printk("this is tasklet_func \r\n");

}
static DECLARE_TASKLET(key_task_name,tasklet_func,0);		
#endif

#ifdef WORKQUEUE
static void workQueue_func(struct work_struct *unused_data)
{

	printk("this is workQueue_func \r\n");
}
static 	DECLARE_WORK(key0Work,workQueue_func);
#endif

static irqreturn_t Key0handler(int irq, void *dev)
{
	struct key_dev *key0_dev;
	key0_dev = (struct key_dev*)dev;
	printk("this is simple irq\r\n");
	count++;
#ifdef TASKLET
	if(count == 5)
	{
		/*开始调度*/
		tasklet_schedule(&key_task_name);
	//	tasklet_hi_schedule(&key_task_name);
	}
#endif
#ifdef WORKQUEUE
	if(count == 6)
	{
		/*开始调度*/
		schedule_work(&key0Work);
	}
#endif
 
	atomic_set(&key0_dev->dataRCondition,1); /*缓存区可读*/
	wake_up_interruptible(&key0_dev->r_wait);
	printk("wake up read ok");
	return IRQ_WAKE_THREAD;  /*唤醒中断下半部线程*/
}

static irqreturn_t Key0_thread_handler(int irq, void *dev)
{
    printk("this is complex irq\r\n");

    // mod_timer(&key0_dev->timer, jiffies + msecs_to_jiffies(10));	/* 10ms定时 */
	return IRQ_RETVAL(IRQ_HANDLED);

}


static int KeyGPIO_init(struct key_dev *dev)
{
    int ret =0;
    /* 5、初始化IO */	
	dev->node = of_find_node_by_path("/key");
	if (dev->node == NULL){
		printk("gpiokey node nost find!\r\n");
		return -EINVAL;
	} 
	
	dev->irq_gpio = of_get_named_gpio(dev->node, "key-gpios", 0);
	if (dev->irq_gpio < 0) {
		printk("can't get key-gpio\r\n");
		return -EINVAL;
	}
	gpio_request(dev->irq_gpio, "key0");
	gpio_direction_input(dev->irq_gpio);
    printk("key_gpio num:%d \r\n",dev->irq_gpio);

    /*申请中断*/
	/*分配内存*/
#ifdef MALLOC
	dev->irq_key0 = (struct irq_keydesc*)kmalloc(sizeof(struct irq_keydesc),GFP_KERNEL);
	if(!dev->irq_key0)
	{
		printk("kmalloc irq_keydesc failed\r\n");
		return -1;
	}
#endif

#ifdef SLAB_CACHE
	/*创建自己的slab描述符*/
	dev->slab_desc = kmem_cache_create("key_slab",sizeof(struct irq_keydesc),SLAB_HWCACHE_ALIGN,0,NULL);
	dev->irq_key0 = kmem_cache_zalloc(dev->slab_desc,GFP_KERNEL);
	if(!dev->irq_key0)
	{
		printk("kmem_cache_zalloc irq_keydesc failed\r\n");
		return -1;	
	}
#endif

#ifdef MEMPOOL
	dev->slab_desc = kmem_cache_create("key_slab",sizeof(struct irq_keydesc),SLAB_HWCACHE_ALIGN,0,NULL);
	/*传入slab_cache的描述符创建内存池对象*/
	dev->key_mempool = mempool_create(5,mempool_alloc_slab,mempool_free_slab,dev->slab_desc);
	/*从内存池中分配对象,内部会调用kmem_cache_alloc*/
	dev->irq_key0 = (struct irq_keydesc *)mempool_alloc(dev->key_mempool,0);
	if(!dev->irq_key0)
	{
		printk("mempool_alloc irq_keydesc failed\r\n");
		return -1;	
	}
#endif

#ifdef VMALLOC
/*vmalloc分配时默认flag为GFP_KERNEL|GFP_HIGHMEM*/
	dev->irq_key0 = (struct irq_keydesc*)vmalloc(sizeof(struct irq_keydesc)); 
	if(!dev->irq_key0)
	{
		printk("vmalloc irq_keydesc failed\r\n");
		return -1;
	}
#endif
    dev->irq_key0->hw_irqnum = dev->irq_gpio;
    /*将硬件中断号映射为软件中断号,保存到dev结构体中*/
	dev->irq_key0->soft_irqnum = irq_of_parse_and_map(dev->node, 0); 
    printk("sofr irq :%d \r\n",dev->irq_key0->soft_irqnum);
    strcpy(dev->irq_key0->name,"KEY0");
    dev->irq_key0->hw_irqnum = dev->irq_gpio;
    dev->irq_key0->handler = Key0handler;


	/*请求中断*/
    // ret = request_irq(dev->irq_key0.soft_irqnum, dev->irq_key0.handler, 
    // IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,
	// dev->irq_key0.name,&keydev);
	/*请求中断+中断线程化*/
	ret = request_threaded_irq(dev->irq_key0->soft_irqnum, Key0handler,
			 Key0_thread_handler, IRQF_TRIGGER_FALLING,
			 dev->irq_key0->name,&keydev);

    return 0;
}
/*
 * @description		: 打开设备
 * @param - inode 	: 传递给驱动的inode
 * @param - filp 	: 设备文件，file结构体有个叫做private_data的成员变量
 * 					  一般在open的时候将private_data指向设备结构体。
 * @return 			: 0 成功;其他 失败
 */
static int key_open(struct inode *inode, struct file *filp)
{
	filp->private_data = &keydev; /* 设置私有数据  */
	return 0;
}

/*
 * @description		: 向设备写数据 
 * @param - filp 	: 设备文件，表示打开的文件描述符
 * @param - buf 	: 要写给设备写入的数据
 * @param - cnt 	: 要写入的数据长度
 * @param - offt 	: 相对于文件首地址的偏移
 * @return 			: 写入的字节数，如果为负值，表示写入失败
 */
static ssize_t key_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *offt)
{
	int ret;
	struct key_dev *dev = (struct key_dev *)filp->private_data;
	
	while(!atomic_read(&dev->dataBufisFull)) /*dataBufisFull = 0 缓存区已满*/
	{
		if(filp->f_flags & O_NONBLOCK)/*非阻塞*/
		return -EAGAIN;

		printk("RW_buf is full,going to sleep\r\n");
		/*缓冲区满的条件一直成立，则一直沉睡*/
		ret = wait_event_interruptible(dev->w_wait,(atomic_read(&dev->dataBufisFull)==0));
		if(ret)
		{
			printk("key_write wait_event_interruptible erro");
			return ret;
		}
		if(signal_pending(current))	{			/* 判断是否为信号引起的唤醒 */
			ret = -ERESTARTSYS;
			return ret;
		}
		mdelay(1000);
	}

	ret = copy_from_user(RW_buf,buf,sizeof(RW_buf));
	atomic_set(&dev->dataBufisFull,0); /*缓冲区已满*/
	/*按下按键，中断函数中唤醒读*/
	return ret;

}
static ssize_t key_read(struct file *filp, char __user *buf, size_t cnt, loff_t *offt)
{
    int ret = 0;
	struct key_dev *dev = (struct key_dev *)filp->private_data;
	//DECLARE_WAITQUEUE(wait, current);	/* 定义一个等待队列 */
	
	while(!atomic_read(&dev->dataRCondition)) /* dataRCondition = 0,如果数据没准备好*/
	{
		printk("data is not ready,going to sleep\r\n");
		if(filp->f_flags & O_NONBLOCK)/*非阻塞*/
		return -EAGAIN;
		
		/*如果是阻塞 睡眠等待有数据读 wait_event_interruptible第二个参数是condition condition=1 保持睡眠*/
		ret = wait_event_interruptible(dev->r_wait,(atomic_read(&dev->dataRCondition)==0));
		if(ret)
		{
			printk("wait_event_interruptible erro");
			return ret;
		}

		if(signal_pending(current))	{			/* 判断是否为信号引起的唤醒 */
			ret = -ERESTARTSYS;
			return ret;
		}
		mdelay(1000);

	}
	ret = copy_to_user(buf,RW_buf,sizeof(RW_buf));
	memset(RW_buf,0,sizeof(RW_buf));
	/*唤醒所有写入*/
	atomic_set(&dev->dataBufisFull,1); /*缓冲区置空*/
	atomic_set(&dev->dataRCondition,0); /*无数据可读*/
	wake_up_interruptible(&dev->w_wait);


    return ret;
}
static long key_ioctl (struct file *file, unsigned int cmd, unsigned long arg)
{
	/*指向用户空间的指针*/
	unsigned long __user *datap = (unsigned long __user *)arg;
	int ret = 0;
	switch (cmd) 
	{
	case KEYCMDR:
		/*从右往左，内核空间到用户空间*/
		ret = copy_to_user(datap,ioctlBuf,sizeof(ioctlBuf));
		break;
	case KEYCMDW:
		/*从右往左，用户空间到内核空间*/
		ret = copy_from_user(ioctlBuf,datap,sizeof(ioctlBuf));
		break;
	default:
		break;
	}
	return 0;
}
/* 设备操作函数 */
static struct file_operations key_fops = {
	.owner = THIS_MODULE,
	.open = key_open,
	.write = key_write,
    .read = key_read,
	.unlocked_ioctl = key_ioctl
};

/*
 * @description		: flatform驱动的probe函数，当驱动与
 * 					  设备匹配以后此函数就会执行
 * @param - dev 	: platform设备
 * @return 			: 0，成功;其他负值,失败
 */
static int key_probe(struct platform_device *dev)
{	
	printk("key driver and device was matched!\r\n");
	/* 1、设置设备号 */
	if (keydev.major) {
		keydev.devid = MKDEV(keydev.major, 0);
		register_chrdev_region(keydev.devid, keyDEV_CNT, keyDEV_NAME);
	} else {
		alloc_chrdev_region(&keydev.devid, 0, keyDEV_CNT, keyDEV_NAME);
		keydev.major = MAJOR(keydev.devid);
	}

	/* 2、注册设备      */
	cdev_init(&keydev.cdev, &key_fops);
	cdev_add(&keydev.cdev, keydev.devid, keyDEV_CNT);

	/* 3、创建类      */
	keydev.class = class_create(THIS_MODULE, keyDEV_NAME);
	if (IS_ERR(keydev.class)) {
		return PTR_ERR(keydev.class);
	}

	/* 4、创建设备 */
	keydev.device = device_create(keydev.class, NULL, keydev.devid, NULL, keyDEV_NAME);
	if (IS_ERR(keydev.device)) {
		return PTR_ERR(keydev.device);
	}
  
	atomic_set(&keydev.dataRCondition,0); /*设置读取缓冲区没有准备好*/
	atomic_set(&keydev.dataBufisFull,1); /*设置缓冲区为空*/
		/* 初始化等待队列头 */
	init_waitqueue_head(&keydev.r_wait);
	init_waitqueue_head(&keydev.w_wait);
    KeyGPIO_init(&keydev);

	
	return 0;
}

/*
 * @description		: platform驱动的remove函数，移除platform驱动的时候此函数会执行
 * @param - dev 	: platform设备
 * @return 			: 0，成功;其他负值,失败
 */
static int key_remove(struct platform_device *dev)
{
	
    printk("key dirver exit\r\n");
    /* 注销中断 */

	free_irq(keydev.irq_key0->soft_irqnum, &keydev); /*释放软件中断号*/
#ifdef MALLOC
	kfree(keydev.irq_key0);
#endif
#ifdef SLAB_CACHE
	kmem_cache_free(keydev.slab_desc,keydev.irq_key0);
	kmem_cache_destroy(keydev.slab_desc);
#endif
#ifdef VMALLOC
	vfree(keydev.irq_key0);
#endif
#ifdef MEMPOOL
	mempool_free(keydev.irq_key0,keydev.key_mempool); 	/*释放对象回到内存池*/
	mempool_destroy(keydev.key_mempool); /*摧毁内存池*/
#endif
#ifdef TASKLET
	tasklet_kill(&key_task_name);
#endif
	gpio_set_value(keydev.irq_gpio, 1); 	/* 卸载驱动的时候关闭key */
	gpio_free(keydev.irq_gpio);				/* 释放gpio 硬件中断号 	*/
	cdev_del(&keydev.cdev);				/*  删除cdev */
	unregister_chrdev_region(keydev.devid, keyDEV_CNT); /* 注销设备号 */
	device_destroy(keydev.class, keydev.devid);
	class_destroy(keydev.class);
    
	return 0;
}

/* 匹配列表 */
static const struct of_device_id key_of_match[] = {
	{ .compatible = "alientek,key" },
	{ /* Sentinel */ }
};

/* platform驱动结构体 */
static struct platform_driver key_driver = {
	.driver		= {
		.name	= "key",			/* 驱动名字，用于和设备匹配 */
		.of_match_table	= key_of_match, /* 设备树匹配表 		 */
	},
	.probe		= key_probe,
	.remove		= key_remove,
};
		
/*
 * @description	: 驱动模块加载函数
 * @param 		: 无
 * @return 		: 无
 */
static int __init keydriver_init(void)
{
	return platform_driver_register(&key_driver);
}

/*
 * @description	: 驱动模块卸载函数
 * @param 		: 无
 * @return 		: 无
 */
static void __exit keydriver_exit(void)
{
	platform_driver_unregister(&key_driver);
}

module_init(keydriver_init);
module_exit(keydriver_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zuozhongkai");



