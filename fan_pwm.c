/*
 * fan_pwm.c – шим для вентилятора NanoPi NEO3 Rockchip_RK3328 (модуль ядра).
 * GPIO2_A6 PWM2 LINUX_IO_70
 */

#include <asm/io.h>  //заголовочник для работы с ioread/iowrite
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
//#include <linux/cdev.h>
//#include <linux/kernel.h>
//#include <asm/uaccess.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
//#include <linux/version.h>
//#include <linux/delay.h> //функции задержек

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aleksandr KZ  e-mail: ya.a-leksandr@yandex.ru");
MODULE_DESCRIPTION("PWM DRIVER[FAN_CON] NANOPI NEO3 V 0.1 alpha");

#define BASE_CRU_ADDR 0xFF440000
#define CRU_CLKSEL_CON24 0x0160
#define CRU_CLKGATE_CON2 0x0208 //8bit = 1 = off clk pwm0
#define CRU_CLKGATE_CON16 0x0240  //6bit = 1 = off pclk
#define CRU_SOFTRST_CON3 0x030C


#define BASE_GRF_ADDR 0xFF100000
#define GRF_GPIO2A_IOMUX 0x0020 //GPIO2A iomux control  // 13:12bit  00=gpio  01=pwm2
#define GRF_GPIO2A_P 0x0120 //GPIO2A PU/PD control
#define GRF_GPIO2A_E 0x0220 //GPIO2A drive strength control
#define GRF_GPIO2L_SR 0x0310 //GPIO2 A/B SR control

#define BASE_PWM_ADDR 0xFF1B0000
#define PWM_PWM2_CNT 0x0020
#define PWM_PWM2_PERIOD_HPR 0x0024
#define PWM_PWM2_DUTY_LPR 0x0028
#define PWM_PWM2_CTRL 0x002C

#define BASE_GPIO2_ADDR 0xFF230000
#define GPIO2_SWPORTA_DR 0x0000
#define GPIO2_SWPORTA_DDR 0x0004

static void* gpio_map; // общий указатель
static int TMP = 0;
static unsigned int PWM2_DUTY_LPR = 0;
static unsigned int PWM2_PERIOD_HPR = 0;

//#define LEN_MSG 160
//static char buf_msg[LEN_MSG + 1]="";


static int freq = 200;
module_param(freq, int, 0660);

static int power = 100;

void init_pwm_mode(void);
void deinit_pwm_mode(void);

void pwm_on(int hz);
void pwm_off(char mode);

void set_freq(int hz);
void set_duty(char duty);

unsigned int get_freq(void);
int get_duty(void);


static ssize_t power_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) 
{
    power = get_duty();
    return sprintf(buf, "%d\n", power);
}

static ssize_t power_store(struct kobject *kobj, struct kobj_attribute *attr, char *buf, size_t count) 
{
	/*
	unsigned long tmp;
	if (strict_strtoul(buf, 10, &tmp) < 0)
		return -EINVAL;
	if(tmp>6)
		return -EINVAL;
	*/
    sscanf(buf, "%du", &power);
    set_duty(power);
    return count;
}

static struct kobject* fan_pwm_module;
static struct kobj_attribute freq_attribute = __ATTR(power, 0660, power_show, (void *)power_store);

int __init pwm_init(void)
{

    printk("PWM module: init %iHz.\n", freq);

    int error = 0;
    fan_pwm_module = kobject_create_and_add("fan_pwm", kernel_kobj);
    if (!fan_pwm_module)
        return -ENOMEM;


    error = sysfs_create_file(fan_pwm_module, &freq_attribute.attr);
    if (error) {
        pr_info("failed to create the freq file "
                "in /sys/kernel/fan_pwm\n");
    }

    if(error == 0)
    {
        pr_info("#insmod fan_pwm.ko freq=from 100 to 40000 \n"
                "in /sys/kernel/fan_pwm/power (echo 0-100 > power)\n");
        init_pwm_mode();
    }

    return error;

    //return 0;
}

void m_exit(void)
{

	printk("PWM module: de_init.\n");
    kobject_put(fan_pwm_module);
    deinit_pwm_mode();
	return;
}

module_init(pwm_init);
module_exit(m_exit);

void init_pwm_mode()
{
    //включаем тактирование PWM  (надо писать в верхнюю половину WORD32 bit1 для записи, очищается сам)
    //------------------------------------------------------------------------------
    gpio_map = ioremap(BASE_CRU_ADDR, 4096);//4096

    TMP = ioread32(gpio_map + CRU_CLKGATE_CON2); // When HIGH, disable clock. 8 bit clk_pwm0_src_en
    TMP |= (1 << 24);
    TMP &= ~(1 << 8);
    iowrite32(TMP,gpio_map + CRU_CLKGATE_CON2);

    TMP = ioread32(gpio_map + CRU_CLKGATE_CON16); // When HIGH, disable clock. 6 bit pclk_rk_pwm_en
    TMP |= (1 << 22);
    TMP &= ~(1 << 6);
    iowrite32(TMP,gpio_map + CRU_CLKGATE_CON16);

    iounmap(gpio_map);
    //-------------------------------------------------------------------------------

    //GPIO2_A6 as output
    //------------------------------------------------------------------------------
    gpio_map = ioremap(BASE_GPIO2_ADDR, 4096);//4096

    TMP = ioread32(gpio_map + GPIO2_SWPORTA_DDR);
    TMP |= (1 << 6);
    iowrite32(TMP,gpio_map + GPIO2_SWPORTA_DDR);

    iounmap(gpio_map);
    //-----------------------------------------------------------------------------

    //GPIO2_A6 as PWM mode (надо писать в верхнюю половину WORD32 bit1 для записи, очищается сам)
    //-------------------------------------------------------------------------------
    gpio_map = ioremap(BASE_GRF_ADDR, 4096);//4096

    TMP = ioread32(gpio_map + GRF_GPIO2A_P); //PULL-UP-01b PULL-DOWN-10b
    TMP &= ~(3 << 12); // reset 13:12 bits
    TMP |= (3 << 28) | (1 << 12); //set 12bit pull-up
    iowrite32(TMP,gpio_map + GRF_GPIO2A_P);

    TMP = ioread32(gpio_map + GRF_GPIO2A_IOMUX); // PIN AS PWM
    TMP &= ~(3 << 12); // reset 13:12 bits
    TMP |= (3 << 28) | (1 << 12); //set 12bit
    iowrite32(TMP,gpio_map + GRF_GPIO2A_IOMUX);

    iounmap(gpio_map);
    //--------------------------------------------------------------------------------
    if(freq == 0){
        pwm_on(200); //200Hz
    } else {
        pwm_on(freq);
    }
}

void deinit_pwm_mode()
{
    //----------------------------------------------------------------------------------
    //PWM RESET
    gpio_map = ioremap(BASE_PWM_ADDR, 4096);//4096

    TMP = 0;
    iowrite32(TMP,gpio_map + PWM_PWM2_PERIOD_HPR);
    iowrite32(TMP,gpio_map + PWM_PWM2_DUTY_LPR);
    iowrite32(TMP, gpio_map + PWM_PWM2_CTRL);

    iounmap(gpio_map);
    //---------------------------------------------------------------------------------

    //GPIO2_A6 as GPIO mode (надо писать в верхнюю половину WORD32 bit1 для записи, очищается сам)
    //-------------------------------------------------------------------------------
    gpio_map = ioremap(BASE_GRF_ADDR, 4096);//4096

    TMP = ioread32(gpio_map + GRF_GPIO2A_P); //ZERRO state
    TMP &= ~(3 << 12); // reset 13:12 bits
    TMP |= (3 << 28);
    iowrite32(TMP,gpio_map + GRF_GPIO2A_P);

    TMP = ioread32(gpio_map + GRF_GPIO2A_IOMUX); // PIN AS PWM
    TMP &= ~(3 << 12);
    TMP |= (3 << 28);
    iowrite32(TMP,gpio_map + GRF_GPIO2A_IOMUX);

    iounmap(gpio_map);
    //--------------------------------------------------------------------------------

    //GPIO2_A6 as output
    //------------------------------------------------------------------------------
    gpio_map = ioremap(BASE_GPIO2_ADDR, 4096);//4096

    TMP = ioread32(gpio_map + GPIO2_SWPORTA_DDR);
    TMP &= ~(1 << 6);
    iowrite32(TMP,gpio_map + GPIO2_SWPORTA_DDR);

    iounmap(gpio_map);
    //-----------------------------------------------------------------------------

    //выключаем тактирование PWM (надо писать в верхнюю половину WORD32 bit1 для записи, очищается сам)
    //------------------------------------------------------------------------------
    gpio_map = ioremap(BASE_CRU_ADDR, 4096);//4096

    TMP = ioread32(gpio_map + CRU_CLKGATE_CON2); // When HIGH, disable clock. 8 bit clk_pwm0_src_en
    TMP |= (1 << 24) | (1 << 8);
    iowrite32(TMP,gpio_map + CRU_CLKGATE_CON2);

    TMP = ioread32(gpio_map + CRU_CLKGATE_CON16); // When HIGH, disable clock. 6 bit pclk_rk_pwm_en
    TMP |= (1 << 22) | (1 << 6);
    iowrite32(TMP,gpio_map + CRU_CLKGATE_CON16);

    iounmap(gpio_map);
    //-------------------------------------------------------------------------------
}

void pwm_on(int hz)
{
    //включаем PWM2
    gpio_map = ioremap(BASE_PWM_ADDR, 4096);//4096

    TMP = 0;
    iowrite32(TMP,gpio_map + PWM_PWM2_CTRL);

    TMP = ioread32(gpio_map + PWM_PWM2_CTRL);
    TMP |= 0b10; //Continuous mode.
    iowrite32(TMP,gpio_map + PWM_PWM2_CTRL);

    PWM2_DUTY_LPR = 0; //100%
    iowrite32(PWM2_DUTY_LPR,gpio_map + PWM_PWM2_DUTY_LPR);

    int freq_ = 0;
    if(hz < 100)
    {
        freq_ = 100;
    }
    else if(hz > 40000)
    {
        freq_ = 40000;
    }
    else
    {
        freq_ = hz;
    }

    PWM2_PERIOD_HPR = (1000000000 / freq_);

    iowrite32(PWM2_PERIOD_HPR,gpio_map + PWM_PWM2_PERIOD_HPR);


    TMP = ioread32(gpio_map + PWM_PWM2_CTRL);
    TMP |= /*(1 << 6) |*/ (0b0001);
    iowrite32(TMP,gpio_map + PWM_PWM2_CTRL);

    iounmap(gpio_map);
}

void pwm_off(char mode)
{
    //включаем PWM2
    gpio_map = ioremap(BASE_PWM_ADDR, 4096);//4096

    TMP = ioread32(gpio_map + PWM_PWM2_CTRL);
    if(mode > 0)
    {
        TMP |= (1 << 4);
        TMP &= ~(1);
    } else {
        TMP = 0;
    }
    iowrite32(TMP,gpio_map + PWM_PWM2_CTRL);

    iounmap(gpio_map);
}

void set_freq(int hz) // 200 - 40 000 Hz
{
    unsigned int freq_;
    if(hz < 100)
    {
        freq_ = 100;
    }
    else if(hz > 40000)
    {
        freq_ = 40000;
    }
    else
    {
        freq_ = hz;
    }

    PWM2_PERIOD_HPR = (1000000000 / freq_);

    gpio_map = ioremap(BASE_PWM_ADDR, 4096);//4096

    iowrite32(PWM2_PERIOD_HPR,gpio_map + PWM_PWM2_PERIOD_HPR);

    iounmap(gpio_map);
}

void set_duty(char duty) // 0-100 %
{
    char power;
    unsigned int tmp = 0;

    if(duty < 0)
    {
        power = 0;
    }
    else if(duty > 100)
    {
        power = 100;
    }
    else
    {
        power = duty;
    }

    tmp = PWM2_PERIOD_HPR *  power / 100;

    PWM2_DUTY_LPR = PWM2_PERIOD_HPR - tmp;

    if(PWM2_DUTY_LPR > PWM2_PERIOD_HPR || PWM2_DUTY_LPR < 0)
    {
        PWM2_DUTY_LPR = 0;
    }

    gpio_map = ioremap(BASE_PWM_ADDR, 4096);//4096

    iowrite32(PWM2_DUTY_LPR,gpio_map + PWM_PWM2_DUTY_LPR);

    iounmap(gpio_map);
}

unsigned int get_freq()
{
    return 1000000000 / PWM2_PERIOD_HPR;
    //return PWM2_PERIOD_HPR;
}

int get_duty()
{
    return 100 - (PWM2_DUTY_LPR * 100 / PWM2_PERIOD_HPR);
    //return PWM2_DUTY_LPR;
}
