#include <linux/string.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm-generic/gpio.h>

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#endif


#include "lcm_drv.h"
/*#include "ddp_irq.h"*/

#define GPIO_OUT_ONE  1
#define GPIO_OUT_ZERO 0
/*static unsigned int GPIO_LCD_PWR_EN;*/
static struct regulator *lcm_vgp;
static unsigned int GPIO_LCD_PWR_EN;
static unsigned int GPIO_LCD_STB_EN;
static unsigned int GPIO_LCD_RST;

static void lcm_init_lcm(void);

/**
 * LCM Driver Implementations
 */
void lcm_get_gpio_infor(void)
{
	static struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "mediatek,lcm");

	GPIO_LCD_PWR_EN = of_get_named_gpio(node, "lcm_power_gpio", 0);
	GPIO_LCD_STB_EN = of_get_named_gpio(node, "lcm_stb_gpio", 0);
	GPIO_LCD_RST = of_get_named_gpio(node, "lcm_reset_gpio", 0);
	printk("lcm_get_gpio_infor GPIO_LCD_PWR_EN=%d,GPIO_LCD_STB_EN=%d,GPIO_LCD_RST=%d \r\n",GPIO_LCD_PWR_EN,GPIO_LCD_STB_EN,GPIO_LCD_RST);
}

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
	gpio_direction_output(GPIO, output);
	gpio_set_value(GPIO, output);
}

/* get LDO supply */
static int lcm_get_vgp_supply(struct device *dev)
{
	int ret;
	struct regulator *lcm_vgp_ldo;

	pr_debug("LCM: lcm_get_vgp_supply is going\n");

	lcm_vgp_ldo = devm_regulator_get(dev, "reg-lcm");
	if (IS_ERR(lcm_vgp_ldo)) {
		ret = PTR_ERR(lcm_vgp_ldo);
		dev_err(dev, "failed to get reg-lcm LDO, %d\n", ret);
		return ret;
	}

	pr_debug("LCM: lcm get supply ok.\n");

	ret = regulator_enable(lcm_vgp_ldo);
	/* get current voltage settings */
	ret = regulator_get_voltage(lcm_vgp_ldo);
	pr_debug("lcm LDO voltage = %d in LK stage\n", ret);

	lcm_vgp = lcm_vgp_ldo;

	return ret;
}

int lcm_vgp_supply_enable(void)
{
	int ret;
	unsigned int volt;

	pr_debug("LCM: lcm_vgp_supply_enable\n");

	if (NULL == lcm_vgp)
		return 0;

	pr_debug("LCM: set regulator voltage lcm_vgp voltage to 1.8V\n");
	/* set voltage to 1.8V */
	ret = regulator_set_voltage(lcm_vgp, 3300000, 3300000);
	if (ret != 0) {
		pr_err("LCM: lcm failed to set lcm_vgp voltage: %d\n", ret);
		return ret;
	}

	/* get voltage settings again */
	volt = regulator_get_voltage(lcm_vgp);
	if (volt == 3300000)
		pr_err("LCM: check regulator voltage=3300000 pass!\n");
	else
		pr_err("LCM: check regulator voltage=3300000 fail! (voltage: %d)\n", volt);

	ret = regulator_enable(lcm_vgp);
	if (ret != 0) {
		pr_err("LCM: Failed to enable lcm_vgp: %d\n", ret);
		return ret;
	}

	return ret;
}

int lcm_vgp_supply_disable(void)
{
	int ret = 0;
	unsigned int isenable;

	if (NULL == lcm_vgp)
		return 0;

	/* disable regulator */
	isenable = regulator_is_enabled(lcm_vgp);

	pr_debug("LCM: lcm query regulator enable status[0x%d]\n", isenable);

	if (isenable) {
		ret = regulator_disable(lcm_vgp);
		if (ret != 0) {
			pr_err("LCM: lcm failed to disable lcm_vgp: %d\n", ret);
			return ret;
		}
		/* verify */
		isenable = regulator_is_enabled(lcm_vgp);
		if (!isenable)
			pr_err("LCM: lcm regulator disable pass\n");
	}

	return ret;
}

static int lcm_probe(struct device *dev)
{
	lcm_get_vgp_supply(dev);
	lcm_get_gpio_infor();

	return 0;
}

static const struct of_device_id lcm_of_ids[] = {
	{.compatible = "mediatek,lcm",},
	{}
};

static struct platform_driver lcm_driver = {
	.driver = {
		   .name = "mtk_lcm",
		   .owner = THIS_MODULE,
		   .probe = lcm_probe,
#ifdef CONFIG_OF
		   .of_match_table = lcm_of_ids,
#endif
		   },
};

static int __init lcm_init(void)
{
	pr_notice("LCM: Register lcm driver\n");
	if (platform_driver_register(&lcm_driver)) {
		pr_err("LCM: failed to register disp driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcm_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	pr_notice("LCM: Unregister lcm driver done\n");
}
late_initcall(lcm_init);
module_exit(lcm_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("Display subsystem Driver");
MODULE_LICENSE("GPL");

/**
 * Local Constants
 */
#define FRAME_WIDTH		(720)
#define FRAME_HEIGHT		(1280)

#define REGFLAG_DELAY		0xFE
#define REGFLAG_END_OF_TABLE	0xFF   /* END OF REGISTERS MARKER */

#define LCM_DSI_CMD_MODE	0

/**
 * Local Variables
 */
static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


/**
 * Local Functions
 */
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)	lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)				lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)	lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg				lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)	lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#ifndef ASSERT
#define ASSERT(expr)					\
	do {						\
		if (expr)				\
			break;				\
		pr_debug("DDP ASSERT FAILED %s, %d\n",	\
		       __FILE__, __LINE__);		\
		BUG();					\
	} while (0)
#endif

struct LCM_setting_table {
	unsigned cmd;
	unsigned char count;
	unsigned char para_list[64];
};

/**
 * Note :
 *
 * Data ID will depends on the following rule.
 *
 * count of parameters > 1	=> Data ID = 0x39
 * count of parameters = 1	=> Data ID = 0x15
 * count of parameters = 0	=> Data ID = 0x05
 *
 * Structure Format :
 *
 * {DCS command, count of parameters, {parameter list}}
 * {REGFLAG_DELAY, milliseconds of time, {} },
 * ...
 *
 * Setting ending by predefined flag
 *
 * {REGFLAG_END_OF_TABLE, 0x00, {}}
 */
#if 0
static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 10, {}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static void push_table(struct LCM_setting_table *table, unsigned int count,
		       unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		unsigned cmd;

		cmd = table[i].cmd;
		switch (cmd) {

		case REGFLAG_DELAY:
			MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count,
					table[i].para_list, force_update);
		}
	}
}
#endif 

static void init_lcm_registers(void)
{
	unsigned int data_array[16];
	
	// SET password
	data_array[0]=0x00043902;
	data_array[1]=0x9483FFB9;
	dsi_set_cmdq(data_array, 2, 1);
//	MDELAY(10);
	
	data_array[0]=0x00033902;
	data_array[1]=0x008333BA;
	dsi_set_cmdq(data_array, 2, 1);
//	MDELAY(10);
	
	data_array[0]=0x00053902;
	data_array[1]=0x7d0000b0;
	data_array[2]=0x0000000c;
	dsi_set_cmdq(data_array, 3, 1);
//	MDELAY(10);	
	
	//Set Power
	data_array[0]=0x00103902;
	data_array[1]=0x15156cB1;
	data_array[2]=0xf1110424;
	data_array[3]=0x2397E480;
	data_array[4]=0x58D2C080;
	dsi_set_cmdq(data_array, 5, 1);
//	MDELAY(10);
	
	// SET CYC 
	data_array[0]=0x000C3902;
	data_array[1]=0x106400B2;
	data_array[2]=0x081C2207;
    data_array[3]=0x004D1C08;	
	dsi_set_cmdq(data_array, 4, 1);
//	MDELAY(10);

    // SET CYC 
	data_array[0]=0x000D3902;
	data_array[1]=0x03FF00B4;
	data_array[2]=0x035A035A;
	data_array[3]=0x306a015A;
	data_array[4]=0x0000006a;
	dsi_set_cmdq(data_array, 5, 1);
//	MDELAY(10);

	data_array[0]=0x00023902;
    data_array[1]=0x000007BC;
	dsi_set_cmdq(data_array, 2, 1);
//	MDELAY(10);
	
	data_array[0]=0x00043902;
	data_array[1]=0x010E41BF;
	dsi_set_cmdq(data_array, 2, 1);	
//	MDELAY(10);	
	
	//Set VCOM
	data_array[0]=0x00033902;
    data_array[1]=0x005c5cB6;
	dsi_set_cmdq(data_array, 2, 1);
//	MDELAY(10);	

	// Set panel 
	data_array[0]=0x00023902;
	data_array[1]=0x000009CC;
	dsi_set_cmdq(data_array, 2, 1);
//	MDELAY(10);

	// SET GIP 
	data_array[0]=0x001F3902;
	data_array[1]=0x000600D3;
	data_array[2]=0x00080740;
	data_array[3]=0x00071032;
	data_array[4]=0x0F155407;
	data_array[5]=0x12020405;
	data_array[6]=0x33070510;
	data_array[7]=0x370B0B33;   
	data_array[8]=0x00070710;
	dsi_set_cmdq(data_array, 9, 1);
//	MDELAY(10);	
	
	// SET GIP 
	data_array[0]=0x002d3902;
	data_array[1]=0x060504D5;
	data_array[2]=0x02010007;
	data_array[3]=0x22212003;
	data_array[4]=0x18181823;
	data_array[5]=0x18181818;
	data_array[6]=0x18191918;
	data_array[7]=0x1B181818;
	data_array[8]=0x181A1A1B;   
	data_array[9]=0x18181818;
	data_array[10]=0x18181818;
	data_array[11]=0x18181818;
	data_array[12]=0x00000018;
	dsi_set_cmdq(data_array, 13, 1);
//	MDELAY(10);	
	
	// SET GIP 
	data_array[0]=0x002D3902;
	data_array[1]=0x010203D6;
	data_array[2]=0x05060700;
	data_array[3]=0x21222304;
	data_array[4]=0x18181820;
	data_array[5]=0x58181818;
	data_array[6]=0x19181858;
	data_array[7]=0x1B181819;   
	data_array[8]=0x181A1A1B;
	data_array[9]=0x18181818;
	data_array[10]=0x18181818;
	data_array[11]=0x18181818;
	data_array[12]=0x00000018;
	dsi_set_cmdq(data_array, 13, 1);
//	MDELAY(10);	

	// R Gamma
	data_array[0]=0x002B3902;
	data_array[1]=0x161000E0;
	data_array[2]=0x233F332D;
	data_array[3]=0x0D0B073E;
	data_array[4]=0x14120E17;
	data_array[5]=0x11061312;
	data_array[6]=0x10001813;
	data_array[7]=0x3F332D16;
	data_array[8]=0x0B073E23;   
	data_array[9]=0x120E170D;
	data_array[10]=0x06131214;
	data_array[11]=0x00181311;
	dsi_set_cmdq(data_array, 12, 1);
//	MDELAY(10);
	
	data_array[0]=0x00033902;
    data_array[1]=0x001430C0;
	dsi_set_cmdq(data_array, 2, 1);
//	MDELAY(10);	
	
	data_array[0]=0x00053902;
	data_array[1]=0x40C000C7;
	data_array[2]=0x000000C0;
	dsi_set_cmdq(data_array, 3, 1);
//	MDELAY(10);

	data_array[0]=0x00023902;
    data_array[1]=0x00008edf;
	dsi_set_cmdq(data_array, 2, 1);
//	MDELAY(10);

	data_array[0]=0x00023902;
    data_array[1]=0x000066d2;
	dsi_set_cmdq(data_array, 2, 1);
//	MDELAY(10);	
	
	data_array[0]= 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
	
	
	data_array[0]= 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
//	MDELAY(20);  


}

static void lcm_init_power(void)
{
	pr_debug("[Kernel/LCM] lcm_init_power() enter\n");
}

static void lcm_suspend_power(void)
{
	lcm_set_gpio_output(GPIO_LCD_PWR_EN,GPIO_OUT_ZERO);
	MDELAY(20);		

	//VDD power off ->VGP1_PMU 1.8V
	lcm_vgp_supply_disable();

    lcm_set_gpio_output(GPIO_LCD_STB_EN,GPIO_OUT_ZERO);

	lcm_set_gpio_output(GPIO_LCD_RST,GPIO_OUT_ZERO);
	MDELAY(20);
}

static void lcm_resume_power(void)
{
	lcm_init_lcm();
}

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
     memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
   	params->dsi.mode   = BURST_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
#endif



	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order 	= LCM_COLOR_ORDER_RGB;	
	params->dsi.data_format.trans_seq   	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     	= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      	= LCM_DSI_FORMAT_RGB888;
	

	// Highly depends on LCD driver capability.
	params->dsi.packet_size=256;
	//video mode timing
	params->dsi.intermediat_buffer_num = 0;
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.word_count=FRAME_WIDTH*3;


	params->dsi.vertical_sync_active				= 2;
	params->dsi.vertical_backporch					= 16;
	params->dsi.vertical_frontporch					= 9;
	params->dsi.vertical_active_line				= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active				= 42;
	params->dsi.horizontal_backporch				= 42;
	params->dsi.horizontal_frontporch				= 69;
		params->dsi.horizontal_active_pixel			= FRAME_WIDTH;
		
    	params->dsi.ssc_disable							= 1;
		params->dsi.PLL_CLOCK = 221;
		params->dsi.cont_clock= 1;
		
}


static void lcm_init_lcm(void)
{
	lcm_get_gpio_infor();
	lcm_set_gpio_output(GPIO_LCD_PWR_EN, GPIO_OUT_ONE);
	MDELAY(20);

	//VDD power on ->VGP1_PMU 1.8V
	lcm_vgp_supply_enable();
	
    lcm_set_gpio_output(GPIO_LCD_STB_EN,GPIO_OUT_ONE);
    MDELAY(20);

	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);

	init_lcm_registers();
}


static void lcm_suspend(void)
{
	lcm_set_gpio_output(GPIO_LCD_PWR_EN,GPIO_OUT_ZERO);
	MDELAY(20);		

	//VDD power off ->VGP1_PMU 1.8V
	lcm_vgp_supply_disable();

    lcm_set_gpio_output(GPIO_LCD_STB_EN,GPIO_OUT_ZERO);

	lcm_set_gpio_output(GPIO_LCD_RST,GPIO_OUT_ZERO);
	MDELAY(20);
	
}


static void lcm_resume(void)
{
	lcm_init_lcm();
}

LCM_DRIVER hx8394d_hd720_dsi_vdo_guoxian_lcm_drv = {
	.name		= "hx8394d_hd720_dsi_vdo_guoxian",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init 			= lcm_init_lcm,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.init_power 	= lcm_init_power,
	.resume_power 	= lcm_resume_power,
	.suspend_power 	= lcm_suspend_power,
};
