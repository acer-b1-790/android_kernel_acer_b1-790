#include <linux/string.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <asm-generic/gpio.h>

#include "lcm_drv.h"
#include "ddp_irq.h"

static unsigned int GPIO_LCD_PWR_EN;
static unsigned int GPIO_LCD_BL_EN;
/**
 * Local Constants
 */
#define FRAME_WIDTH		(800)
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
static struct LCM_setting_table lcm_initialization_setting[] = {
	/* sleep out */
	{0x11, 0, {} },
	{REGFLAG_DELAY, 120, {} },
	{0x29, 0, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};
#endif

static void init_lcm_registers(void)
{
	unsigned int data_array[16];
	
	// SET password
	data_array[0]=0x00043902;
	data_array[1]=0x9483FFB9;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(10);
	
	data_array[0]=0x00033902;
	data_array[1]=0x008333BA;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(10);
	
	data_array[0]=0x00053902;
	data_array[1]=0x7d0000b0;
	data_array[2]=0x0000000c;
	dsi_set_cmdq(data_array, 3, 1);
	MDELAY(10);	
	
	//Set Power
	data_array[0]=0x00103902;
	data_array[1]=0x15156cB1;
	data_array[2]=0xf1110424;
	data_array[3]=0x2397E480;
	data_array[4]=0x58D2C080;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(10);
	
	// SET CYC 
	data_array[0]=0x000C3902;
	data_array[1]=0x106400B2;
	data_array[2]=0x081C2207;
    data_array[3]=0x004D1C08;	
	dsi_set_cmdq(data_array, 4, 1);
	MDELAY(10);

    // SET CYC 
	data_array[0]=0x000D3902;
	data_array[1]=0x03FF00B4;
	data_array[2]=0x035A035A;
	data_array[3]=0x306a015A;
	data_array[4]=0x0000006a;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(10);

	data_array[0]=0x00023902;
    data_array[1]=0x000007BC;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(10);
	
	data_array[0]=0x00043902;
	data_array[1]=0x010E41BF;
	dsi_set_cmdq(data_array, 2, 1);	
	MDELAY(10);	
	
	//Set VCOM
	data_array[0]=0x00033902;
    data_array[1]=0x005c5cB6;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(10);	

	// Set panel 
	data_array[0]=0x00023902;
	data_array[1]=0x000009CC;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(10);

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
	MDELAY(10);	
	
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
	MDELAY(10);	
	
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
	MDELAY(10);	

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
	MDELAY(10);
	
	data_array[0]=0x00033902;
    data_array[1]=0x001430C0;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(10);	
	
	data_array[0]=0x00053902;
	data_array[1]=0x40C000C7;
	data_array[2]=0x000000C0;
	dsi_set_cmdq(data_array, 3, 1);
	MDELAY(10);

	data_array[0]=0x00023902;
    data_array[1]=0x00008edf;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(10);

	data_array[0]=0x00023902;
    data_array[1]=0x000066d2;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(10);	
	
	data_array[0]= 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(200);
	
	
	data_array[0]= 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(50);  


}



#if 0
static struct LCM_setting_table lcm_set_window[] = {
	{0x2A, 4, {0x00, 0x00, (FRAME_WIDTH >> 8), (FRAME_WIDTH&0xFF) } },
	{0x2B, 4, {0x00, 0x00, (FRAME_HEIGHT >> 8), (FRAME_HEIGHT&0xFF)} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
	/* Sleep Out */
	{0x11, 0, {} },
	{REGFLAG_DELAY, 120, {} },

	/* Display ON */
	{0x29, 0, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};
#endif


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	/* Sleep Mode On */
	{0x10, 0, {} },
	{REGFLAG_DELAY, 120, {} },
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

/**
 * LCM Driver Implementations
 */
void lcm_get_gpio_infor(void)
{
	static struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "mediatek,mt6735-dispsys");

	GPIO_LCD_PWR_EN = of_get_named_gpio(node, "lcm_power_gpio", 0);
	GPIO_LCD_BL_EN = of_get_named_gpio(node, "lcm_bl_gpio", 0);
}

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
	gpio_direction_output(GPIO, output);
	gpio_set_value(GPIO, output);
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
    params->dsi.mode   = SYNC_EVENT_VDO_MODE;//SYNC_EVENT_VDO_MODE;//BURST_VDO_MODE; //SYNC_PULSE_VDO_MODE;
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


		params->dsi.vertical_sync_active				= 6;
		params->dsi.vertical_backporch					= 3;
		params->dsi.vertical_frontporch					= 20;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active			= 6;
		params->dsi.horizontal_backporch				= 48;
		params->dsi.horizontal_frontporch				= 16;
		params->dsi.horizontal_active_pixel			= FRAME_WIDTH;
		
    	params->dsi.ssc_disable							= 1;
		params->dsi.PLL_CLOCK = 221;
		
		params->dsi.cont_clock= 0;		
		params->dsi.clk_lp_per_line_enable = 1;		
}


static void lcm_init(void)
{
	lcm_get_gpio_infor();
	lcm_set_gpio_output(GPIO_LCD_PWR_EN, 1);
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(20);

	//push_table(lcm_initialization_setting,
	//	   sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	init_lcm_registers();
	
	lcm_set_gpio_output(GPIO_LCD_BL_EN, 1);

}


static void lcm_suspend(void)
{
	lcm_get_gpio_infor();

	lcm_set_gpio_output(GPIO_LCD_BL_EN, 0);
	push_table(lcm_deep_sleep_mode_in_setting,
		   sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	lcm_set_gpio_output(GPIO_LCD_PWR_EN, 0);
}


static void lcm_resume(void)
{
	lcm_init();
}

#if 0
static void lcm_setpwm(unsigned int divider)
{
	/* TBD */
}


static unsigned int lcm_getpwm(unsigned int divider)
{
	/* ref freq = 15MHz, B0h setting 0x80, so 80.6% * freq is pwm_clk; */
	/* pwm_clk / 255 / 2(lcm_setpwm() 6th params) = pwm_duration = 23706 */
	unsigned int pwm_clk = 23706 / (1 << divider);

	return pwm_clk;
}
#endif

LCM_DRIVER nt35521_wxga_dsi_vdo_lcm_drv = {
	.name		= "nt35521_wxga_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
#if (LCM_DSI_CMD_MODE)
	/*.set_backlight	= lcm_setbacklight,*/
	/* .set_pwm        = lcm_setpwm, */
	/* .get_pwm        = lcm_getpwm, */
	/*.update         = lcm_update, */
#endif
};
