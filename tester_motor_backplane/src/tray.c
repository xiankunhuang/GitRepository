#include "config.h"
#include "backplane.h"



#define TRAY_EEPROM_BUS_ID	2
#define TRAY_EEPROM_BUS_MUX_CHANNEL	3


#define MN_8472_ADDR		0x70
#define MN_8436_ADDR		0xF0


void tray_soft_unplug()
{
	printk("tray_soft_unplug %d\n", g_tick_100us);

	tray_init();

}

void tray_update_state()
{
	static unsigned int first_tick_plugged = 0;
	static int is_first_tick_set = 0;

	unsigned char _idx =6;	// 一共6次机会；

	int tray_plugged = gpio_is_tray_plugged_in();

	if (g_data_tray.is_plugged_in) {
		if(0 == g_data_runtime.has_tray_eeprom)
			g_data_tray.need_to_update_eeprom = 0;
	
		if (g_data_tray.need_to_update_eeprom) {	// 如果需要更新tray eeprom
			while (_idx) {
				_idx--;
				if (0 != i2c_set_PCA9548_blocking(TRAY_EEPROM_BUS_ID, I2C_MUX_SLA, TRAY_EEPROM_BUS_MUX_CHANNEL)){// tray eepromi2c_mux的channel-3）
					gpio_p_reset();
					printk("set_PCA9548 err\n");
					continue;
				}
				break;
			}

			while (_idx) {
				_idx--;
				if ((i2c_write_AT24C16A_blocking(TRAY_EEPROM_BUS_ID, TRAY_EEPROM_SLA , (unsigned char *)(&tray_eeprom))) !=0 )
					continue;

				printk("tray eeprom update ok, soft-unplug to apply\n");
				tray_soft_unplug();
				// eeprom 写完之后要等5ms
				IODELAY(20000);// 每次循环大约0.25us；
				first_tick_plugged = g_tick_100us;
				break;
			}
			
			if (0 ==_idx) {
				g_data_runtime.has_tray_eeprom = 0;
				printk("tray eeprom update failed\n");
			}

			g_data_tray.need_to_update_eeprom = 0;
		}

		return;
	}

	if (!g_data_tray.is_plugged_in && !tray_plugged) {
		is_first_tick_set = 0;
		return;
	}

	// 检查到插入了// 初次计时
	if (!is_first_tick_set) {
		is_first_tick_set = 1;
		first_tick_plugged = g_tick_100us;
	}



	// 延时防抖，连续0.5s都检查到插入才认为插入了
	if (is_time_elapsed_100us(5000, first_tick_plugged)){
		// tray插入时，复位各种标志后，关闭dut开关，打开电源模块，然后开始做plugged_in_scan
		// 注意plugged_in_scan可能分很多步骤，每个步骤都需要在保持电源模块打开的情况下，设置自己需要的状态

		i2c_finish_session_forcibly(TRAY_EEPROM_BUS_ID);

		tray_init();
		g_data_tray.is_plugged_in = 1;	
		g_data_runtime.has_tray_eeprom = 1;
		_idx = 10;

		while(_idx)
		{
			
			_idx--;
			if (0 != i2c_set_PCA9548_blocking(PCAL6416A_BUS_ID, I2C_MUX_SLA, 3)) {// tray eepromi2c_mux的channel-4）
				gpio_p_reset();
				printk("set_PCA9548 err\n");
				continue;
			}
			break;
		}
	
		while(_idx)
		{
			_idx--;
			if ((i2c_read_AT24C16A_blocking(TRAY_EEPROM_BUS_ID, TRAY_EEPROM_SLA , (unsigned char *)(&tray_eeprom))) !=0 )
				continue;

			unsigned char *_tray_eeprom = (unsigned char *)(&tray_eeprom);

			for(int i = 0; i < sizeof(tray_eeprom); i++)
				if (*(_tray_eeprom + i) == 0xFF) 
					*(_tray_eeprom + i) = 0;	// 未设置的值写为0

			break;
		}

		if ( 0 ==_idx) {
			g_data_runtime.has_tray_eeprom = 0;
			printk("read tray eeprom failed\n");
		}

		i2c_finish_session_forcibly(TRAY_EEPROM_BUS_ID);

		printk("plugged_in_scan done %d\n", g_tick_100us);
	}
}


/*
static int is_valid_vendor_sn(unsigned char * sn_buf)
{
	if (!sn_buf)
		return 0;
	for (int i = 0; i < DUT_SN_SIZE; i++) {
		if ((sn_buf[i] < 32) || (sn_buf[i] > 126))		// 32~126 printable ascii
			return 0;
	}
	return 1;
}
*/


void tray_init()
{
	memset(&g_data_tray, 0, sizeof(g_data_tray));
	memset(&tray_eeprom, 0, sizeof(tray_eeprom));

	g_data_tray.eeprom = (unsigned char *)(&tray_eeprom);
	g_data_runtime.has_tray_eeprom = 0;
	g_data_tray.is_new_tray = 1;
	
	g_data_tray.DUT_OE[0] = 0xFFFF;
	g_data_tray.DUT_OE[1] = 0xFFFF;
	g_data_tray.DUT_OE[2] = 0xFFFF;
	g_data_tray.DUT_OE[3] = 0xFFFF;
	g_data_tray.DUT_OE[4] = 0xFFFF;
	g_data_tray.DUT_OE[5] = 0xFFFF;
}

void tray_print()
{
	unsigned char copied_tray_hw_rev[17];
	unsigned char copied_tray_sn[17];
	unsigned char copied_tray_sign[17];
	memcpy(copied_tray_hw_rev, g_data_tray.eeprom, 16);
	copied_tray_hw_rev[16] = 0;
	memcpy(copied_tray_sn, g_data_tray.eeprom + 16, 16);
	copied_tray_sn[16] = 0;
	printk("tray=%d type=%d ee=%d rev=%s sn=%s\n", g_data_tray.is_plugged_in, g_data_tray.type, g_data_tray.is_eeprom_valid, copied_tray_hw_rev, copied_tray_sn);

	memcpy(copied_tray_sign, g_data_tray.eeprom + TRAY_EEPROM_CMD_OFFSET, 16);
	copied_tray_sign[16] = 0;
	
}




