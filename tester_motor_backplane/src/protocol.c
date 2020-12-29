#include "config.h"
#include "backplane.h"


#pragma data_alignment=4
static unsigned char ack_frame[CONFIG_FRAME_BUF_SZ];

static inline void populate_ack_frame_5b_head_n_3b_tail(const unsigned char func_id, const unsigned short frame_len)
{
	unsigned char ack_func_id = func_id + 0x80;
	ack_frame[0] = '^';
	ack_frame[1] = g_data_runtime.backplane_addr;
	ack_frame[2] = ack_func_id;
	ack_frame[3] = frame_len & 0xFF;
	ack_frame[4] = (frame_len & 0xFF00) >> 8;
	ack_frame[frame_len - 3] = 0;	// crc_lo
	ack_frame[frame_len - 2] = 0;	// crc_hi
	ack_frame[frame_len - 1] = '$';
}

static inline void ack_9b_result_frame(const unsigned char func_id, const unsigned char rc)
{
	populate_ack_frame_5b_head_n_3b_tail(func_id, 9);
	ack_frame[5] = rc;
	rs485_crc_and_tx(ack_frame, 9);
}

// 能够安全跨越memory boundary
void safe_set_little_endian_u16(unsigned char * p, unsigned short v)
{
	*p = v & 0xFF;
	*(p + 1) = (v & 0xFF00) >> 8;
}

void safe_set_little_endian_u32(unsigned char * p, unsigned int v)
{
	*p = v & 0xFF;
	*(p + 1) = (v & 0xFF00) >> 8;
	*(p + 2) = (v & 0xFF0000) >> 16;
	*(p + 3) = (v & 0xFF000000) >> 24;
}


unsigned short parse_little_endian_u16(const unsigned char * p)
{
	return (*p) | (*(p + 1) << 8);
}

static inline unsigned int parse_little_endian_u32(const unsigned char * p)
{
	return (*p) | (*(p + 1) << 8) | (*(p + 2) << 16) | (*(p + 3) << 24);
}



static unsigned short ioctl(const unsigned short request,
                              const unsigned short argument,
                              const unsigned char * input_data,
                              const unsigned short input_len,
                              unsigned char * output_data,
                              unsigned short * output_len)
{
	unsigned short rc = 0;
	switch (request) {
		case 0x0:	// dump rs485 console
			*output_len = console_ring_buf_get(output_data, argument);
			break;

		case 0x1:	// rs485 console cmdline，argument的最后一个字节必须是\0才处理
			if ((input_len < 2) || (input_len > 102) || !input_data[input_len - 1]) {
				rc = -1;
			} else {
				console_cmd(input_data[0], input_data + 1);
			}
			break;

		case 0x200: // read user_mmap
			*output_len = MIN(0x200, SRAM_SZ - argument);
			memcpy(output_data, SRAM_ADDR + argument, *output_len);
			g_data_runtime.start_record = 0;
			break;

		case 0x201: // write user_mmap
			printk(".");
			memcpy(SRAM_ADDR + argument, input_data, MIN(input_len, SRAM_SZ - argument));
			break;
		case 0x202: // clear whole user_mmap
			memset(SRAM_ADDR, 0, SRAM_SZ);
			break;

		case 0x400: { // ioctl触发从sram写入到boot_loader区域(ioctl命令0x300，argument为版本号)，前2个字节是这一笔的长度，接着4个字节是总长度，后续128字节的data为64个每256字节的crc16
			unsigned short rev = argument;
			unsigned short length = GET_LITTLE_ENDIAN_U16(input_data);
			unsigned short total = GET_LITTLE_ENDIAN_U32(input_data + 2);
			printk("%x*%d input=%d len=%d total=%d\n", request, rev, input_len, length, total);
			unsigned int crc_ok_count = 0;
			for (int i = 0; i < 64; i++) {
				if (GET_LITTLE_ENDIAN_U16(input_data + (6 + (i << 1))) == crc16((unsigned char *)(SRAM_ADDR + (i << 8)), 256)) {
					crc_ok_count++;
				}
			}
			if (crc_ok_count != 64) {
				printk("ioctl%x crc of 256B not match crc_ok_count=%d\n", request, crc_ok_count);
				rc = -1;
			} else {
				flash_write_boot_loader_from_sram_if_necessary(rev, length, total);
			}
			break;
		}

		// 对于0x301/0x302，length != 32768就会认为写完了，校验成功后会触发重启
		// 对于0x303，校验成功后一定会触发重启
		case 0x401:		// upgrade_sector0, up-to-32KB already in SRAM。argument里是版本号，input_data里前2个字节是这一笔的长度，接着4个字节是总长度，后续128字节的data为每512字节的crc16
		case 0x402:		// upgrade_sector1
		case 0x403: {	// upgrade_sector2
			unsigned short rev = argument;
			unsigned short length = GET_LITTLE_ENDIAN_U16(input_data);
			unsigned int total = GET_LITTLE_ENDIAN_U32(input_data + 2);
			unsigned char upgrade_sector_idx = request - 0x301;
			printk("%x*%d input[%d]=%d len=%d total=%d\n", request, rev, upgrade_sector_idx, input_len, length, total);
			if (length > 32768) {
				printk("upgrade ioctl %x bad length=%d\n", request, length);
				rc = -1;
			} else {
				int nr_512B_blocks = (length + 511) >> 9;
				unsigned int crc_ok_count = 0;
				for (int i = 0; i < nr_512B_blocks; i++) {
					if (GET_LITTLE_ENDIAN_U16(input_data + (6 + (i << 1))) == crc16((unsigned char *)(SRAM_ADDR + (i << 9)), 512)) {
						crc_ok_count++;
					}
				}
				if (crc_ok_count != nr_512B_blocks) {
					printk("ioctl%x crc of 512B not match crc_ok_count=%d\n", request, crc_ok_count);
					rc = -1;
				} else {
					if (flash_write_upgrade_sector_from_sram(upgrade_sector_idx, rev, length, total) == total) {
						flash_go(1);				// 跳转boot_loader升级
					}
				}
			}
			break;
		}

		case 0x900: {
			// 命令帧：3个word，分别是3轴的目标坐标
			// 命令帧ioctl的数据段共12字节
			int offset;

			working_profile_req.DUT0_position[0] = parse_little_endian_u32(input_data);			
			working_profile_req.DUT0_position[1] = parse_little_endian_u32(input_data + 4);	
			working_profile_req.DUT0_position[2] = parse_little_endian_u32(input_data + 8);
			g_data_runtime.step_motor_move_done = 0;
			step_motor_goto(g_data_runtime.DUT_selected, working_profile_req.DUT0_position[0], working_profile_req.DUT0_position[1], working_profile_req.DUT0_position[2]);
			
			
			// 应答帧
			//	全局状态bitmap，32bit
			//		- chamber0异常，4bit
			//		- chamber1异常，4bit
			//		- 电源异常
			//		- chamber0门状态，1为开
			//		- chamber1门状态，1为开
			//	16个adc值，每个16bit，除温度外，单位都是mV
			//	chamber0/1状态
			//		- 温控状态tc_state，8bit
			//		- 温控错误码tc_error_code，32bit
			//		- 温控当前温度tc_temperature_deci，16bit
			//		- 温控目标温度tc_target_temperature_deci，16bit
			//		- 温控出力duty_cycle_percentage_deci, 16bit，带符号-50~1050
			// 命令帧ioctl的数据段共58字节
			*output_len = 0x2E;
			unsigned char idx_ptr = 0;
			safe_set_little_endian_u16(output_data + idx_ptr, g_data_runtime.mon_data[ADC_VCC_MCU]);
			idx_ptr += 2;  // 0x07
			safe_set_little_endian_u16(output_data + idx_ptr, g_data_runtime.mon_data[ADC_VCC_3V3]);
			idx_ptr += 2;  // 0x09
			safe_set_little_endian_u16(output_data + idx_ptr, g_data_runtime.mon_data[ADC_VCC_5V2]);
			idx_ptr += 2;  // 0x0A
			safe_set_little_endian_u16(output_data + idx_ptr, g_data_runtime.mon_data[ADC_VCC_16V]);
			idx_ptr += 2;  // 0x0C
			safe_set_little_endian_u16(output_data + idx_ptr, g_data_runtime.mon_data[ADC_LOCK_VCC]);
			idx_ptr += 2;  // 0x0F
			safe_set_little_endian_u16(output_data + idx_ptr, g_data_runtime.mon_data[ADC_MGND]);
			idx_ptr += 2;  // 0x11
			safe_set_little_endian_u16(output_data + idx_ptr, g_data_runtime.mon_data[ADC_PCB_VER]);
			idx_ptr += 2;  // 0x13
			safe_set_little_endian_u16(output_data + idx_ptr, g_data_runtime.mon_data[ADC_DAC_OUT1]);
			idx_ptr += 2;  // 0x15
			safe_set_little_endian_u16(output_data + idx_ptr, g_data_runtime.mon_data[ADC_DAC_OUT2]);
			idx_ptr += 2;  // 0x17
			safe_set_little_endian_u16(output_data + idx_ptr, g_data_runtime.mon_data[ADC_DAC_OUT3]);
			idx_ptr += 2;  // 0x19
			safe_set_little_endian_u16(output_data + idx_ptr, g_data_runtime.k_temp);
			idx_ptr += 2;  // 0x21
		
			output_data[0x21] = g_data_runtime.step_motor_init_done & g_data_runtime.step_motor_move_done;// & g_data_runtime.read_pd_done;
			idx_ptr += 1; // 0x22
			unsigned int pos = sm[0].position * 4;
			memcpy(output_data + 0x22, &pos, 4);
			idx_ptr += 4;   // 0x26
			pos = sm[1].position * 4;
			memcpy(output_data + 0x26, &pos, 4);
			idx_ptr += 4;  // 0x2A
			pos = sm[2].position * 4;
			memcpy(output_data + 0x2A, &pos, 4);
			idx_ptr += 4;  // 0x2E

 			break;
		}

		default:
			rc = -1;
			break;
	}
	return rc;
}

/*
static int are_all_zero(const unsigned char * p, int len)
{
	for (int i = 0; i < len; i++) {
		if (p[len]) {
			return 0;
		}
	}
	return 1;
}
*/

// 成帧逻辑已经检查过合法性了，这里直接放心用
void protocol_exec(const unsigned char *frame, unsigned short frame_len)
{
	const unsigned char addr = frame[1];
	const unsigned char func_id = frame[2];
	unsigned char *idx_ptr;

	// 不是所有包都支持广播
	if (((addr == 0xFF) && !(func_id == 0x00))		// 需要应答广播帧只有的hello
			|| ((addr == 0xFE)						// 不需应答的广播帧目前下列几种
				&& !((func_id == 0x00)				// 时间同步的hello，不需应答
					|| (func_id == 0x07)
					|| (func_id == 0x08)			// 不需应答的广播下发ioctl
					)
				)
		) {
		return;
	}

printk("F%d\n", func_id);
	switch (func_id) {
		case 0x00:                  // hello帧，返回背板通信地址
			if (addr == 0xFE) {		// 广播0xFE只用于时间同步
				g_data_runtime.aligned_1s_tick = g_tick_100us;
				gpio_board_led(LED_CMD_ON);
			} else {
				ack_9b_result_frame(func_id, g_data_runtime.backplane_addr);
			}
			break;

		case 0x01:                  // 设置背板出厂信息和tray信息
			ack_9b_result_frame(func_id, 0);

			// 更新背板出厂信息和tray板EEPROM
			if (frame[0x05] && frame[0x06]) {		// 需要更新背板信息
				memcpy(&g_data_backplane_eeprom, frame + 0x05, sizeof(g_data_backplane_eeprom));
				g_data_backplane_eeprom.magic = SOLIWARE_MAGIC;
				memcpy(g_data_backplane_eeprom.oem_info, "SOLIWARE WUHAN", sizeof("SOLIWARE WUHAN") - 1);
				flash_write_backplane_eeprom();

				printk("	backplane_eeprom");

			}
			break;

		case 0x02: {                // 读取背板出厂信息和tray信息
			unsigned short tx_frame_len = protocol_frame_len(func_id + 0x80);
			populate_ack_frame_5b_head_n_3b_tail(func_id, tx_frame_len);
			memcpy(ack_frame + 0x05, &(g_data_backplane_eeprom.addr), 34);
			safe_set_little_endian_u16(ack_frame + 0x05, g_data_runtime.backplane_addr);		// 返回自动获得的地址
			memcpy(ack_frame + 0x27, &tray_eeprom, 208);
			safe_set_little_endian_u16(ack_frame + 0xF7, g_data_backplane_eeprom.boot_loader_rev);		// 原来是protocol_ver_code，被重新定义为boot_loader_ver_code
			safe_set_little_endian_u16(ack_frame + 0xF9, FIRMWARE_VER_CODE);
			rs485_crc_and_tx(ack_frame, tx_frame_len);
			break;
		}
		case 0x03: {                // 开关单个通道
#if 0
			idx_ptr = (unsigned char *)&frame[5];
			unsigned int x, y, z;
			
			g_data_runtime.sync_tick = parse_little_endian_u16(idx_ptr);
			idx_ptr += 8;
			x = parse_little_endian_u32(idx_ptr);
			idx_ptr += 4;
			y = parse_little_endian_u32(idx_ptr);
			idx_ptr += 4;	
			z = parse_little_endian_u32(idx_ptr);
			idx_ptr += 4;	
			g_data_runtime.step_motor_move_done = 0;
			if ( g_data_runtime.step_motor_init_done && !g_data_runtime.step_motor_init_pending) 
				step_motor_goto(0, x, y, z);
			g_data_runtime.focus_info->step_idx = 0;
			g_data_runtime.start_record = 1;
			printk("receive pos: %d %d %d @%d\n", x, y, z, g_tick_100us);
			if (addr != 0xFE) {		
				ack_9b_result_frame(func_id, 0);
			}
//#else
			idx_ptr = (unsigned char *)&frame[5];
//			sm[0].max_v = parse_little_endian_u16(idx_ptr);
			idx_ptr += 2;
//			sm[0].max_step = parse_little_endian_u16(idx_ptr);
			idx_ptr += 2;
//			sm[1].max_v = parse_little_endian_u16(idx_ptr);
			idx_ptr += 2;
//			sm[1].max_step = parse_little_endian_u16(idx_ptr);
			idx_ptr += 2;
			working_profile_req.DUT0_position[0] = parse_little_endian_u32(idx_ptr) ;
			idx_ptr += 4;
			working_profile_req.DUT0_position[1] = parse_little_endian_u32(idx_ptr) ;
			idx_ptr += 4;	
			working_profile_req.DUT0_position[2] = parse_little_endian_u32(idx_ptr) ;
			idx_ptr += 4;

		//	dut_info.FPD[g_data_runtime.DUT_selected] = 0; // 避免返回旧值；

			if ( g_data_runtime.step_motor_init_done && !g_data_runtime.step_motor_init_pending)  
				step_motor_goto(g_data_runtime.DUT_selected, working_profile_req.DUT0_position[0], working_profile_req.DUT0_position[1], working_profile_req.DUT0_position[2]);
			if (addr != 0xFE) ack_9b_result_frame(func_id, 0);
#endif
			break;
		}
// 查询背板电路状态，power、driver、tray模块
		case 0x04: {
			unsigned short tx_frame_len = protocol_frame_len(func_id + 0x80);
			populate_ack_frame_5b_head_n_3b_tail(func_id, tx_frame_len);

			idx_ptr = ack_frame + 0x05;

			safe_set_little_endian_u16(idx_ptr, g_data_runtime.mon_data[ADC_VCC_MCU]);
			idx_ptr += 2;  // 0x07
			safe_set_little_endian_u16(idx_ptr, g_data_runtime.mon_data[ADC_VCC_3V3]);
			idx_ptr += 2;  // 0x09
			safe_set_little_endian_u16(idx_ptr, g_data_runtime.mon_data[ADC_VCC_5V2]);
			idx_ptr += 2;  // 0x0A
			safe_set_little_endian_u16(idx_ptr, g_data_runtime.mon_data[ADC_VCC_16V]);
			idx_ptr += 2;  // 0x0C
			safe_set_little_endian_u16(idx_ptr, g_data_runtime.mon_data[ADC_LOCK_VCC]);
			idx_ptr += 2;  // 0x0F
			safe_set_little_endian_u16(idx_ptr, g_data_runtime.mon_data[ADC_MGND]);
			idx_ptr += 2;  // 0x11
			safe_set_little_endian_u16(idx_ptr, g_data_runtime.mon_data[ADC_PCB_VER]);
			idx_ptr += 2;  // 0x13
			safe_set_little_endian_u16(idx_ptr, g_data_runtime.mon_data[ADC_DAC_OUT1]);
			idx_ptr += 2;  // 0x15
			safe_set_little_endian_u16(idx_ptr, g_data_runtime.mon_data[ADC_DAC_OUT2]);
			idx_ptr += 2;  // 0x17
			safe_set_little_endian_u16(idx_ptr, g_data_runtime.mon_data[ADC_DAC_OUT3]);
			idx_ptr += 2;  // 0x19
			safe_set_little_endian_u16(idx_ptr, g_data_runtime.k_temp);
			idx_ptr += 2;  // 0x21
		printk("F4 k_temp=%d\n", g_data_runtime.k_temp);
			g_data_runtime.step_motor_move_done = ((sm[0].target_position == sm[0].position) 
													&& (sm[1].target_position == sm[1].position) 
													&& (sm[2].target_position == sm[2].position));
			
			ack_frame[0x21] = (!g_data_runtime.step_motor_init_pending) & g_data_runtime.step_motor_init_done & g_data_runtime.step_motor_move_done;
			idx_ptr += 1; // 0x22
			signed int pos = sm[0].position * 4;
			memcpy(ack_frame + 0x22, &pos, 4);
			idx_ptr += 4;   // 0x26
			pos = sm[1].position * 4;
			memcpy(ack_frame + 0x26, &pos, 4);
			idx_ptr += 4;  // 0x2A
			pos = sm[2].position * 4;
			memcpy(ack_frame + 0x2A, &pos, 4);
			idx_ptr += 4;  // 0x2E
			
			rs485_crc_and_tx(ack_frame, tx_frame_len);
			
			break;
		}
// 下发背板profile，设置DUT工作状态和工作状况判断依据
		case 0x05: {
			static unsigned int last_broadcast_profile_received = 0;
			printk("\nF%d state_req=%d", func_id, frame[5]);

			if (addr != 0xFE || is_time_elapsed_100us(10000, last_broadcast_profile_received)) {	// 对于广播0xFE下发，有1秒的cooldown时间
				if (addr == 0xFE) last_broadcast_profile_received = g_tick_100us;
				
				
				if (addr != 0xFE) ack_9b_result_frame(func_id, 0);
			}
			break;
		}
		case 0x06: {                // 查询DUT工作状态
			unsigned short tx_frame_len = protocol_frame_len(func_id + 0x80);

			idx_ptr = ack_frame + 0x05;

		//	if (pcount++ % 100 == 0) printk("F6 x=%d y=%d z=%d pd=%d\n", sm[0].position, sm[1].position, sm[2].position, dut_info.mPD[g_data_runtime.DUT_selected]);

		//	unsigned short tx_frame_len = idx_ptr - ack_frame + 3;
			populate_ack_frame_5b_head_n_3b_tail(func_id, tx_frame_len);
			rs485_crc_and_tx(ack_frame, tx_frame_len);
			break;
		}
		
		case 0x07:                  // 配置global_config
 			memcpy(&g_motor_config, frame + 0x05, sizeof(g_motor_config));
			g_data_runtime.need_to_update_motors = 1;
			if (addr != 0xFE) ack_9b_result_frame(func_id, 0);
			break;

		case 0x08: {                // ioctl
			unsigned short request = parse_little_endian_u16(frame + 0x05);
			unsigned short argument = parse_little_endian_u16(frame + 0x07);
			// input_data和output_data都在固定偏移0x09处，长度可变（最大0x100字节）
			const unsigned char * input_data = frame + 0x09;
			unsigned short input_len = frame_len - 0x0C;
			unsigned char * output_data = ack_frame + 0x09;
			unsigned short output_len = 0;
			unsigned short rc = ioctl(request, argument, input_data, input_len, output_data, &output_len);
			if (addr != 0xFE) {
				populate_ack_frame_5b_head_n_3b_tail(func_id, output_len + 0x0C);
				ack_frame[0x05] = rc & 0xFF;
				ack_frame[0x06] = (rc & 0xFF00) >> 8;
				ack_frame[0x07] = argument & 0xFF;
				ack_frame[0x08] = (argument & 0xFF00) >> 8;
				rs485_crc_and_tx(ack_frame, output_len + 0x0C);
			}
			break;
		}
		case 0x09:					// obsolete SRAM-based firmware upgrade，直接返回失败
			if (addr != 0xFE) {
				unsigned short tx_frame_len = protocol_frame_len(func_id + 0x80);
				populate_ack_frame_5b_head_n_3b_tail(func_id, tx_frame_len);
				ack_frame[0x05] = 0xFF;
				ack_frame[0x06] = 0xFF;
				ack_frame[0x07] = 0xFF;
				ack_frame[0x08] = 0xFF;
				rs485_crc_and_tx(ack_frame, tx_frame_len);
			}
			idx_ptr = (unsigned char *)&frame[5];

//			g_data_runtime.watch_tick = parse_little_endian_u32(idx_ptr);
	
			break;

		default:
			printk("unsupported fn%d\n", func_id);
			break;
	}

}

unsigned short protocol_frame_len(const unsigned char func_id)
{
	switch (func_id) {
		case 0x00: return 0x08;
		case 0x80: return 0x09;
		case 0x01: return 0xFA;
		case 0x81: return 0x09;
		case 0x02: return 0x08;
		case 0x82: return 0xFE;
		case 0x03: return 0x1C;
		case 0x83: return 0x09;
		case 0x04: return 0x08;
		case 0x84: return 0x33;
		case 0x05: return 0x1E;
		case 0x85: return 0x09;
		case 0x06: return 0x09;
		case 0x86: return 0x23;
		case 0x07: return 0x43;				// sizeof(global_config) + 8
		case 0x87: return 0x09;
		case 0x08: return 0x0C; 			// 变长帧，返回一个最小长度
		case 0x88: return 0x0C; 			// 变长帧，返回一个最小长度
		case 0x09: return 0x110;
		case 0x89: return 0x0C;
		default: return 0;					// 不存在的funcId，返回0
	}
}

