#include "config.h"
#include "backplane.h"

#define ADDR_6416_SET					2
#define ADDR_6416_DIR					6
#define ADDR_6416_DRIVE_STRENGTH_START	0x40
#define ADDR_6416_OPEN_DRAIN			0x4F

// dir：值为1的bit是输入，值为0是输出
// value：值为1的bit是高电平，值为0是低电平
// 6个PCAL6416, 其中：
// chip0：0, 0x20， DUT_EN0~15, chip1：0, 0x21， DUT_EN16~31
// chip2：0, 0x20， DUT_EN1~16, chip3：0, 0x21， DUT_EN16~31
// chip4：0, 0x20， LD_CONFIG, PD_CONFIG, chip5：0, 0x21， EML_CONFIG


void pcal6416a_init(pcal6416a_t * chip6416, unsigned char bus_id, unsigned char i2c_mux_sla, unsigned char i2c_mux_channel, unsigned char sla, unsigned short dir, unsigned short value)
{
	unsigned char drive_strengh_5[] = { ADDR_6416_DRIVE_STRENGTH_START, 0, 0, 0, 0 };		// 用0.25x drive strength，省一点电
	unsigned char dir_3[] = { ADDR_6416_DIR, dir & 0xFF, (dir & 0xFF00) >> 8 };
	unsigned char value_3[] = { ADDR_6416_SET, value & 0xFF, (value & 0xFF00) >> 8 };

	memset(chip6416, 0, sizeof(pcal6416a_t));
	chip6416->bus_id = bus_id;
	chip6416->i2c_mux_sla = i2c_mux_sla;
	chip6416->i2c_mux_channel = i2c_mux_channel;
	chip6416->sla = sla;
	chip6416->dir = dir;
	chip6416->value = value;
	chip6416->value_expected = value;

	int rc = 0;
	if (i2c_mux_sla) rc += i2c_set_PCA9548_blocking(bus_id, i2c_mux_sla, i2c_mux_channel);
	rc += i2c_write_n_bytes_blocking(bus_id, sla, drive_strengh_5, 5);
	rc += i2c_write_n_bytes_blocking(bus_id, sla, dir_3, 3);
	rc += i2c_write_n_bytes_blocking(bus_id, sla, value_3, 3);
	printk("pcal6416a %p init %s value=%x rc=%d\n", chip6416, (rc == 0) ? "ok" : "failed", value, rc);
}

void pcal6416a_set_if_needed(pcal6416a_t *chip6416, unsigned short value_expected)
{
	chip6416->value_expected = value_expected;
	if (chip6416->value != chip6416->value_expected) {
		debug("pcal6416a %p %x->%x ", chip6416, chip6416->value, chip6416->value_expected);
		unsigned char value_3[] = { ADDR_6416_SET, chip6416->value_expected & 0xFF, (chip6416->value_expected & 0xFF00) >> 8 };
		i2c_finish_session_forcibly(chip6416->bus_id);
		if (chip6416->i2c_mux_sla) i2c_set_PCA9548_blocking(chip6416->bus_id, chip6416->i2c_mux_sla, chip6416->i2c_mux_channel);
		if (i2c_write_n_bytes_blocking(chip6416->bus_id, chip6416->sla, value_3, 3) == 0) {
			chip6416->value = chip6416->value_expected;
		}
		debug((chip6416->value == chip6416->value_expected) ? "ok\n" : "failed\n");
	}
}

int pcal6416a_need_to_set(pcal6416a_t *chip6416)
{
	return chip6416->value != chip6416->value_expected;
}

void pcal6416a_gpio_init()
{
	pcal6416a_init(chip6416_array + 0, PCAL6416A_BUS_ID, I2C_MUX_SLA, 0, 0x20, 0, 0xFFFF);
	pcal6416a_init(chip6416_array + 1, PCAL6416A_BUS_ID, I2C_MUX_SLA, 0, 0x21, 0, 0xFFFF);
	pcal6416a_init(chip6416_array + 2, PCAL6416A_BUS_ID, I2C_MUX_SLA, 1, 0x20, 0, 0xFFFF);
	pcal6416a_init(chip6416_array + 3, PCAL6416A_BUS_ID, I2C_MUX_SLA, 1, 0x21, 0, 0xFFFF);
	pcal6416a_init(chip6416_array + 4, PCAL6416A_BUS_ID, I2C_MUX_SLA, 2, 0x20, 0, 0xFFFF);
	pcal6416a_init(chip6416_array + 5, PCAL6416A_BUS_ID, I2C_MUX_SLA, 2, 0x21, 0, 0xFFFF);
}

// 这次 PCAL6416 没有装在 iic mux 上
void PCAL6416A_poll( )
{
	static unsigned short dut_onoff[6];
	static unsigned char iic_mux_err=0;

	unsigned char buf[3];
	unsigned char idx, addr;

	for (idx = 0; idx < NR_PCAL6416A; idx++) {
		if (g_data_tray.DUT_OE[idx] == dut_onoff[idx])  // 变量需要吗
			continue;

		gpio_p_reset();
		if (0 != i2c_set_PCA9548_blocking(PCAL6416A_BUS_ID, I2C_MUX_SLA, idx/2)) {
			printk("set_PCA9548 err\n");
			iic_mux_err++;	
		}

		addr = (idx % 2 == 0) ? PCAL6416A_ADDR_L : PCAL6416A_ADDR_H;
		buf[0] = ADDR_6416_P0;
		buf[1] = (unsigned char)(g_data_tray.DUT_OE[idx] & 0xFF);
		buf[2] = (unsigned char)(g_data_tray.DUT_OE[idx] >> 8);
		printk("set_PCAL6416[%d] %x %x %x %x \n", idx, addr, buf[0], buf[1], buf[2]);
		if (0> i2c_write_n_bytes_blocking(PCAL6416A_BUS_ID, addr, buf, 3)) {
		 	i2c_finish_session_forcibly(PCAL6416A_BUS_ID); 
			printk("set_PCAL6416[%d] %x %x %x %x err\n", idx, addr, buf[0], buf[1], buf[2]);
			iic_mux_err++;
		}
		dut_onoff[idx] = g_data_tray.DUT_OE[idx];
	}

	
}

//todo 重试3遍，不行返回失败；
  void PCAL6416A_init( )
{
	unsigned char buf[3];
	unsigned char idx, _err, addr;

	for (idx = 0; idx < NR_PCAL6416A; idx++) {
		printk("idx=%d\n",idx);
	//	s_idx = idx / 2;
		_err = 0;
		
		if (0 > i2c_set_PCA9548_blocking(PCAL6416A_BUS_ID, I2C_MUX_SLA, idx/2)) {
			printk("set_PCA9548 err\n");
		}

		addr = (idx % 2 == 0) ? PCAL6416A_ADDR_L : PCAL6416A_ADDR_H;
		
		buf[0]= 0x40;
		buf[1]= 0x00;  
		buf[2]= 0x00;   
		if (0> i2c_write_n_bytes_blocking(PCAL6416A_BUS_ID, addr, buf, 3)) {
			printk("set_PCAL6416 err\n");
			_err++;
		}
		
		buf[0]= ADDR_6416_P0;
		buf[1]= 0xff;  // 先关闭；
		buf[2]= 0xff;   
		if (0 > i2c_write_n_bytes_blocking(PCAL6416A_BUS_ID, addr, buf, 3)) {
			printk("set_PCAL6416 err\n");
			_err++;
		}

		buf[0]= ADDR_6416_DIR;
		buf[1]= 0x00;  // 16个口都输出
		buf[2]= 0x00;
		if (0> i2c_write_n_bytes_blocking(PCAL6416A_BUS_ID, addr, buf, 3)) {
			printk("set_PCAL6416 err\n");
			_err++;
		}

		g_data_runtime.channel_good[idx] = (_err >0) ? 0x00 : 0xFFFF ;
		printk("DRV_hardware_good=%x\n",g_data_runtime.channel_good[idx]);
	}
}



void hivoltage_mcp47_init()
{
	// 配置mcp47，i2c0, 地址0x63，12bit双通道：通道0是电流，通道1是电压
	// 电流每格对应1uA，取值0~3000uA
	// 电压是在3V上的分压，Vcc100 = (Vset - 0.1) * 301 / 5
	// 因此电压调节每格约4.4mV，dac值(v)与期望电压(mV)的对应关系为 v = (mV * 20480 / 301 + 100) / 3000
	// mV不大于100000，因此直接按上述公式的计算顺序，对于int32/uint32不会溢出
	unsigned char bus_id = 1;
	unsigned char da0[] = { 0x08 << 3, 0x00, 0x0A };		// 1. 两个通道都unbuf，reg0x08写1010b
	unsigned char da1[] = { 0x09 << 3, 0x00, 0x00 };		// 2. 保持打开
	unsigned char da2[] = { 0x00 << 3, 0x00, 0x00 };		// 5. channel0 配置为0
	unsigned char da3[] = { 0x01 << 3, 0x00, 0x00 };		// 6. channel1 配置为0
	i2c_session_request_t req[] = {
		{ 1, 0x63, da0, sizeof(da0) },
		{ 1, 0x63, da1, sizeof(da1) },
		{ 1, 0x63, da2, sizeof(da2) },
		{ 1, 0x63, da3, sizeof(da3) },
		I2C_SESSION_REQUEST_END
	};
	while (i2c_do_session_blocking(bus_id, req) != 0) {
		printk("mcp47 init failed, retry\n");
	}
	printk("mcp47 init ok\n");
}


void hivoltage_set_current_dac(unsigned int dac_value)
{
	unsigned char bus_id = 1;
	unsigned char reg_addr = 1;
	unsigned char da[] = { reg_addr << 3, dac_value >> 8, dac_value & 0xFF };
	i2c_finish_session_forcibly(bus_id);
	while (i2c_write_n_bytes_blocking(bus_id, 0x63, da, sizeof(da)) != 0){
		printk("mcp47 set %x failed, retry\n", dac_value);
	}
//	printk("mcp47 set %d ok\n", dac_value);
	g_data_runtime.set_current_finished = 1;
	g_data_tray.DAC_need_update = 0;
}


#define MCP47_BUS_ID 1
#define MCP47_ADDR	0x63 //0X63 //0x60  // 0X60 ~ 0X63


#define MCP47_VREF_VREF_BUF 0x03
#define MCP47_VREF_VREF_UNBUF 0x02
#define MCP47_VREF_BANDGAP 0x01
#define MCP47_VREF_VDD 0x00


#define MCP47_VREF_MEM_ADDR 0x08


#define MCP47_POWERDOWN_1K 0x01
#define MCP47_POWERDOWN_NORMAL 0x00
#define MCP47_POWERDOWN_MEM_ADDR 0x09


#define MCP47_GAIN0_2X 0x01
#define MCP47_GAIN1_2X 0x02
#define MCP47_GAIN_MEM_ADDR 0x0a


/***********************************
MCP47FVB22A3,  8/10/12 bit dual DAC

ADDRESS 00h		DAC0 Output value(default:0x0000)

ADDRESS 01h		DAC1 Output value(default:0x0000)

ADDRESS 08h		DAC Voltage Reference Control bits(default:00)
bit 3:2 DAC1
bit 1:0 DAC0
11 = VREF pin (Buffered); VREF buffer enabled.
10 = VREF pin (Unbuffered); VREF buffer disabled.
01 = Internal Band Gap (1.22V typical); VREF buffer enabled. VREF voltage driven when powered-down.
00 = VDD (Unbuffered); VREF buffer disabled. Use this state with Power-down bits for lowest current.

ADDRESS 09h		DAC Power-Down Control bits(default:00)
bit 3:2 DAC1
bit 1:0 DAC0
11 = Powered Down - VOUT is open circuit.
10 = Powered Down - VOUT is loaded with a 100 k resistor to ground.
01 = Powered Down - VOUT is loaded with a 1 k resistor to ground.
00 = Normal Operation (Not powered-down).

ADDRESS 0Ah (default:0 )
bit 9 : DAC1 Output Driver Gain control bits
bit 8 : DAC0 Output Driver Gain control bits
1 = 2x Gain.
0 = 1x Gain.
***********************************/

void mcp47_init( )
{
	unsigned char buf[3];

	i2c_finish_session_forcibly(MCP47_BUS_ID);


	buf[0]= MCP47_VREF_MEM_ADDR <<3; // memory address;
	buf[1]= 0x00 ;
//	buf[2]= MCP47_VREF_BANDGAP | (MCP47_VREF_BANDGAP <<2) ;
//	buf[2]= 0x5 ;	
//	buf[2]= MCP47_VREF_VREF_BUF | (MCP47_VREF_VREF_BUF <<2) ;
//	buf[2]= MCP47_VREF_VDD | (MCP47_VREF_VDD <<2) ;

	buf[2]= 0x0A; // MCP47_VREF_VREF_UNBUF | (MCP47_VREF_VREF_UNBUF <<2) ;

	if(0> i2c_write_n_bytes_blocking(MCP47_BUS_ID, MCP47_ADDR, buf, 3))
	{
		printk("set_mcp47 err\n");
	}
/*
		buf[0]= MCP47_GAIN_MEM_ADDR <<3; // memory address;
		buf[1]= (MCP47_GAIN0_2X | MCP47_GAIN1_2X) ;
		buf[2]= 0x00 ;
	
		if(0> i2c_write_n_bytes_blocking(MCP47_BUS_ID, MCP47_ADDR, buf, 3))
		{
			printk("set_mcp47 err1\n");
		}
	*/

	g_data_tray.DAC_out_req[0] = 1;
	g_data_tray.DAC_out_req[1] = 1;
	mcp47_poll();

	g_data_tray.DAC_out_req[0] = 0;
	g_data_tray.DAC_out_req[1] = 0;
	mcp47_poll();

}

void mcp47_onoff(unsigned char ison) 
{
	unsigned char buf[3];
	unsigned char reg;

	reg = (ison & 0x02) ? MCP47_POWERDOWN_NORMAL : MCP47_POWERDOWN_1K ;
	reg <<= 2;
	reg |= (ison & 0x01) ? MCP47_POWERDOWN_NORMAL : MCP47_POWERDOWN_1K ;


	buf[0]= MCP47_POWERDOWN_MEM_ADDR <<3; // memory address;
	buf[1]= 0x00 ;
	buf[2]= reg ;
	
	if (0 > i2c_write_n_bytes_blocking(MCP47_BUS_ID, MCP47_ADDR, buf, 3)) {
		printk("set_mcp47 err1\n");
	}
}

// 设置DAC输出；
void mcp47_poll()
{
	static unsigned short dac_set[2];
	static unsigned char mcp47_err=0;
	static unsigned char _on_flag=0x03;

	unsigned char buf[3];
	unsigned char idx, flag,delay_flag =0;

	flag = (0 == dac_set[0]) ? 0x00 : 0x01;
	flag |= (0 == dac_set[1]) ? 0x00 : 0x02;

	if (_on_flag != flag) {   // 打开mcp47，初始化完后关闭
		_on_flag = flag;
		mcp47_onoff(_on_flag);
		delay_flag = 1;
	}

	for (idx = 1; idx < 2; idx++) {
		if(g_data_tray.DAC_out_req[idx] == dac_set[idx])
			continue;

		printk("DAC_out_req[%d]:	%x -> %x\n",idx, dac_set[idx], g_data_tray.DAC_out_req[idx]);

		buf[0]= idx <<3; // memory address;
		buf[1]= g_data_tray.DAC_out_req[idx] >> 8 ;
		buf[2]=	g_data_tray.DAC_out_req[idx] & 0xff ;
		printk("write mcp47 %x,%x %x %x %x\n", MCP47_BUS_ID, MCP47_ADDR, buf[0], buf[1], buf[2]);
		i2c_finish_session_forcibly(MCP47_BUS_ID);
		if (0 > i2c_write_n_bytes_blocking(MCP47_BUS_ID, MCP47_ADDR, buf, 3)) {
			printk("set_mcp47 err\n");
			mcp47_err++;
		}
		else
			dac_set[idx] = g_data_tray.DAC_out_req[idx] ; // 更新设置值；

	}

	if(1 == delay_flag)
		IODELAY(100); //todo 约25us/100

	g_data_tray.DAC_need_update = 0;
}




