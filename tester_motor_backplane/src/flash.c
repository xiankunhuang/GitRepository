#include "config.h"
#include "backplane.h"


// flash layout，总共256K，共0x15个sectors
// 注意sector0x0~0xF是4KB，0x10~0x15是32KB
// 注意sector0x11的高16K空出来不用，升级firmware的时候会被erase
// firmware的尺寸限制为96KB
//  content		start		size	memo
//  boot_loader	0x00000000	16K		sector0x0~0x3
//  firmware	0x00004000	96K		sector0x4~0x11的低16K
//  empty(空)	0x0001C000	16K		sector0x11的高16K空出来不用，升级firmware的时候会被erase
//  upgrade		0x00020000	96K		sector0x12~0x14
//  eeprom		0x00038000	32K		sector0x15，前256个字节是背板eeprom，能够被用户配置，以4字节0xFACEC0DE开头表示有效；后面的保留
// system_ram	0x10000000	32K		运行内存，编译器用
// sram			0x2007C000	32K		可以从485读写的内存，功能号9的升级方法中，顶部16个字节是control block
// 如何升级boot_loader:	可用0xFE的不应答广播方式批量升级
//  1. 向sram写入最大16K内容
//  2. ioctl触发从sram写入到boot_loader区域(ioctl命令0x300，argument为版本号)，前2个字节是这一笔的长度，接着4个字节是总长度，后续128字节的data为64个每256字节的crc16
//	3. 背板校验数据后，更新backplane_eeprom和boot_loader，不重启
// 如何升级program: 可用0xFE的不应答广播方式批量升级
//  1. 向sram写入第一个32K内容
//  2. ioctl触发从sram写入到upgrade区域(ioctl命令0x300，argument为版本号)，前2个字节是这一笔的长度，接着4个字节是总长度，后续128字节的data为64个每512字节的crc16
//  3. 如果需要就写入后续的字节，需要清零
//  3. 背板更新backplane_eeprom和
#define BOOT_LOADER_START_ADDR			0x00000000
#define FIRMWARE_START_ADDR				0x00004000   // loader up to 16KB
#define UPGRADE_START_ADDR				0x00020000	// 3个32k的sector
#define BACKPLANE_INFO_ADDR				0x00038000	// sector size = 32KB
#define BOOT_LOADER_START_SECTOR_NUM	0x0
#define BOOT_LOADER_END_SECTOR_NUM		0x3
#define FIRMWARE_START_SECTOR_NUM		0x4
#define FIRMWARE_END_SECTOR_NUM			0x11
#define UPGRADE_START_SECTOR_NUM		0x12
#define UPGRADE_END_SECTOR_NUM			0x14
#define BACKPLANE_INFO_ADDR_SECTOR_NUM	0x15


void flash_print_backplane_eeprom()
{
	unsigned char sn[sizeof(g_data_backplane_eeprom.sn) + 1];
	unsigned char oem_info[sizeof(g_data_backplane_eeprom.oem_info) + 1];
	sn[sizeof(sn) - 1] = 0;
	oem_info[sizeof(oem_info) - 1] = 0;
	memcpy(sn, g_data_backplane_eeprom.sn, sizeof(g_data_backplane_eeprom.sn));
	memcpy(oem_info, g_data_backplane_eeprom.oem_info, sizeof(g_data_backplane_eeprom.oem_info));
	printk("BP addr=%x hw_rev=%d sn=%s oem_info=%s\n", g_data_backplane_eeprom.addr, g_data_backplane_eeprom.hw_rev, sn, oem_info);
	printk("boot_loader rev=%d len=%d firmware_rev=%d upgrade=%d/%d %d/%d %d/%d\n",
		g_data_backplane_eeprom.boot_loader_rev, g_data_backplane_eeprom.boot_loader_sz,
		g_data_backplane_eeprom.firmware_rev,
		g_data_backplane_eeprom.upgrade_sector_rev[0], g_data_backplane_eeprom.upgrade_sector_sz[0],
		g_data_backplane_eeprom.upgrade_sector_rev[1], g_data_backplane_eeprom.upgrade_sector_sz[1],
		g_data_backplane_eeprom.upgrade_sector_rev[2], g_data_backplane_eeprom.upgrade_sector_sz[2]);
}

void flash_write_boot_loader_from_sram_if_necessary(unsigned short rev, unsigned int length, unsigned int total)	// buf必须4字节对齐，最大16K
{
	int nr_sectors = (length <= 4096) ? 1
			: (length <= 8192) ? 2
			: (length <= 12288) ? 3
			: 4;
	int sector_start = BOOT_LOADER_START_SECTOR_NUM;
	int sector_end = BOOT_LOADER_START_SECTOR_NUM + nr_sectors - 1;
	if (length != total) {
		printk("why? length=%d total=%d rev=%d\n", length, total, rev);
	} else if (rev <= g_data_backplane_eeprom.boot_loader_rev) {
		printk("no need to write boot_loader %d >= %d\n", g_data_backplane_eeprom.boot_loader_rev, rev);
	} else {
		IRQDisable();
		sectorPrepare(sector_start, sector_end);
		sectorErase(sector_start, sector_end);
		sectorPrepare(sector_start, sector_end);	// 因为每次写1个完整的sector，只需要prepare1次
		for (int i = 0; i < nr_sectors; i++) {
			int rc = ramCopy(BOOT_LOADER_START_ADDR + (i << 12), SRAM_ADDR + (i << 12), 4096);
			if (rc != 0) {
				printk("flash_upgrade_boot_loader sector#%d failed rc=%d\n", sector_start + i, rc);
			} else {
				printk("flash_upgrade_boot_loader sector#%d ok\n", sector_start + i);
			}
		}
		IRQEnable();

		// 失败也挂了，认为不可能失败
		g_data_backplane_eeprom.boot_loader_rev = rev;
		g_data_backplane_eeprom.boot_loader_sz = length;
		flash_write_backplane_eeprom();
	}
}

// 返回upgrade区里合法的字节数，错误返回<0
int flash_write_upgrade_sector_from_sram(unsigned char upgrade_sector_idx, unsigned short rev, unsigned int length, unsigned int total)
{
	int sector_num = UPGRADE_START_SECTOR_NUM + upgrade_sector_idx;
	unsigned int start_addr = UPGRADE_START_ADDR + (upgrade_sector_idx << 15);
	int nr_times_to_copy = (length + 4095) >> 12;
	if (upgrade_sector_idx >= 3) {
		printk("bad upgrade_sector_idx=%d", upgrade_sector_idx);
		return -1;
	} else {
		IRQDisable();
		sectorPrepare(sector_num, sector_num);
		sectorErase(sector_num, sector_num);
		for (int i = 0; i < nr_times_to_copy; i++) {
			sectorPrepare(sector_num, sector_num);		// 需要每次prepare
			int rc = ramCopy(start_addr + (i << 12), SRAM_ADDR + (i << 12), 4096);
			if (rc != 0) {
				printk("flash_write_upgrade_sector sector#%d failed rc=%d\n", upgrade_sector_idx, rc);
				return -1;
			} else {
				printk("flash_write_upgrade_sector sector#%d ok\n", upgrade_sector_idx);
			}
		}
		IRQEnable();

		g_data_backplane_eeprom.upgrade_sector_rev[upgrade_sector_idx] = rev;
		g_data_backplane_eeprom.upgrade_sector_sz[upgrade_sector_idx] = length;
		g_data_backplane_eeprom.upgrade_total_len_expected = total;
		flash_write_backplane_eeprom();
		if ((g_data_backplane_eeprom.upgrade_sector_rev[0] == rev) && (g_data_backplane_eeprom.upgrade_sector_rev[1] == rev) && (g_data_backplane_eeprom.upgrade_sector_rev[2] == rev)) {
			return g_data_backplane_eeprom.upgrade_sector_sz[0] + g_data_backplane_eeprom.upgrade_sector_sz[1] + g_data_backplane_eeprom.upgrade_sector_sz[2];
		} else if ((g_data_backplane_eeprom.upgrade_sector_rev[0] == rev) && (g_data_backplane_eeprom.upgrade_sector_rev[1] == rev)) {
			return g_data_backplane_eeprom.upgrade_sector_sz[0] + g_data_backplane_eeprom.upgrade_sector_sz[1];
		} else if (g_data_backplane_eeprom.upgrade_sector_rev[0] == rev) {
			return g_data_backplane_eeprom.upgrade_sector_sz[0];
		} else {
			return 0;
		}
	}
}

// 如果需要就从upgrade区拷贝最大96K到firmware区
// 注意sector#0x4~sector#0xF是4K大小，共48K；sector0x10为32K；sector0x11为32K，但只用前16K
void flash_write_firmware_in_boot_loader_if_necessary()
{
	const static unsigned char sector_num_of_each_4k[] = { 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
	                                                       0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x10, 0x10, 0x10,
	                                                       0x10, 0x10, 0x10, 0x10, 0x11, 0x11, 0x11, 0x11 };
	backplane_eeprom_t * e = &g_data_backplane_eeprom;
	unsigned short new_rev = e->upgrade_sector_rev[0];
	int nr_upgrade_sectors_has_value = 0;

	if ((e->upgrade_total_len_expected < 0x500) || (e->upgrade_total_len_expected > 32768 * 3)) {
		printk("bad upgrade 0\n");
		return;
	}
	if ((e->upgrade_sector_rev[1] == new_rev) && (e->upgrade_sector_rev[2] == new_rev)) {
		if (e->upgrade_total_len_expected == (unsigned int)e->upgrade_sector_sz[0] + (unsigned int)e->upgrade_sector_sz[1] + (unsigned int)e->upgrade_sector_sz[2]) {
			nr_upgrade_sectors_has_value = 3;
		} else {
			printk("bad upgrade 1\n");
			return;
		}
	} else if (e->upgrade_sector_rev[1] == new_rev) {
		if (e->upgrade_total_len_expected == (unsigned int)e->upgrade_sector_sz[0] + (unsigned int)e->upgrade_sector_sz[1]) {
			nr_upgrade_sectors_has_value = 2;
		} else {
			printk("bad upgrade 2\n");
			return;
		}
	} else {
		if (e->upgrade_total_len_expected == e->upgrade_sector_sz[0]) {
			nr_upgrade_sectors_has_value = 1;
		} else {
			printk("bad upgrade 3\n");
			return;
		}
	}
	IRQDisable();

	sectorPrepare(FIRMWARE_START_SECTOR_NUM, FIRMWARE_END_SECTOR_NUM);
	sectorErase(FIRMWARE_START_SECTOR_NUM, FIRMWARE_END_SECTOR_NUM);

	for (int block_idx_32K = 0; block_idx_32K < nr_upgrade_sectors_has_value; block_idx_32K++) {
		memcpy(SRAM_ADDR, UPGRADE_START_ADDR + (block_idx_32K << 15), SRAM_SZ);	// 每32K用SRAM中转一下
		for (int i = 0; i < 8; i++) {	// 每次拷贝4KB
			unsigned int offset_4k = (block_idx_32K << 15) + (i << 12);
			sectorPrepare(sector_num_of_each_4k[offset_4k >> 12], sector_num_of_each_4k[offset_4k >> 12]);
			int rc = ramCopy(FIRMWARE_START_ADDR + offset_4k, SRAM_ADDR + (i << 12), 4096);
			if (rc != 0) {
				printk("write_firmware 4K#%d failed\n", offset_4k >> 12);
			} else {
				printk("write_firmware 4K#%d ok\n", offset_4k >> 12);
			}
		}
	}
	g_data_backplane_eeprom.upgrade_total_len_expected = 0x00; 	// 当作升级标记用；0 表示已经升级；
	flash_write_backplane_eeprom();
	IRQEnable();
}

void flash_write_backplane_eeprom()
{
	unsigned char page_cache_buf[256 + 4];
	unsigned char *page = page_cache_buf + (4 - ((unsigned int)page_cache_buf & 0x3));		// 4字节对齐的256字节，写flash的长度只能是256/512/1024/4096
	memset(page, 0, 256);
	memcpy(page, &g_data_backplane_eeprom, sizeof(g_data_backplane_eeprom));
	unsigned int rc;
	IRQDisable(); 
	sectorPrepare(BACKPLANE_INFO_ADDR_SECTOR_NUM, BACKPLANE_INFO_ADDR_SECTOR_NUM);
	sectorErase(BACKPLANE_INFO_ADDR_SECTOR_NUM, BACKPLANE_INFO_ADDR_SECTOR_NUM);
//	blankChk(BACKPLANE_INFO_ADDR_SECTOR_NUM, BACKPLANE_INFO_ADDR_SECTOR_NUM);
	sectorPrepare(BACKPLANE_INFO_ADDR_SECTOR_NUM, BACKPLANE_INFO_ADDR_SECTOR_NUM);
	ramCopy(BACKPLANE_INFO_ADDR, (unsigned int)page, 256);
	rc = dataCompare(BACKPLANE_INFO_ADDR, (unsigned int)page, 256);
	IRQEnable();
	if (rc != 0) {
		printk("flash_write_backplane_eeprom failed\n");
	} else {
		printk("flash_write_backplane_eeprom ok\n");
	}
}

void flash_read_backplane_eeprom()
{
	memcpy(&g_data_backplane_eeprom, (unsigned char *)BACKPLANE_INFO_ADDR, sizeof(g_data_backplane_eeprom));	// load backplane_info from flash
	if (g_data_backplane_eeprom.magic != SOLIWARE_MAGIC) {
		printk("bad backplane eeprom, update with default value\n");
		memset(&g_data_backplane_eeprom, 0, sizeof(g_data_backplane_eeprom));
		g_data_backplane_eeprom.magic = SOLIWARE_MAGIC;
		g_data_backplane_eeprom.addr = 0xFF;
		g_data_backplane_eeprom.hw_rev = 0xFF;
		memset(g_data_backplane_eeprom.sn, ' ', sizeof(g_data_backplane_eeprom.sn));
		memcpy(g_data_backplane_eeprom.sn, "NO BACKPLANE SN", sizeof("NO BACKPLANE SN") - 1);
		memset(g_data_backplane_eeprom.oem_info, ' ', sizeof(g_data_backplane_eeprom.oem_info));
		memcpy(g_data_backplane_eeprom.oem_info, "SOLIWARE WUHAN", sizeof("SOLIWARE WUHAN") - 1);

		flash_write_backplane_eeprom();
		return;
	}
}

static __noreturn void boot_jump(unsigned int firmwareStartAddress)
{
	VTOR = firmwareStartAddress;
	__asm("  LDR SP, [R0]   \n" " LDR PC, [R0, #4]  ");
	// LDR SP, [R0]			;Load new stack pointer address
	// LDR PC, [R0, #4]		;Load new program counter address
}

__noreturn void flash_go(unsigned char is_boot_loader)
{
	boot_jump(is_boot_loader ? BOOT_LOADER_START_ADDR : FIRMWARE_START_ADDR);
}

//
// 下面这几个函数是针对chamber_controller的，为了在升级的时候置个标记，避免升级过程中主控板异常掉电
// SRAM的最后是连续三个SOLIWARE_MAGIC，表示初始化的时候不给主控板掉电
//
int flash_is_keeping_main_controller_on()
{
	volatile unsigned int * m1 = (unsigned int *)(SRAM_ADDR + SRAM_SZ - 12);
	volatile unsigned int * m2 = (unsigned int *)(SRAM_ADDR + SRAM_SZ - 8);
	volatile unsigned int * m3 = (unsigned int *)(SRAM_ADDR + SRAM_SZ - 4);
	return (*m1 == SOLIWARE_MAGIC) && (*m2 == SOLIWARE_MAGIC) && (*m3 == SOLIWARE_MAGIC);
}

void flash_clear_keeping_main_controller_on_flag()
{
	volatile unsigned int * m1 = (unsigned int *)(SRAM_ADDR + SRAM_SZ - 12);
	volatile unsigned int * m2 = (unsigned int *)(SRAM_ADDR + SRAM_SZ - 8);
	volatile unsigned int * m3 = (unsigned int *)(SRAM_ADDR + SRAM_SZ - 4);
	*m1 = 0;
	*m2 = 0;
	*m3 = 0;
}

__noreturn void flash_go_while_keeping_main_controller_on(unsigned char is_boot_loader)
{
	volatile unsigned int * m1 = (unsigned int *)(SRAM_ADDR + SRAM_SZ - 12);
	volatile unsigned int * m2 = (unsigned int *)(SRAM_ADDR + SRAM_SZ - 8);
	volatile unsigned int * m3 = (unsigned int *)(SRAM_ADDR + SRAM_SZ - 4);
	*m1 = SOLIWARE_MAGIC;
	*m2 = SOLIWARE_MAGIC;
	*m3 = SOLIWARE_MAGIC;
	boot_jump(is_boot_loader ? BOOT_LOADER_START_ADDR : FIRMWARE_START_ADDR);
}

