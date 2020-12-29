# 一般性说明
## 文本文件格式要求
1. 换行符是DOS风格
2. 含中文字符的文本文件编码成UTF-8 without BOM
3. 不含中文字符的默认为ANSI，不用改，这个格式跟UTF-8 without BOM是兼容的
4. 文本的说明文件用markdown语法格式化，后缀为.md

## SourceInsight
1. 直接把SI4的工程建在项目的根目录下，会被忽略
2. 如果是SI3，需要把工程放在SourceInsight目录下，也会被忽略，但SI3对UTF-8支持得不好，不建议使用

## 如何使用build.sh
1. 把你自己的Default.jflash配置成需要的样子
2. 修改build.sh配置IAR_BUILD和JFLASH_ARM的路径，注意路径的写法是mingw的写法
3. 右键点这个目录，“Git Bash Here”（你机器上应该已经装过了Git For Windows）
4. ./build.sh COMMAND IAR_BUILD_CONFIG versionCode
	* COMMAND有
		* clean：只清除
		* build：只编译
		* flash：只烧写
		* buildflash：先编译再烧写
	* IAR_BUILD_CONFIG有
		* boot_loader：目前有
		* Debug：目前有
		* Release：目前有
	* versionCode是编译的版本号，最终输出文件为带产品型号、版本号和签名的二进制固件firmware-MODEL_NAME-vVERSION_CODE.bin，可用于从主控板升级背板

# 项目说明
## CoC_tester
本项目是CoC 配套的测试设备对应的FW;

# 设备工作流程：
1. 开机自检，步进电机找顶点和底点-> R_STATE_IDLE_INSIDE
2. 由APP指令，Y轴电机将夹具台由小窗送出测试机箱 -> R_STATE_IDLE_OUTSIDE
3. 用户安装夹具，FW根据插入监测状态（eeprom，监测pin，heater电阻），插好后APP 启动插入扫描，扫描完成后自动转 R_STATE_IDLE_OUTSIDE -> R_STATE_PLUGIN_SCAN
4. 用户启动测试，Y轴电机将夹具带回测试机箱内部 -> R_STATE_IDLE_INSIDE + DUT_idx
5. 启动温控，等待到目标温度
6. R_STATE_IDLE_INSIDE + DUT_idx, 选中通道DUT_idx 以 I_set_uA 驱动，探头移动到 DUT_idx 并准直，然后关DUT电；-> R_STATE_IDLE_INSIDE
7. 温度达标，开启LIV 测试流程，上传数据；-> R_STATE_LIV
8. FW 的 LIV 测试完成后，自动以 I_set_uA 驱动DUT， APP可以启动光谱测试；-> R_STATE_CC
9. 重复步骤 6/7/8 ；
10. 所有DUT测完之后，断温控断DUT电，Y轴电机将夹具台由小窗送出测试机箱 -> R_STATE_IDLE_OUTSIDE


# tray_eeprom_cmd
1. 背板需要配合主控板做一些特别的事情，现将之前没用的tray_sign_16（在eeprom中0x30~0x3F）重新定义如下：
2. 第一个字节是op，后面最多15字节是参数
	* op=0，什么事情也不做
    * op=1，WO_ONLY，用于给主控板识别这个tray当前属于哪个工单，注意WO真实的名字是前若干字节，后面的有可能是递增的号码（为了保持每个TRAY上写入的WO不一样，以便给DUT命名）
    * op=2，WO_NO_DDM，其后跟随的以\0结尾的最多14字节为WO，没有DDM，不访问I2C但会正常上报数据。DUT会自动命名为WO01~WO25
    * op=3，WO_WRITE_8472，其后跟随的以\0结尾的最多14字节为WO，I2C访问跟标准8472一样。DUT会自动命名为WO01~WO25
    * op=4，

# Feature
## 突出特点
1. 主体框架基本在backplane.h确定，主要参数在config.h中定义，主任务在main.c；
2. 功能模块和底层驱动层次分明，耦合性较小，重用性好。
	* 背板的主要功能模块有：
		* 电源模块（power_module.c）；
		* 驱动模块(drive_module.c)；
		* adc（adc.c）；
		* 通信（protocol.c/rs485.c）和调试（console.c）；
		* tray的检测（tray.c）；
		* 固件升级和背板信息访问（flash.c）；
	* 中间层模块有INA219的访问；
	* 底层端口有：
		* I2C访问（i2c.c）；
		* GPIO访问;
		* UART访问（rs485.c和console.c）
		* flash访问（IAP.c）
3. 模块的访问逻辑由前置状态决定，而前置条件为主控板下发的工作状态及访问条件决定：
	* tray的在线状态和外接电源决定电源模块的启动；
	* 电源模块的状态决定通道开启；
	* 通道开启决定ADC的监控和DUT的SN访问，而通道（DUT）状态反过来影响通道开关；
	* 预置工作条件和通道状态决定驱动；
	* 模块访问主要按照状态机编写，条件和状态影响下一步的访问；
4. 采用数据结构管理模块数据，清晰简洁
	* 采用定时器和g_tick_100us用来做计时处理；
	* 采用g_data_power_modules记录电源模块的状态和中间变量；
	* 采用g_data_driver_modules记录DUT、通道的状态和中间变量；
	* 采用g_data_adc记录通道、温度等的状态和中间变量；
	* 采用g_data_tray记录电源模块、DUT、通道和tray的状态和中间变量；
	* g_data_runtime保存实时值，其中包含下发的工作配置条件；
5. 背板程序完成的功能有：
	* 电源模块:
		* 电源模块开关控制（涉及到tray板拔插）；
		* 输入输出电压电流监控和温度监控，电压精度为mV，输入精度为1mA，输出精度为0.1mA；
		* 电压设定和周期调整，保证电源稳定；
		* 外部掉电保护；
	* 驱动模块（及ADC）：
		* 老化的逐步上电；
		* 老化的通道电流电压的监控；
		* DUT的EEPROM（DDM）的访问；
	* TRAY：
		* 夹具插入和拔出检测；
		* 夹具类型确定；
		* K值；
		* 逐步上电；
		* SN访问和芯片类型检测；
	* 通信：
		* 数据上传；
		* 下发数据状态传递；
		* 固件升级；
		* 同步；
	* ADC：
		* 通道电流电压检测；
		* 板上3.3V电压检测；
		* 温度采集；
		* 地址译码；
	* 调试访问
6. 特别功能有：
	* 保护功能：
		* 断电保护；
		* 扫描和老化阶段的逐个上电；
		* 过流和断路（保险丝）检测；
		* SN合法检测；
	* 访问模块多样化：
		* 夹具类型自动检测；
		* ；
	* 调试访问
		* 状态访问，可用来检测
		* 统计值访问
7. 亮点：稳定检测，防止错误访问
	* tray插入稳定检测；
	* 电源开启稳定检测；
	* DUT上电稳定后访问；
	* 另外，还有动态调压

## 待改进的地方
1. 空白SN的DUT（能成功访问）与坏的DUT区分开；
2. 设定可调的最小电压和最大电压范围，防止影响DUT的正常工作；

## TODO LIST
1. 检测时能开启单独通道调试；
2. 调试能将背板信息写入flash；

# 版本记录


