#ifndef ____COMPUTER_FRAME_PARSE_H____
#define ____COMPUTER_FRAME_PARSE_H____

#include "gd32f4xx.h"
#include "config.h"

#define COMM_BUFFER_SIZE            	256

#define FRAME_HEAD						0xAF55FA
#define FrameHead_Index					2
#define FRAME_END						0xFF

#define CMD_SET_OUTPUT_FORMAT			0xAA11
#define CMD_SET_SERIAL_CONFIG			0xAA13
#define CMD_SET_SAVE_CONFIG				0xAA23
#define CMD_SET_READ_CONFIG				0xAA24
#define CMD_SET_SYS_MODE				0xAA26
#define CMD_SET_MECHAN_MIGRA			0xAA18
#define CMD_SET_COURSE_ANGLE			0xAA21
#define CMD_SET_USER_AXIS				0xAA1E
#define CMD_SET_ZERO_OFFSET_TIME		0xAA25
#define CMD_SET_GNSS_BASELINE			0xAA20
#define CMD_SET_ALL						0xAA27

#define FIRST_PROGRAM_BYTE		0xCC

#define PRODUCT_ID			0x1A0C;
#define DEVICE_ID			0x3E01;
 

typedef struct INS_frame_set_t
{
	uint8_t frameType;
	uint8_t baudrate;
	uint16_t freq;
}INS_Frame_Setting_TypeDef;

typedef enum frame_serial_baud_cfg_t{
	cfg_baud_9600 = 0x01,
	cfg_baud_19200 = 0x02,
	cfg_baud_38400 = 0x03,
	cfg_baud_57600 = 0x04,
	cfg_baud_115200 = 0x05,
	cfg_baud_230400 = 0x06,
	cfg_baud_460800 = 0x07,
	cfg_baud_614400 = 0x08,
}Frame_Serial_Baud_Cfg_TypeDef;

typedef enum frame_serial_freq_cfg_t{
	cfg_freq_1Hz = 0x0001,
	cfg_freq_10Hz = 0x000A,
	cfg_freq_20Hz = 0x0014,
	cfg_freq_25Hz = 0x0019,
	cfg_freq_50Hz = 0x0032,
	cfg_freq_100Hz = 0x0064,
	cfg_freq_125Hz = 0x007D,
	cfg_freq_250Hz = 0x00FA,
	cfg_freq_500Hz = 0x01F4,
}Frame_Serial_Freq_Cfg_TypeDef;

typedef enum frame_serial_format_cfg_t{
	cfg_format_RAWIMU = 0x04,
	cfg_format_GPGGA = 0x03,
	cfg_format_GPRMC = 0x02,
	cfg_format_GIPOT = 0x01,
}Frame_Serial_Format_Cfg_TypeDef;

typedef enum serial_index_t{
	index_RS422 = 0x00,
	index_RS232A = 0x01,
	index_RS232B = 0x02,
}Serial_Index_TypeDef;

typedef enum ins_data_type_t
{
 INS_DATA_MODE_0 = 0,
 INS_DATA_MODE_1 = 1,
 INS_DATA_MODE_2 = 2,
 INS_DATA_MODE_3 = 3,
 INS_DATA_MODE_4 = 4,
 INS_DATA_MODE_5 = 5,
 INS_DATA_MODE_6 = 6,
 INS_DATA_MODE_7 = 7,
 INS_DATA_MODE_8 = 8,
 INS_DATA_MODE_9 = 9,
 INS_DATA_MODE_MAX,
}INS_DATA_ENUMTypeDef;

typedef enum ins_boot_mode_t
{
 INS_BOOT_MODE_1 = 1,
 INS_BOOT_MODE_2 = 2,
 INS_BOOT_MODE_MAX,
}INS_BOOT_MODE_ENUMTypeDef;

typedef struct
{
	uint8_t comX;
	uint32_t baudRate;
	uint8_t data_bits ;    
    uint8_t parity    ;
    uint8_t stop_bits ;
    uint8_t mode ;
    uint8_t usr_type    ;
}COMM_CONFIG_TypeDef;

typedef struct setting_t{
	uint8_t firstProgram;				//	首次烧写

	
	uint16_t ProductID;					//	产品ID
	uint16_t DeviceID;					//	设备ID 5
	uint32_t ChipID[3];					//	器件ID 12

	COMM_CONFIG_TypeDef comm_cfg;
	Frame_Serial_Format_Cfg_TypeDef msg_out_type;	//消息输出格式
	INS_Frame_Setting_TypeDef serialFrameSetting[3];	//串口输出设置 12
	uint8_t imuAxis;					//用户设置的坐标轴类型 
	uint8_t gInsSysStatus;				//导航系统工作状态 3
	INS_DATA_ENUMTypeDef datamode;		//导航数据模式 
	INS_BOOT_MODE_ENUMTypeDef workmode;	//导航工作模式 2
	float courseAngleCompensation;		//航向角补偿 4
	short timeCompensation;				//时间补偿 2 
	float gnssBaselineLength;			//GNSS基线长度 4

	float gnssMechanicalMigration_x;	//传感器基点相对测量点的机械偏移x
	float gnssMechanicalMigration_y;	//传感器基点相对测量点的机械偏移y
	float gnssMechanicalMigration_z;	//传感器基点相对测量点的机械偏移z 12
	
	uint32_t ARM1_FW_Ver;				//ARM1软件版本
	uint32_t ARM2_FW_Ver;				//ARM2软件版本
	
	uint32_t FPGA_FW_Ver;				//FPGA软件版本 12
	
}AppSettingTypeDef;

typedef enum AppState_t{
	APP_STATE_INIT = 0,
	APP_STATE_MEASURE = 1,
	APP_STATE_CALC = 2,
	APP_STATE_INTEREACT = 3,
	APP_STATE_MAX,
}AppStateTypeDef;

extern AppSettingTypeDef hSetting;
extern AppSettingTypeDef hDefaultSetting;

uint8_t frame_setting_is_update(void);
void frameParse(uint8_t* pData, uint16_t len);
#if (configUse_COMM == COMM_MODE_RS232)
void comm_handle(void);
EventStatus usart_dispose_recvDataTask(void);
#endif
#if (configUse_COMM == COMM_MODE_RS422)
void rs422_comm1_rx(void);
void rs422_comm1_task(void);
#endif


#endif // ____COMPUTER_FRAME_PARSE_H____
