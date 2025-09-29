///===================================================================
/// File Name: SH1106.h
/// Type     : STM32 C-header
/// Purpose  : SH1106 Low-level Driver
/// Version  : 1.0
///===================================================================
/// Description
///     * Driver for SH1106 OLED Display
///     * I2C communication driver
///===================================================================
/// Revision History
/// Version/Date : V1.0 / 2025-Sep-19 / G.RUIZ
///		* Initial release
///===================================================================



#ifndef SH1106_H__
#define SH1106_H__


//#include <stdint.h>


/// @attention Change according to current MCU clock settings
#define MCU_FREQUENCY_MHZ	( 8U		)
#define MCU_PERIOD_NS		( 125U		)


///	CONTROL BYTE CONSTRUCTION:
///	Co [7] : Continuation Bit
///		1 = no-continuation (only one byte to follow)
///		0 = device expects a stream of bytes
///	D/C# [6] : Data/Command Select Bit
///		1 = the next byte or byte stream will be Data
///		0 = the next byte or byte stream will be Command
///	Usage:
///		0x80: Single Command byte
///		0x00: Command stream
///		0xC0: Single Data byte
///		0x40: Data stream

/// @brief Enum for Control Byte's Continue bit
typedef enum
{
	CONT_BIT_STREAM_BYTES,
	CONT_BIT_SINGLE_BYTE
} ContBitType;

/// @brief Enum for Control Bytes Data/Cmd bit
typedef enum
{
	DCN_BIT_CMD_BYTE,
	DCN_BIT_DATA_BYTE
} DataCmdBitType;

typedef enum
{
	CHARGEPUMP_6V4,
	CHARGEPUMP_7V4,
	CHARGEPUMP_8V0,
	CHARGEPUMP_9V0
} ChargePumpVoltage;

typedef enum
{
	SEG_REMAP_NORMAL,
	SEG_REMAP_REVERSE
} SegmentRemapType;

typedef enum
{
	ENTIRE_DISPLAY_NORMAL,
	ENTIRE_DISPLAY_ON
} EntireDisplayMode;

typedef enum
{
	DISPLAY_LOGIC_NORMAL,
	DISPLAY_LOGIC_INVERSE
} DisplayLogic;

typedef enum
{
	DC_TO_DC_OFF,
	DC_TO_DC_ON
} DCMode;

typedef enum
{
	DISPLAY_OFF,
	DISPLAY_ON
} DisplayEnable;

typedef enum
{
	COM_SCAN_NORMAL,
	COM_SCAN_REVERSE
} COMScanDir;

typedef enum
{
	COM_PIN_SEQUENTIAL,
	COM_PIN_ALTERNATIVE
} COMPinConfig;


struct SH1106_HWSetup
{
	uint8_t disp_start_line;
	SegmentRemapType seg_remap;
	uint8_t mux_ratio;
	COMScanDir com_scan_dir;
	uint8_t disp_offset;
	COMPinConfig pin_cfg;
};

struct SH1106_TimingDriveSetup
{
	uint8_t osc_freq;
	uint8_t div_ratio;
	uint8_t vcom_desel;
	uint8_t precharge;
	uint8_t discharge;
	ChargePumpVoltage cpvoltage;
};

struct SH1106_Instance
{
	uint8_t i2c_channel;
	struct SH1106_HWSetup hwsetup;
	struct SH1106_TimingDriveSetup tdsetup;

	EntireDisplayMode entire_display_mode;
	DisplayLogic logic;
	uint8_t contrast;

	DisplayEnable enable;
};

extern void SH1106_Initialize( struct SH1106_Instance * display );
extern void SH1106_Set_Contrast( struct SH1106_Instance * display );
extern void SH1106_Set_EntireDisplay_Mode( struct SH1106_Instance * display );
extern void SH1106_Set_Display_Logic( struct SH1106_Instance * display );
extern void SH1106_Set_Display( struct SH1106_Instance * display );
extern void SH1106_Set_Address( struct SH1106_Instance * display, uint8_t col_addr, uint8_t page_addr );
extern void SH1106_Write_Page_Data( struct SH1106_Instance * display, uint8_t page, uint8_t col_start, uint16_t size, uint8_t * array );
extern void SH1106_Clear_DisplayRAM( struct SH1106_Instance * display );
extern void SH1106_Sample_Setup( struct SH1106_Instance * display );
extern void SH1106_Write_Frame_Buffer( struct SH1106_Instance * display, uint8_t page_start, uint8_t page_stop,
							uint8_t col_start, uint8_t col_stop, uint16_t page_size, uint8_t * framebuffer );



#endif /*SH1106_H__*/