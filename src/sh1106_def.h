///===================================================================
/// File Name: sh1106_def.h
/// Type     : STM32 C-header
/// Purpose  : SH1106 Definition and Memory Map
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


#ifndef __SH1106_DEF_H__
#define __SH1106_DEF_H__

///===================================================================
///							Sec_1_0 CONSTANTS
///===================================================================

#define SH1106_I2C_ADDR_GND				( 0x3CUL		)		/// I2C address when SA0 is strapped to GND; requires left-shift
#define SH1106_I2C_ADDR_VCC				( 0x3DUL		)		/// I2C address when SA0 is strapped to VDD; requires left-shift

#define SH1106_CTRL_BYTE_CO_Pos			( 7U			)
#define SH1106_CTRL_BYTE_DC_Pos			( 6U			)


///===================================================================
///						Sec_2_0 TYPE DEFINITIONS
///===================================================================


///===================================================================
///					Sec_3_0 STRUCTURES AND ENUMS
///===================================================================


///===================================================================
///						  Sec_4_0 COMMANDS
///===================================================================

#define SH1106_CMD_SET_COL_HI_ADDR				( 0x10UL		)
#define SH1106_CMD_COL_LO_Mask					( 0x0FUL		)
#define SH1106_CMD_COL_HI_Mask					( 0xF0UL		)
#define SH1106_COL_ADDR_MAX						( 0X83UL		)

#define SH1106_CMD_SET_CHARGEPUMP_VOLTAGE		( 0x30UL		)
#define SH1106_CMD_CHARGEPUMP_VOLTAGE_Mask		( 0x03UL		)

#define SH1106_CMD_SET_DISPLAY_START_LINE		( 0x40UL		)
#define SH1106_CMD_DISPLAY_START_LINE_Mask		( 0x3FUL		)

#define SH1106_CMD_SET_CONTRAST					( 0x81UL		)
#define SH1106_CMD_CONTRAST_Mask				( 0xFFUL		)

#define SH1106_CMD_SET_SEGMENT_REMAP			( 0xA0UL		)
#define SH1106_CMD_SEGMENT_REMAP_Mask			( 0x01UL		)

#define SH1106_CMD_SET_ENTIRE_DISPLAY_MODE		( 0xA4UL		)
#define SH1106_CMD_ENTIRE_DISPLAY_MODE_Mask		( 0x01UL		)

#define SH1106_CMD_SET_DISPLAY_LOGIC			( 0xA6UL		)
#define SH1106_CMD_DISPLAY_LOGIC_Mask			( 0x01UL		)

#define SH1106_CMD_SET_MUX_RATIO				( 0xA8UL		)
#define SH1106_CMD_MUX_RATIO_Mask				( 0x3FUL		)

#define SH1106_CMD_SET_DC_CTRL					( 0xADUL		)
#define SH1106_CMD_SET_DC_MODE					( 0x8AUL		)
#define SH1106_CMD_DC_MODE_Mask					( 0x01UL		)

#define SH1106_CMD_SET_DISPLAY_CTRL				( 0xAEUL		)
#define SH1106_CMD_DISPLAY_CTRL_Mask			( 0x01UL		)

#define SH1106_CMD_SET_PAGE_ADDR				( 0xB0UL		)
#define SH1106_CMD_PAGE_ADDR_Mask				( 0x07UL		)
#define SH1106_PAGE_ADDR_MAX					( 8U			)

#define SH1106_CMD_SET_COM_SCAN_DIR				( 0xC0UL		)
#define SH1106_CMD_COM_SCAN_DIR_Pos				( 3U			)
#define SH1106_CMD_COM_SCAN_DIR_Mask			( 0x01UL << SH1106_CMD_COM_SCAN_DIR_Pos )

#define SH1106_CMD_SET_DISP_OFFSET				( 0xD3UL		)
#define SH1106_CMD_DISP_OFFSET_Mask				( 0x3FUL		)

#define SH1106_CMD_SET_CLK_RATIO_FREQ			( 0xD5UL		)
#define SH1106_CMD_CLK_RATIO_Pos				( 0U			)
#define SH1106_CMD_CLK_RATIO_Mask				( 0x0FUL << SH1106_CMD_CLK_RATIO_Pos )
#define SH1106_CMD_CLK_FREQ_Pos					( 4U			)
#define SH1106_CMD_CLK_FREQ_Mask				( 0x0F << SH1106_CMD_CLK_FREQ_Pos )

#define SH1106_CMD_SET_CHARGE_PERIOD			( 0xD9UL		)
#define SH1106_CMD_PRECHARGE_PERIOD_Pos			( 0U			)
#define SH1106_CMD_PRECHARGE_PERIOD_Mask		( 0x0FUL << SH1106_CMD_PRECHARGE_PERIOD_Pos )
#define SH1106_CMD_DISCHARGE_PERIOD_Pos			( 4U			)
#define SH1106_CMD_DISCHARGE_PERIOD_Mask		( 0x0FUL << SH1106_CMD_DISCHARGE_PERIOD_Pos )

#define SH1106_CMD_SET_PAD_HW_CONFIG			( 0xDAUL		)
#define SH1106_CMD_SEQ_ALT_MODE					( 0x02UL		)
#define SH1106_CMD_SEQ_ALT_MODE_Pos				( 4U			)
#define SH1106_CMD_SEQ_ALT_MODE_Mask			( 0x01UL << SH1106_CMD_SEQ_ALT_MODE_Pos )

#define SH1106_CMD_SET_VCOM_DESEL_LVL			( 0xDBUL		)
#define SH1106_CMD_VCOM_DESEL_LVL_Mask			( 0xFFUL		)

#define SH1106_CMD_SET_RMW_START				( 0xE0UL		)
#define SH1106_CMD_SET_RMW_END					( 0xEEUL		)

#define SH1106_CMD_NOP							( 0xE3UL		)



#endif /*__SH1106_DEF_H__*/