SRC_DIR := src
INC_DIR := include
BIN_DIR := bin
FONT_DIR := fonts

COMPILER = arm-none-eabi-gcc
CFLAGS = -c -mcpu=cortex-m3 -mthumb -std=gnu11 -I$(INC_DIR) -I$(FONT_DIR)
LDFLAGS = -nostdlib -T "$(SRC_DIR)/stm32_linker.ld" -Wl,-Map=$(BIN_DIR)/bme680.map

this: $(BIN_DIR)/bme680.elf

$(BIN_DIR)/%.o : $(SRC_DIR)/%.c
	$(COMPILER) $(CFLAGS) $^ -o $@

$(BIN_DIR)/bme680.elf : $(BIN_DIR)/stm32f103rb_startup.o $(BIN_DIR)/stm32f1_i2c.o $(BIN_DIR)/bme680_i2c.o $(BIN_DIR)/stm32f1_waitdly.o $(BIN_DIR)/BME680Ctrl.o $(BIN_DIR)/ggg_sh1106_i2c.o $(BIN_DIR)/SH1106.o $(BIN_DIR)/main.o 
	$(COMPILER) $(LDFLAGS) $(BIN_DIR)/*.o -o $@

load:
	openocd -f board/st_nucleo_f103rb.cfg

clear:
	del $(BIN_DIR) *.o *.elf *.map

