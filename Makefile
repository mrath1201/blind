PROGRAM ?= temp

EXTRA_COMPONENTS = \
	extras/dhcpserver \
	extras/http-parser \
	extras/rboot-ota \
	$(abspath ../esp-homekit-demo/components/common/button) \
	$(abspath ../esp-homekit-demo/components/esp8266-open-rtos/wifi_config) \
	$(abspath ../esp-homekit-demo/components/esp8266-open-rtos/cJSON) \
	$(abspath ../esp-homekit-demo/components/common/wolfssl) \
	$(abspath ../esp-homekit-demo/components/common/homekit)

SENSOR_PIN ?= 4
HK_PASSWORD ?= "111-11-111"
HK_SETUP_ID ?= "1QJ8"
FLASH_SIZE ?= 32
FW ?= "0.0.1"
EXTRA_CFLAGS += -I../.. -DHOMEKIT_SHORT_APPLE_UUIDS -DSENSOR_PIN=$(SENSOR_PIN) -DHK_PASSWORD="\"$(HK_PASSWORD)\""  -DHK_SETUP_ID="\"$(HK_SETUP_ID)\"" -DFW="\"$(FW)\""

include $(SDK_PATH)/common.mk

monitor:
	$(FILTEROUTPUT) --port $(ESPPORT) --baud 115200 --elf $(PROGRAM_OUT)
