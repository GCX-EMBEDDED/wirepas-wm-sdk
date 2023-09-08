# Use Shared Data
SHARED_DATA=yes

APP_PRINTING=yes

# Use App Scheduler and configure it
APP_SCHEDULER=yes
APP_SCHEDULER_TASKS=3

# Add lib to use GPIO interrupts
SRCS +=$(SRCS_PATH)gpio_nrf52.c
INCLUDES +=-I$(SRCS_PATH)

HAL_I2C=yes

HTS221=yes