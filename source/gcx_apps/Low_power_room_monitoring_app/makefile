# Use Shared Data
SHARED_DATA=yes

APP_PRINTING=no

# Use App Scheduler and configure it
APP_SCHEDULER=yes
APP_SCHEDULER_TASKS=3

# Add lib to use GPIO interrupts
SRCS +=$(SRCS_PATH)gpio_nrf52.c

SRCS +=$(SRCS_PATH)battery_measurement.c

INCLUDES +=-I$(SRCS_PATH)

HAL_I2C=yes
# And HAL voltage measurement examples
HAL_VOLTAGE=yes

HTS221=yes
SX1509=yes
CCS811=yes