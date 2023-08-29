# Boards compatible with this app 
TARGET_BOARDS := silabs_brd4254a efr32_template pca10059 silabs_brd4180b pca10056 nrf52_template silabs_brd4184a silabs_brd4181b promistel_rpi_hat pca10100 ublox_b204 tbsense2 pca10112 pca10040 bgm220-ek4314a wuerth_261101102 silabs_brd4210a mdbt50q_rx nrf52832_mdk_v2 ruuvitag pan1780 silabs_brd4312a silabs_brd4253a pca20020
#
# Network default settings configuration
#

# If this section is removed, node has to be configured in
# a different way
default_network_address ?= 0x123456
default_network_channel ?= 2

#
# App specific configuration
#

# Define a specific application area_id
app_specific_area_id=0x72754B

# App version
app_major=0
app_minor=0
app_maintenance=5
app_development=0

#define node operating mode (i.e low-energy or low-latency : 0 => low-energy / 1 => low-latency)
default_operating_mode ?= 0