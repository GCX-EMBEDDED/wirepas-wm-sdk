# Boards compatible with this app 
TARGET_BOARDS := silabs_brd4312a nrf52832_mdk_v2 tbsense2 nrf52_template pca10112 bgm220-ek4314a efr32_template silabs_brd4253a pca10090_nrf52840 silabs_brd4180b pca10059 pca10090_low_tx_power silabs_brd4184a ublox_b204 pca10100 mdbt50q_rx pca10056 silabs_brd4181b ruuvitag pan1780 promistel_rpi_hat silabs_brd4254a pca10040 silabs_brd4210a pca10090 wuerth_261101102 
#
# Network default settings configuration
#

# If this section is removed, node has to be configured in
# a different way
default_network_address ?= 0xA1B2C3
default_network_channel ?= 2
# !HIGHLY RECOMMENDED! : To enable security keys please un-comment the lines below and fill with a
#                        randomly generated authentication & encryption keys (exactly 16 bytes)
#default_network_cipher_key ?= 0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??
#default_network_authen_key ?= 0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??

#
# App specific configuration
#

# Define a specific application area_id
app_specific_area_id=0x84BEBD

# App version
app_major=1
app_minor=0
app_maintenance=0
app_development=0
