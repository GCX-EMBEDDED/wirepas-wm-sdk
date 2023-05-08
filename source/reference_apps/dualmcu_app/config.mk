# Boards compatible with this app 
TARGET_BOARDS := ublox_b204 promistel_rpi_hat mdbt50q_rx pca10090 pca10059 pca10040 wuerth_261101102 nrf52832_mdk_v2 pca10056 pca10100 tbsense2 silabs_brd4254a silabs_brd4253a bgm220-ek4314a silabs_brd4312a silabs_brd4180b silabs_brd4181b silabs_brd4184a silabs_brd4210a silabs_brd2703a silabs_brd2601b pan1780 

# Define a specific application area_id
app_specific_area_id=0x846B74

# App version
app_major=$(sdk_major)
app_minor=$(sdk_minor)
app_maintenance=$(sdk_maintenance)
app_development=$(sdk_development)

# Uncomment to allow reading scratchpad via dual-MCU API
#allow_scratchpad_read=yes
