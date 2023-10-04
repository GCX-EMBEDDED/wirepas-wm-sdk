# Boards compatible with this app 
TARGET_BOARDS := pca20020
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
app_maintenance=16
app_development=0

#define node operating mode (i.e low-energy or low-latency : 0 => low-energy / 1 => low-latency)
default_operating_mode ?= 0
