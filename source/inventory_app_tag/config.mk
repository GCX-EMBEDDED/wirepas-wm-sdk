# Boards compatible with this app 
TARGET_BOARDS := pca10040 pca10056 pca10059 pca10100 



#
# Network default settings configuration
#

# If this section is removed, node has to be configured in
# a different way
default_network_address ?= 0x1234FF
default_network_channel ?= 10
# !HIGHLY RECOMMENDED! : To enable security keys please un-comment the lines below and fill with a
#                        randomly generated authentication & encryption keys (exactly 16 bytes)
#default_network_cipher_key ?= 0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??
#default_network_authen_key ?= 0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??,0x??

#
# App specific configuration
#
# Define a specific application area_id
app_specific_area_id=0x84BEAA


# the rate at which advertiser packets are sent
default_advertiser_rate_s = 30

# App version
app_major=5
app_minor=0
app_maintenance=0
app_development=0

