set(MASTER_MODE "${MODE}")
if(NOT MASTER_MODE)
    set(MASTER_MODE "RX")
endif()
string(TOUPPER "${MASTER_MODE}" MASTER_MODE)

if(MASTER_MODE STREQUAL "TX")
    set(MASTER_MODE_DEFINE "MODE_TX")
elseif(MASTER_MODE STREQUAL "RX")
    set(MASTER_MODE_DEFINE "MODE_RX")
else()
    message(FATAL_ERROR "Invalid MODE='${MASTER_MODE}'. Use TX or RX, for example: idf.py -DMODE=TX build")
endif()

idf_component_register(SRCS "main_master.c"
                    PRIV_REQUIRES nvs_flash esp_event esp_netif esp_wifi esp_driver_gpio esp_driver_i2s
                    INCLUDE_DIRS ".")

target_compile_definitions(${COMPONENT_LIB} PRIVATE
                    MODE_TX=1
                    MODE_RX=2
                    MODE=${MASTER_MODE_DEFINE}
                    MODE_STR=\"${MASTER_MODE}\")
