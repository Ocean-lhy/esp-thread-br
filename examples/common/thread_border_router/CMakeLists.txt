set(requires esp_ot_cli_extension openthread protocol_examples_common vfs esp_wifi esp_eth esp_rcp_update m5stack_core_s3 console)
set(src_dirs "src")
set(include_dirs "include")

if(CONFIG_OPENTHREAD_BR_UI)
list(APPEND src_dirs "gui" "gui/image")
list(APPEND include_dirs "gui")
endif()

if(CONFIG_OPENTHREAD_CLI_OTA)
    list(APPEND requires esp_http_client esp_br_http_ota)
endif()

idf_component_register(SRC_DIRS "${src_dirs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES "${requires}"
)

if(CONFIG_OPENTHREAD_BR_UI)
file(GLOB_RECURSE LV_DEMOS_SOURCES ./*.c)
set_source_files_properties(
    ${LV_DEMOS_SOURCES}
    PROPERTIES COMPILE_OPTIONS
    -DLV_LVGL_H_INCLUDE_SIMPLE)
endif()