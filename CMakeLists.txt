target_include_directories(
    app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(CONFIG_ZMK_PIMORONI_PIM447)

    zephyr_library()

    zephyr_include_directories(include)

    zephyr_library_include_directories(
        ${APPLICATION_SOURCE_DIR}/include
    )

    zephyr_library_sources(
        src/pimoroni_pim447.c
        src/pimoroni_pim447_led.c
    )

endif()

target_sources_ifdef(
    CONFIG_ZMK_PIM447_BEHAVIORS
    app PRIVATE
    src/behaviors/behavior_pim447.c
)
