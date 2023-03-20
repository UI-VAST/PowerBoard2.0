add_library(TinyGPSPlus INTERFACE)

target_sources(TinyGPSPlus INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/TinyGPS++.cpp
)

target_include_directories(TinyGPSPlus INTERFACE
    src
    include
)

target_link_libraries(TinyGPSPlus INTERFACE
        hardware_spi
        hardware_dma
        hardware_rtc
        hardware_timer
        pico_time
        pico_stdlib 
)