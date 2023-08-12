idf_component_register(
    SRCS "rf433.cpp"
    REQUIRES driver
    REQUIRES esp_event
    INCLUDE_DIRS "include"
)
