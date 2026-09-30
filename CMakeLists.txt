# LoTom user-config module. Hardware and DYA Studio features are supplied by
# the modules pinned in config/west.yml.
target_sources_ifdef(CONFIG_LOTOM_INPUT_PROCESSOR_GESTURE app PRIVATE src/input_processor_gesture.c)
