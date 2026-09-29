idf_component_register(SRCS "nav_state.c" "perf_stats.c" "biquad.c"
                            "vert_filter.c" "alt_hold.c" "geo.c"
                            "horiz_filter.c" "vel_hold.c" "alt_ref.c"
                            "iterm_gate.c"
                       INCLUDE_DIRS "."
                       REQUIRES freertos esp_hw_support)