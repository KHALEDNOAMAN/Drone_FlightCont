#include "nav_state.h"
#include <string.h>

nav_state_t global_nav_state;
SemaphoreHandle_t state_mutex;

void nav_state_init(void) {
    state_mutex = xSemaphoreCreateMutex();
    
    // Clear everything to 0
    memset(&global_nav_state, 0, sizeof(nav_state_t));

    global_nav_state.is_armed = 0;
}