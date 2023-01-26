#include "user.h"

static uint32_t user_u32;

void user_init(void) {
    user_u32 = 0;
}

uint32_t user_get(void) {
    return user_u32;
}

void user_set(uint32_t u32) {
    user_u32 = u32;
}


