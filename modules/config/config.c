// CONFIG — Placeholder for future configuration distribution.
//
// Encoder offset is now auto-detected at RAMP→CL transition every startup,
// so no flash load/publish is needed.

#include "config.h"
#include <pubsub.h>

void config_setup(void) {
    // Nothing to configure — enc_offset auto-detected in foc.c
}
