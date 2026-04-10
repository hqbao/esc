// ═══════════════════════════════════════════════════════════════════════════
// FOC mode dispatcher — selects implementation via build-time define
//
// Build with:  make FOC_MODE=encoder | no_feedback | bemf
// Defines:     -DFOC_MODE_ENCODER / -DFOC_MODE_NO_FEEDBACK / -DFOC_MODE_BEMF
// ═══════════════════════════════════════════════════════════════════════════

#include "foc.h"

#if defined(FOC_MODE_ENCODER)
  extern void foc_encoder_setup(void);
#elif defined(FOC_MODE_NO_FEEDBACK)
  extern void foc_no_feedback_setup(void);
#elif defined(FOC_MODE_BEMF)
  extern void foc_bemf_setup(void);
#else
  #error "No FOC mode defined. Set -DFOC_MODE_ENCODER, -DFOC_MODE_NO_FEEDBACK, or -DFOC_MODE_BEMF"
#endif

void foc_setup(void) {
#if defined(FOC_MODE_ENCODER)
    foc_encoder_setup();
#elif defined(FOC_MODE_NO_FEEDBACK)
    foc_no_feedback_setup();
#elif defined(FOC_MODE_BEMF)
    foc_bemf_setup();
#endif
}
