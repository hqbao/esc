#ifndef FOC_H
#define FOC_H

// Default: if no FOC_MODE_* was passed via -D, fall back to encoder.
// This must live in the shared header so every translation unit sees it.
#if !defined(FOC_MODE_ENCODER) && !defined(FOC_MODE_NO_FEEDBACK) && !defined(FOC_MODE_BEMF)
  #define FOC_MODE_ENCODER
#endif

void foc_setup(void);

#endif // FOC_H
