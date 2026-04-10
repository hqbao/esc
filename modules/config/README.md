# Config Module

## Overview

Placeholder for future configuration distribution. Currently a no-op — encoder offset is auto-detected at the RAMP→CL transition every startup, so no flash-stored configuration is needed.

## API

```c
void config_setup(void);  // No-op
```

## Future Use

When motor parameters (pole pairs, resistance, inductance) or calibration data need to be persisted and distributed to other modules at startup, this module will load them from `local_storage` and publish via dedicated PubSub topics.
