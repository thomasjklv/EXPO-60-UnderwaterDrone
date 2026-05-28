# Smart Control Allocation BlueOS Extension (Code Generation Version)

This version extends the BlueOS actuator configurator so that every save automatically generates runtime files from the stored JSON configuration.

## What it saves
- `/data/actuator_config.json`

## What it generates automatically
- `/data/generated/actuator_specs.h`
- `/data/generated/actuator_specs.c`
- `/data/generated/runtime_config.json`
- `/data/generated/actuator_runtime_settings.h`
- `/data/generated/manifest.json`

## Main idea
You configure actuators in the BlueOS web UI.
Then the extension saves the JSON and regenerates C files that can be copied into your runtime branch.

## Useful endpoints
- `/api/config`
- `/api/generate`
- `/api/generated/manifest`
- `/api/generated/actuator_specs.h`
- `/api/generated/actuator_specs.c`
- `/api/generated/runtime_config.json`
- `/api/generated/actuator_runtime_settings.h`

## BlueOS URL
`/extensionv2/smartcontrolallocation/`

## Integration into your C project
Expected placement of the generated C files:
- `src/common_Control/actuator_specs.h`
- `src/common_Control/actuator_specs.c`

Then add `src/common_Control/actuator_specs.c` to your `CMakeLists.txt`.

## Persistence
Everything under `/data` survives container restarts if your BlueOS extension uses a persistent volume for `/data`.
