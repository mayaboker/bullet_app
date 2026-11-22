# Betaflight connection config

Goal: Let Bullet App select TCP or serial Betaflight connections through a YAML file, with a fully commented example that also supplies the current TCP defaults.

Implementation steps: Add a bundled `app.yaml` and a separate config loader with validation for independent MSP and MAVLink connection types, endpoints, ports, and serial baud rates. Add `--config` to the CLI, keep existing TCP connection flags as overrides, and select `TcpTransport` or `SerialTransport` when App starts. Document the configuration and update tests.

Interface changes: `AppOptions` gains a config path and connection overrides become optional until configuration is resolved. `--msp-host` and `--msp-port` override YAML only for TCP MSP; `--mavlink-endpoint` overrides the selected MAVLink endpoint. Default CLI startup reads bundled TCP configuration. No MAVLink or MSP message values change.

Verification: The locked App suite passed (65 tests), including TCP and serial parsing/validation, CLI overrides, transport selection, and automatic-run startup with both link types. CLI help, README and control-loop links, and `git diff --check` passed. A wheel build succeeded and contains the bundled `app.yaml`.

Scope/defaults: The YAML config covers Betaflight MSP and MAVLink connections only; mission and View options remain CLI driven. Preserve current TCP behavior by default. No hardware flight claim or live hardware test.

Status: Implemented and verified. Implementation commit: Pending (no commit requested).
