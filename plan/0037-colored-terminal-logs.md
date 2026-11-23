# Colored terminal logs

Goal: make Bullet App diagnostics easier to scan in a terminal while keeping redirected output readable.

Implementation: configure the entry-point Loguru stderr sink with a timestamp, severity level, and message. Color the timestamp and level using Loguru markup, with severity coloring for the message. Let Loguru detect whether stderr supports color.

Interfaces: no CLI, dependency, or telemetry changes.

Verification: check INFO, WARNING, and ERROR output in a terminal and redirected stderr for ANSI escape sequences.

Scope/defaults: use Loguru's standard level colors and automatic terminal detection. Keep sink configuration in the application entry point.

Status: Implemented. INFO, WARNING, and ERROR were verified with a terminal-like stderr and redirected stderr; ANSI colors appeared only for the terminal when `NO_COLOR` was unset. Implementation commit: Pending (no commit requested).
