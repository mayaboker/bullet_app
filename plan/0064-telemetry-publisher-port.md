# Telemetry publisher port

Goal: Avoid the renderer's control endpoint on port 5557 so the App's blackboard publisher can bind during SITL runs.

Implementation: Move the example App publisher endpoint to port 5558, align the workspace launch configs and MCAP Bagger channel endpoints, and update the affected documentation. Keep topic names and rates unchanged.

Interface change: The default example telemetry publisher endpoint becomes `tcp://127.0.0.1:5558`. Existing custom configs keep their chosen endpoint.

Verification: Parse and compare App and Bagger configs, run App configuration/publisher tests, and check that the renderer reserves port 5557 while 5558 is available.

Status: Implemented. App configuration/publisher tests passed (26), Bagger registration tests passed (18), and both workspace launch configs parsed with matching port 5558. Implementation commit: Pending (no commit requested).
