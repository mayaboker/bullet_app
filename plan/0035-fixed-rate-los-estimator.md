# Fixed-rate LOS sensor estimator

Goal: feed target-hit LOS guidance with smoothed, nominal 50 Hz attitude and altitude estimates while retaining each real camera observation until it can be aligned.

Implementation: ingest each new MAVLink measurement and each validated camera box; smooth the SITL boot-time offset so the mapped attitude clock follows its measured drift. Predict attitude at control ticks for at most 60 ms, resample altitude with bounded velocity, and queue boxes in capture-time order. Deliver each box once when attitude covers its capture time. Expire boxes after 0.2 s and report estimator age, skipped ticks, queue depth, and expirations. Preserve source freshness and current stale-telemetry protection.

Interfaces: App-owned estimator and target-ingress batch handling. LOS public inputs and Bullet View wire format remain unchanged.

Verification: deterministic 47 Hz MAVLink/30 Hz camera/50 Hz control tests with jitter, stale and reordered inputs, and a live target-stack run that remains in tracking through the prior failure window or reaches commit/contact.

Scope/defaults: target-hit only; new boxes are never repeated as new detections. Missed control ticks are counted, not replayed as commands. No commit requested.

Status: Implemented; 25 App tests passed and the live target mission reached commit and a confirmed hit with zero alignment replacements. Implementation commit: `9cfa2de`.
