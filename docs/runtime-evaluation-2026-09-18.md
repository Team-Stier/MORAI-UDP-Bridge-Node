# Runtime Evaluation — 2026-09-18

## Scope and Method

- Live MORAI sensor input, Mac Docker with x86_64 Rosetta translation; 11 visible CPUs.
- RViz, VLP16 nodelets and the existing ROS stack remained running.
- No driving command was published. The bridge alone was restarted after build/tests.
- CPU: process utime+stime delta divided by elapsed monotonic time; 100% means one core.
- RSS: sampled once per second. Measurements are short observations, not a leak soak test.
- Sensor/diagnostic delay: ROS header stamp to Python subscriber callback, with TCP_NODELAY.
  This includes ROS transport, scheduling and observer cost; it does not isolate bridge
  computation or include time spent in the kernel before recvfrom().
- Control timing: tcpdump of outgoing 55-byte UDP packets to 192.168.0.8:9093,
  independent of diagnostic callback timing.
- Pressure: eight bounded CPU workers, 191.93 CPU-seconds in 25 seconds
  (approximately 7.68 continuously busy cores). No GPU or memory-bandwidth stress.
- Temporary observers and pressure workers exited after measurement.

## Results

| Metric | Normal | CPU pressure |
| --- | ---: | ---: |
| Bridge CPU, one-core percent | 8.32% | 12.51% |
| 당시 측정한 high-hz health helper CPU | 3.39% | 5.28% |
| Bridge RSS | 25.04 MiB | 24.66–25.03 MiB |
| Diagnostic callbacks | 202.6/s | 193.5/s |
| Diagnostic callback age p95 / p99 | 2.47 / 3.37 ms | 7.83 / 17.58 ms |
| Diagnostic callback age max | 6.83 ms | 66.25 ms |
| Control UDP interval p99 | 21.73 ms | 26.89 ms |
| Control UDP interval max | 26.02 ms | 61.21 ms |
| Camera output | approximately 20 fps each | approximately 20 fps each |
| IMU output | approximately 40–43 Hz | 40.43 Hz |
| GPS output | approximately 5 Hz | 4.99 Hz |
| VLP16 point cloud | approximately 9 Hz | 9.12 Hz |

Normal CPU/helper results use a 15-second matched-helper run; pressure uses 20 seconds.
Normal outgoing UDP capture uses a separate 20-second no-pressure window.
Other no-pressure runs of 20–25 seconds gave bridge CPU 8.24–8.31%.
These sequential windows had variable live input, not a controlled causal A/B experiment.

- High-hz messages were approximately 735 bytes median, 872 bytes p95, 878 bytes max:
  roughly 0.15 MB/s serialized per subscriber, excluding transport overhead.
- Diagnostics-only CPU and mutex wait/hold times were NOT isolated. The total bridge
  CPU bounds their CPU cost, but does not bound worst-case lock waiting.
- Default subscriber transport initially showed diagnostic p99 around 39 ms.
  TCP_NODELAY observers reduced the measured no-pressure p99 to approximately 3–4 ms.
  Do not interpret the original callback delay as mutex hold time.
- RViz used approximately 127–129% of one core normally, substantially more than the bridge.
  The VLP16 nodelet manager used approximately 1.4% normally and 2.2% under pressure.
- 이 측정 뒤 packet-health helper는 패키지에서 제거되었다. 위 수치는 역사적인
  측정값일 뿐 현재 launch 구성의 CPU 비용이 아니다.
- /diagnostics still exists because Velodyne publishes there; the bridge's rich publisher
  is removed.

## Camera Bound

A roughly 12-second loopback capture observed 65,000-byte camera datagrams:
front approximately 60/s, left 20/s, right 40/s. Maximum per-channel packets within
one millisecond were respectively 3, 1, and 2.

The camera batch is now capped at 16 packets per channel, approximately 1 MiB worst-case
payload storage. Reaching the cap immediately processes the full batch in arrival order;
it does not discard packets or wait to fill the batch. EAGAIN still processes partial batches.
The existing frame assembler survives across batches.

The incomplete-frame assembler is separately bounded to 256 chunks, approximately
16 MiB per camera in the worst case. Silence retains at most that incomplete frame,
not an ever-growing queue. Changing the 200 ms timeout into a wall-clock expiry requires
a new timer policy; it was not changed.

## Limits and Assessment

The bridge is lightweight at the observed traffic rate. The now-removed helper included,
the measured total was approximately 0.12 core normally and 0.18 core under pressure. This is not a hard
real-time guarantee: a 50 Hz control loop had an observed 61 ms transmission gap under load.

The kernel capped SO_RCVBUF at 212,992 bytes despite the requested 4 MiB. Observed
socket drop-counter deltas were zero during the normal and pressure windows, but the
small actual buffer leaves limited scheduling margin for 65 KB camera datagrams.
No kernel settings were changed.

Camera frame drops were not universally zero: one normal 25-second window had front=9,
right=1; a later normal 15-second window had front=5. The 20-second pressure window had
zero frame-drop deltas. No source is blamed from these numbers: zero local socket drops
does not exclude upstream relay/network loss or reordering.

GPS/IMU/status retain their existing drain-to-EAGAIN selection policy. Their application
batches are not capped by this camera-only change; sustained overload without EAGAIN
remains a separate resource risk.

Before competition deployment:
1. Repeat with the actual native Ubuntu hardware and real downstream models.
2. Keep CPU scheduling headroom; test CPU affinity/isolation if control deadlines matter.
3. Make the requested receive-buffer size effective at the OS level, then verify actual values.
4. Avoid unnecessary RViz rendering during competition runs.
5. Include long-duration and memory/GPU contention tests before claiming worst-case bounds.

## Verification

- Docker build passed.
- Isolated ROS contract check passed: mixed valid/malformed/non-GGA GPS batch retains
  two parse errors and one non-target count and publishes a WARN plus the selected sample.
- Forty queued camera frames were all processed across the 16-packet batch boundaries.
- Existing timestamp, sanitize and channel-isolation checks passed.
- Network-launch check passed: selected profile starts without UDP probing or fallback.
