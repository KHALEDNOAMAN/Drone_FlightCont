#ifndef FC_CORES_H
#define FC_CORES_H

/* Core assignment for every task in the flight controller.
 *
 * Core 1 is reserved for the two tasks with a hard deadline:
 *   fusion_task  ~416 Hz, paced by the IMU DRDY interrupt (~2.4 ms/tick)
 *   flight_task  PID -> mixer -> motors, 250 Hz on vTaskDelayUntil
 *
 * Everything else - GPS UBX parsing, telemetry, the 50 Hz debug stream,
 * iBUS RC parsing, the slow I2C sensors, the blackbox writer, the baro
 * retry - goes on core 0 where it cannot preempt
 * them or evict their cache lines. None of those have a deadline tighter
 * than a few tens of ms.
 *
 * Second reason, easy to overlook: perf_stats times sections with the
 * Xtensa CCOUNT cycle counter, which is PER CORE. It throws away any
 * sample whose begin and end landed on different cores and counts it in
 * `discards` on the P, line. While tasks float, every measurement is one
 * scheduler decision away from being silently dropped - so pinning is what
 * makes the P1 numbers trustworthy, not just faster.
 *
 * Do not put anything else on core 1 without re-checking loop_max in the
 * P, stats afterwards. */
#define FC_CORE_REALTIME  1
#define FC_CORE_SUPPORT   0

#endif /* FC_CORES_H */
