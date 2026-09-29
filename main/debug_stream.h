#pragma once

/**
 * Bench debug streamer over USB-Serial-JTAG console.
 *
 * OUTPUT (50 Hz CSV, one line per sample; field map in debug_stream.c):
 *   D,t_us,ax,ay,az,gx,gy,gz,mx,my,mz,q0,q1,q2,q3,roll,pitch,yaw,
 *     fusion_dt,loop_dt,m1,m2,m3,m4,armed,heartbeat,alt_m,lat,lon,sats,
 *     fix,sensor_health,tof_mm,flow_seq,flow_sum_dx,flow_sum_dy,
 *     flow_valid,flow_lost,flow_squal,flow_shutter,flow_flags
 *   plus a 1 Hz P, perf line.
 *
 * INPUT (newline-terminated commands): the console command set - send
 *   HELP for the list. HIL builds also accept:
 *   S,ax,ay,az,gx,gy,gz,mx,my,mz  HIL sensor injection  (HIL_MODE only)
 *   R,roll,pitch,thr,yaw,arm      HIL RC injection, 1000-2000 (HIL_MODE only)
 */
void debug_stream_init(void);
