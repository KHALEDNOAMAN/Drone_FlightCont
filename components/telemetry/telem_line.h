#ifndef TELEM_LINE_H
#define TELEM_LINE_H

/* Line assembler for the C6 -> S3 command link (telemetry_rx_task).
 *
 * Pure C, no ESP headers, so tools/telem_line_test.py compiles THIS file.
 *
 * LINE NOISE . The link's RX input collects bytes nobody sent:
 * measured at ~70 B/s of 0xFF, in bursts every ~2.7 s, with the aircraft
 * disarmed and the C6 idle - short negative glitches on the wire, each
 * decoding as 0xFF. None of it ends in a newline, so it used to sit in the
 * line buffer and get glued onto the FRONT of the next real command:
 * "unknown command: \xFF\xFF...MOT,1,1050,3000" on the dashboard, and the
 * retyped command working because the first one's newline had flushed the
 * junk. Two rules, both safe because the C6 sends a command as ONE write of
 * printable ASCII (cmd_post in the C6 firmware refuses anything else):
 *   - silence ends a line: a partial line when the link goes quiet is noise
 *     (telem_line_silence, called after a read that timed out empty);
 *   - a line with any byte outside 0x20..0x7E is noise, never a command.
 * Dropped bytes are counted and the first few kept, so the SOURCE of the
 * glitches stays measurable instead of being silently absorbed. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TELEM_LINE_MAX 160       /* longest command + NUL; C6 caps at 158 */
#define TELEM_JUNK_HEAD 6

typedef struct {
    char     buf[TELEM_LINE_MAX];
    int      pos;
    bool     skip;                        /* after an overlong line: drop
                                             to the next newline, so its
                                             tail is not run as a command */
    uint32_t lines;                       /* printable lines handed out */
    uint32_t junk_bytes;                  /* bytes dropped as noise */
    uint8_t  junk_head[TELEM_JUNK_HEAD];  /* first bytes of the latest drop */
    int      junk_head_n;
} telem_line_t;

void telem_line_init(telem_line_t *s);

/* Feed one received byte. Returns the completed command line (NUL-
 * terminated, in s->buf, valid until the next feed) or NULL. */
char *telem_line_feed(telem_line_t *s, uint8_t b);

/* The link was silent for a whole read timeout: drop any partial line. */
void telem_line_silence(telem_line_t *s);

#endif /* TELEM_LINE_H */
