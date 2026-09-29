#include "telem_line.h"

#include <string.h>

void telem_line_init(telem_line_t *s)
{
    memset(s, 0, sizeof(*s));
}

static void drop(telem_line_t *s, int n)
{
    s->junk_bytes += (uint32_t)n;
    s->junk_head_n = n < TELEM_JUNK_HEAD ? n : TELEM_JUNK_HEAD;
    memcpy(s->junk_head, s->buf, (size_t)s->junk_head_n);
    s->pos = 0;
}

char *telem_line_feed(telem_line_t *s, uint8_t b)
{
    if (s->skip) {
        if (b == '\n' || b == '\r') s->skip = false;
        else                        s->junk_bytes++;
        return NULL;
    }
    if (b == '\n' || b == '\r') {
        const int n = s->pos;
        if (n == 0) return NULL;            /* "\r\n", or a bare newline */
        for (int k = 0; k < n; k++) {
            const uint8_t u = (uint8_t)s->buf[k];
            if (u < 0x20 || u > 0x7E) {     /* not a command, whatever it is */
                drop(s, n);
                return NULL;
            }
        }
        s->buf[n] = 0;
        s->pos = 0;
        s->lines++;
        return s->buf;
    }
    if (s->pos < TELEM_LINE_MAX - 1) {
        s->buf[s->pos++] = (char)b;
        return NULL;
    }
    /* Overlong: no command is this long. Drop it and everything up to the
     * next newline - the old parser restarted mid-line and would have run
     * the tail as a command of its own. */
    drop(s, s->pos);
    s->junk_bytes++;                        /* the byte that overflowed */
    s->skip = true;
    return NULL;
}

void telem_line_silence(telem_line_t *s)
{
    if (s->pos > 0) drop(s, s->pos);
    s->skip = false;             /* silence is a line boundary as well */
}
