/*
 * The interactor, driven with no transport at all.
 *
 * That is the point of the test and not an economy: if this file needs a socket to reach the
 * physics, the contract has been broken and the compiler is where to find out. `fabric-interactor`
 * has the same test for the same reason -- `proof/roundtrip.c` there, this here, over a scene.
 *
 * It decodes the replies rather than eyeballing their length. A reply nobody can decode is not
 * a reply, and a byte count would pass on one.
 */

#include "mj_interactor.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* The same free-fall scene the physics test uses, so the two disagree only about how they
 * reach the number. */
static const char *FREEFALL_MJCF =
    "<mujoco>"
    "  <option timestep=\"0.001\"/>"
    "  <worldbody>"
    "    <body name=\"faller\" pos=\"0 0 10\">"
    "      <freejoint/>"
    "      <geom type=\"sphere\" size=\"0.1\" mass=\"1\"/>"
    "    </body>"
    "  </worldbody>"
    "</mujoco>";

/* ── Just enough CBOR to read a reply ──────────────────────────────────────────
 *
 * Only what RFC 8949 needs for what the interactor writes: unsigned, negative, text, array,
 * map, bool, and the break the writer uses for nothing here but might tomorrow. A decoder that
 * understood more would be testing itself.
 */

static int failures;

static const uint8_t *head(const uint8_t *p, const uint8_t *end, int *major, uint64_t *val)
{
    if (p >= end) return NULL;
    *major = (*p >> 5) & 7;
    uint64_t ai = *p & 31;
    p++;
    if (ai < 24) { *val = ai; return p; }
    int len = ai == 24 ? 1 : ai == 25 ? 2 : ai == 26 ? 4 : ai == 27 ? 8 : -1;
    if (len < 0 || end - p < len) return NULL;
    *val = 0;
    for (int i = 0; i < len; i++) *val = (*val << 8) | *p++;
    return p;
}

static const uint8_t *skip(const uint8_t *p, const uint8_t *end);

static const uint8_t *skip_items(const uint8_t *p, const uint8_t *end, uint64_t n)
{
    for (uint64_t i = 0; i < n && p; i++) p = skip(p, end);
    return p;
}

static const uint8_t *skip(const uint8_t *p, const uint8_t *end)
{
    int major;
    uint64_t val;

    if (p < end && *p == 0xff) return p + 1; /* break */
    p = head(p, end, &major, &val);
    if (!p) return NULL;
    switch (major) {
    case 0: case 1: case 7: return p;
    case 2: case 3: return end - p < (long)val ? NULL : p + val;
    case 4: return skip_items(p, end, val);
    case 5: return skip_items(p, end, val * 2);
    default: return NULL;
    }
}

/* The value for `key` in the map starting at `p`, or NULL. Keys are text and are compared
 * whole: a prefix match would find `x` inside `qx`. */
static const uint8_t *find(const uint8_t *p, const uint8_t *end, const char *key)
{
    int major;
    uint64_t pairs;

    p = head(p, end, &major, &pairs);
    if (!p || major != 5) return NULL;
    for (uint64_t i = 0; i < pairs; i++) {
        int kmajor;
        uint64_t klen;
        const uint8_t *k = head(p, end, &kmajor, &klen);
        if (!k || kmajor != 3 || end - k < (long)klen) return NULL;
        int match = strlen(key) == klen && memcmp(k, key, klen) == 0;
        p = skip(p, end);
        if (!p) return NULL;
        if (match) return p;
        p = skip(p, end);
        if (!p) return NULL;
    }
    return NULL;
}

static int64_t as_int(const uint8_t *p, const uint8_t *end)
{
    int major;
    uint64_t val;
    p = head(p, end, &major, &val);
    if (!p || (major != 0 && major != 1)) { failures++; return INT64_MIN; }
    return major == 0 ? (int64_t)val : -1 - (int64_t)val;
}

static int as_bool(const uint8_t *p, const uint8_t *end)
{
    int major;
    uint64_t val;
    p = head(p, end, &major, &val);
    if (!p || major != 7 || (val != 20 && val != 21)) { failures++; return -1; }
    return val == 21;
}

static int text_is(const uint8_t *p, const uint8_t *end, const char *want)
{
    int major;
    uint64_t len;
    const uint8_t *s = head(p, end, &major, &len);
    if (!s || major != 3 || end - s < (long)len) { failures++; return 0; }
    return strlen(want) == len && memcmp(s, want, len) == 0;
}

/* The first body's map, inside the `bodies` array. */
static const uint8_t *first_body(const uint8_t *reply, const uint8_t *end)
{
    int major;
    uint64_t items;
    const uint8_t *p = find(reply, end, "bodies");
    if (!p) { failures++; return NULL; }
    p = head(p, end, &major, &items);
    if (!p || major != 4 || items < 1) { failures++; return NULL; }
    return p;
}

static void check(int cond, const char *what)
{
    if (!cond) {
        failures++;
        fprintf(stderr, "FAIL %s\n", what);
    }
}

int main(void)
{
    mj_physics_t phys;
    char error[512] = {0};
    unsigned char reply[MJ_REPLY_MAX];
    int stop = 0;

    if (mj_physics_init(&phys, FREEFALL_MJCF, error, sizeof(error)) != 0) {
        fprintf(stderr, "mj_physics_init failed: %s\n", error);
        return 1;
    }

    weft_interactor_t in = mj_interactor(&phys);

    /* A scene that has not been stepped is at time zero, and its one body is where the MJCF
     * put it. This is also the check that `xpos` is populated before the first step: MuJoCo
     * fills it in `mj_makeData`'s forward pass, and a reply of zeros here would mean the
     * interactor is reading a frame nobody has computed. */
    size_t n = weft_ask(&in, "look", reply, sizeof reply, &stop);
    const uint8_t *end = reply + n;
    check(n > 0, "look answered");
    check(as_bool(find(reply, end, "ok"), end) == 1, "look ok");
    check(text_is(find(reply, end, "say"), end, "the scene"), "look says the scene");
    check(as_int(find(reply, end, "time_us"), end) == 0, "look at time zero");
    check(as_int(find(reply, end, "timestep_us"), end) == 1000, "timestep is 1 ms");
    check(as_int(find(reply, end, "nbody"), end) == 1, "the world body is not a body");

    const uint8_t *b = first_body(reply, end);
    check(b && text_is(find(b, end, "name"), end, "faller"), "the body is named");
    check(b && as_int(find(b, end, "z"), end) == 10000000, "the body starts at 10 m");
    check(b && as_int(find(b, end, "x"), end) == 0, "and on the axis");

    /* Half a second of free fall, in one command. The closed form is z0 - 0.5*g*t^2, and the
     * tolerance is the physics test's own 0.05 m: this is checking that the interactor reports
     * the scene MuJoCo integrated, not that MuJoCo integrates. */
    n = weft_ask(&in, "step 500", reply, sizeof reply, &stop);
    end = reply + n;
    check(as_bool(find(reply, end, "ok"), end) == 1, "step ok");
    check(as_int(find(reply, end, "time_us"), end) == 500000, "500 steps is 0.5 s");
    b = first_body(reply, end);
    int64_t z = b ? as_int(find(b, end, "z"), end) : 0;
    int64_t z_expected = (int64_t)llround((10.0 - 0.5 * 9.81 * 0.25) * 1000000.0);
    check(llabs(z - z_expected) <= 50000, "the body fell");

    /* A refusal must not move the scene. `step 0` and `step 2000` are out of range, and an
     * interactor that clamped would answer with a time the caller never asked for. */
    n = weft_ask(&in, "step 0", reply, sizeof reply, &stop);
    end = reply + n;
    check(as_bool(find(reply, end, "ok"), end) == 0, "step 0 refused");
    check(as_int(find(reply, end, "time_us"), end) == 500000, "and the scene did not move");

    n = weft_ask(&in, "step 2000", reply, sizeof reply, &stop);
    end = reply + n;
    check(as_bool(find(reply, end, "ok"), end) == 0, "step 2000 refused");
    check(as_int(find(reply, end, "time_us"), end) == 500000, "and the scene did not move");

    n = weft_ask(&in, "step 12x", reply, sizeof reply, &stop);
    end = reply + n;
    check(as_bool(find(reply, end, "ok"), end) == 0, "a count that is not a number is refused");

    /* A command nobody defined still gets a whole, decodable reply that says so. A service
     * that answered nothing would be indistinguishable from one that had died. */
    n = weft_ask(&in, "explode", reply, sizeof reply, &stop);
    end = reply + n;
    check(as_bool(find(reply, end, "ok"), end) == 0, "no such command");
    check(text_is(find(reply, end, "say"), end, "no such command"), "and says which");

    /* Too long is refused rather than cut. The truncation would parse: `step 1` is a valid
     * command and a wrong one. */
    char toolong[MJ_COMMAND_MAX + 64];
    memset(toolong, 'a', sizeof toolong - 1);
    toolong[sizeof toolong - 1] = '\0';
    memcpy(toolong, "step 1", 6);
    n = weft_ask(&in, toolong, reply, sizeof reply, &stop);
    end = reply + n;
    check(as_bool(find(reply, end, "ok"), end) == 0, "a long command is refused");
    check(as_int(find(reply, end, "time_us"), end) == 500000, "and did not step");

    /* A reply that does not fit is reported as nothing, never as a short one. */
    unsigned char tiny[8];
    check(weft_ask(&in, "look", tiny, sizeof tiny, &stop) == 0, "a reply that does not fit is 0");

    /* `stop` is the service's flag. The transport never reads it, and this is the only thing
     * that sets it. */
    check(stop == 0, "nothing has asked to stop yet");
    weft_ask(&in, "quit", reply, sizeof reply, &stop);
    check(stop == 1, "quit asks the service to wind down");

    mj_physics_close(&phys);

    if (failures) {
        fprintf(stderr, "mj_interactor: %d failed\n", failures);
        return 1;
    }
    printf("mj_interactor: a command in, reply bytes out, over a real scene -- pass\n");
    return 0;
}
