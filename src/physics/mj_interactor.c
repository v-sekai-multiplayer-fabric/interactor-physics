// The physics interactor: a command in, reply bytes out, over a MuJoCo scene.
//
// It has no socket, no poll loop and no idea what carried the command here. That is the whole
// of what this file adds to `mj_physics.c`, and it is the difference between a physics library
// and a physics service: `mj_physics_step` is a function a caller must already be linked
// against, and this is a command a caller can send from another process, another machine, or a
// test with no transport at all.
//
// SPDX-License-Identifier: Apache-2.0

#include "mj_interactor.h"

#include "weft/cbor.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// The most steps one command may ask for. It refuses past this rather than clamping: a clamped
// `step` answers with a scene at a time the caller did not ask for and cannot distinguish from
// the one it did, which is the same failure as a truncated batch. The number is 1000 because a
// scene at MuJoCo's default 2 ms timestep is then two seconds of simulation in one answer, and
// an interactor that takes longer than that to reply has stopped being a service and become a
// batch job.
#define MJ_STEP_MAX 1000

// ── The scene, as a caller sees it ────────────────────────────────────────────

// Everything integral, and everything absolute.
//
// A body's position comes out of `data->xpos`, which is the body frame in world coordinates
// after the last `mj_forward`/`mj_step` — not `qpos`, which is joint coordinates and means
// nothing to a reader who does not have the model. A subscriber matching this against an
// `XRGridEntityPacket` is comparing world positions, so this is the one that answers.
static int64_t um(mjtNum metres) { return (int64_t)llround((double)metres * MJ_UM_PER_M); }

// The world body is index 0 and is always at the origin with an identity rotation. It is left
// out rather than sent: it is a row every reply would carry, in every tick, saying nothing.
static void say_bodies(weft_cbor_t *c, const mj_physics_t *phys) {
	const mjModel *m = phys->model;
	const mjData *d = phys->data;

	weft_cbor_text(c, "bodies");
	weft_cbor_array(c, (uint64_t)(m->nbody - 1));
	for (int i = 1; i < m->nbody; i++) {
		const char *name = mj_id2name(m, mjOBJ_BODY, i);

		weft_cbor_map(c, 8);
		weft_cbor_kv_int(c, "id", i);
		weft_cbor_kv_text(c, "name", name ? name : "");
		weft_cbor_kv_int(c, "x", um(d->xpos[3 * i + 0]));
		weft_cbor_kv_int(c, "y", um(d->xpos[3 * i + 1]));
		weft_cbor_kv_int(c, "z", um(d->xpos[3 * i + 2]));
		// The rotation is a unit quaternion, so scaling by the same million a metre gets keeps
		// it inside ±1000000 and integral. It is not swing-twist: `XRGridEntityPacket` packs
		// three i16 swing-twist components, and that conversion belongs where the packet is
		// written, because doing it here would make every caller that wants a quaternion undo
		// it. `w` is dropped for the same reason it is dropped on the wire — a unit quaternion
		// recovers it, and sending it invites a reader to trust a fourth number that is not
		// independent of the other three.
		weft_cbor_kv_int(c, "qx", um(d->xquat[4 * i + 1]));
		weft_cbor_kv_int(c, "qy", um(d->xquat[4 * i + 2]));
		weft_cbor_kv_int(c, "qz", um(d->xquat[4 * i + 3]));
	}
}

// The scalars a caller needs to know what it is looking at, and when.
//
// `time` is microseconds of simulated time, not wall-clock. A caller comparing two replies is
// asking how far the scene moved, and wall-clock would answer how busy the machine was.
static void say_scene(weft_cbor_t *c, const mj_physics_t *phys) {
	weft_cbor_text(c, "time_us");
	weft_cbor_int(c, um(phys->data->time));
	weft_cbor_text(c, "timestep_us");
	weft_cbor_int(c, um(phys->model->opt.timestep));
	weft_cbor_text(c, "nbody");
	weft_cbor_int(c, phys->model->nbody - 1);
	say_bodies(c, phys);
}

// ── The commands ──────────────────────────────────────────────────────────────

// `step` takes a count, and a count is the only argument anything here takes. It is parsed
// whole: `strtol` stopping early means the caller typed something that is not a number, and
// running the prefix it did understand would be answering a command nobody sent.
static int step_count(const char *arg, int *out) {
	char *end;
	long n;

	if (arg == NULL || *arg == '\0') {
		*out = 1;
		return 1;
	}
	n = strtol(arg, &end, 10);
	while (*end == ' ') end++;
	if (*end != '\0' || n < 1 || n > MJ_STEP_MAX) return 0;
	*out = (int)n;
	return 1;
}

static int mj_command(mj_physics_t *phys, char *line, weft_cbor_t *c, int *stop) {
	char *verb = line, *arg = NULL;
	const char *say;
	int ok = 1, n;

	// A leading slash is how a person types a command and how the ward's interactor reads one.
	// Accepting both spellings costs two characters here and saves every caller a rule.
	while (*verb == '/' || *verb == ' ') verb++;
	for (char *p = verb; *p; p++) {
		if (*p == ' ') {
			*p = '\0';
			arg = p + 1;
			while (*arg == ' ') arg++;
			break;
		}
	}

	if (!strcmp(verb, "look") || !*verb) {
		say = "the scene";
	} else if (!strcmp(verb, "step")) {
		if (!step_count(arg, &n)) {
			ok = 0;
			say = "step takes a count from 1 to 1000";
		} else {
			for (int i = 0; i < n; i++) mj_physics_step(phys);
			say = "stepped";
		}
	} else if (!strcmp(verb, "quit")) {
		*stop = 1;
		say = "goodbye";
	} else {
		ok = 0;
		say = "no such command";
	}

	// Six: `ok`, `say`, and the four `say_scene` writes. A definite map that undercounts is not
	// a short reply, it is a malformed one — the reader stops at the declared pair and the rest
	// of the frame becomes trailing bytes it never sees.
	weft_cbor_map(c, 6);
	weft_cbor_kv_bool(c, "ok", ok);
	weft_cbor_kv_text(c, "say", say);
	say_scene(c, phys);
	return *stop;
}

// ── The contract ──────────────────────────────────────────────────────────────

static size_t mj_ask(void *ctx, const char *command, unsigned char *reply, size_t cap,
                     int *stop) {
	mj_physics_t *phys = (mj_physics_t *)ctx;
	char line[MJ_COMMAND_MAX];

	weft_cbor_t c = weft_cbor_to(reply, cap);

	// Too long is refused here rather than copied and cut. `snprintf` alone would truncate, and
	// a truncated command is a different command that still parses: `step 1000` arriving as
	// `step 100` is a scene nine hundred steps short of what was asked for, answered `ok`.
	if (strlen(command) >= sizeof line) {
		weft_cbor_map(&c, 6);
		weft_cbor_kv_bool(&c, "ok", 0);
		weft_cbor_kv_text(&c, "say", "command too long");
		say_scene(&c, phys);
		return weft_cbor_over(&c) ? 0 : c.n;
	}

	// `mj_command` writes NULs into the line to split it, and the command is the transport's
	// memory until it says otherwise. So it gets a copy it may own.
	snprintf(line, sizeof line, "%s", command);

	(void)mj_command(phys, line, &c, stop);
	if (weft_cbor_over(&c)) {
		// A scene too big to describe in one reply is a real limit, and it is reported rather
		// than silently cut: a truncated batch decodes as a short one.
		fprintf(stderr, "the scene does not fit in %zu bytes\n", cap);
		return 0;
	}
	return c.n;
}

weft_interactor_t mj_interactor(mj_physics_t *phys) {
	weft_interactor_t in = {mj_ask, phys};
	return in;
}
