// The physics interactor: a command in, reply bytes out, over a MuJoCo scene.
//
// SPDX-License-Identifier: Apache-2.0
#ifndef MJ_INTERACTOR_H
#define MJ_INTERACTOR_H

#include "mj_physics.h"

#include "weft/interactor.h"

// The longest command this will read. A longer one is dropped rather than truncated, because a
// truncated command is a different command: `step 1000` cut to `step 10` is a scene ninety-nine
// hundredths of a tick behind what the caller asked for, and nothing downstream can tell.
#define MJ_COMMAND_MAX 512

// The largest reply. It is `WARD_REPLY_MAX` from `fabric-store-domain`, deliberately: a caller
// that can hold one zone's snapshot must be able to hold one scene's, and two limits that mean
// the same thing drift apart the first time one of them is raised.
#define MJ_REPLY_MAX 262144

// Positions and velocities leave here as int64 micrometres, never as mjtNum.
//
// MuJoCo works in metres of double, and `XRGridEntityPacket` is integral all the way across —
// int64 absolute micrometres for position, no floats anywhere on the wire. Converting at the
// interactor rather than at the transport is what keeps the reply comparable to the packet a
// subscriber already has: a float that crossed a wire and came back is not the number that
// left, so a differential check against the packet's golden vectors would fail on the codec
// and tell you nothing about the physics.
#define MJ_UM_PER_M 1000000.0

// Binds a scene to the contract. The interactor borrows `phys` and does not own it: the caller
// loaded the scene and the caller closes it, because a `quit` command that freed the model
// would be a client deciding when a service's state ends.
weft_interactor_t mj_interactor(mj_physics_t *phys);

#endif
