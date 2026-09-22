# SO-101 leader/follower tools

Local helpers for Arvind's SO-101 pair (Waveshare ST3215 servos on Waveshare
CH343 bus boards, 12 V supply).  Everything runs from the repo root through
`make`; the targets live in `so101.mk`, which the root `Makefile` includes.

```bash
make help            # list targets and current port/id settings
```

## Hardware map

Arms are selected by calibration id.  The id -> port table lives at the top of
`so101.mk`; add a line per board (port names embed the board's USB serial
number, so they survive replugging).

| Id           | Port                           | Notes                          |
|--------------|--------------------------------|--------------------------------|
| `follower_1` | `/dev/tty.usbmodem5B8E1123491` | Waveshare board, firmware 2307 |
| `follower_2` | `/dev/tty.usbmodem5C4C1251981` | other CH343 board, firmware 2819 |
| `leader_1`   | `/dev/tty.usbmodem5B8E1133621` | Waveshare board, encoder-only servos |

```bash
make arms                                  # which arms are known / connected
make teleop FOLLOWER_ID=follower_2         # any target accepts FOLLOWER_ID / LEADER_ID
make calibrate-follower FOLLOWER_ID=follower_2
```

A target whose arm is not plugged in stops with a message listing the
connected boards instead of a Python traceback.

## Daily use

```bash
make teleop                                  # leader drives follower, Ctrl+C stops
make teleop CAMERAS='{ front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}'
make angles                                  # compare joint angles before starting teleop
make record TASK=pick_cube TASK_DESC="Pick up the red cube" CAMERAS='...'
make replay TASK=pick_cube EPISODE=0
```

`teleop` and `record` pass `--robot.max_relative_target=10` by default so the
follower cannot be commanded more than 10 degrees from where it is in a single
step.  Set `MAX_REL=` (empty) to remove the cap once you trust the setup.

## First-time setup (in order)

```bash
make env                 # uv environment: python 3.12 + feetech + CLI extras
make ports               # both boards plugged in and powered?
make scan                # which motor IDs answer on each arm
make setup-follower      # interactive, one motor at a time (see notes below)
make setup-leader
make calibrate-follower  # centre joints, Enter, sweep every joint to both stops, Enter
make calibrate-leader
# or, non-interactively (handy when an agent is driving the session):
make cal-center FOLLOWER_ID=follower_2            # joints centred -> homing offsets
make cal-sweep  FOLLOWER_ID=follower_2 SECONDS=45 # sweep joints to both stops while it records, then it saves
make angles              # both arms in the same pose should agree within a few degrees
make teleop
```

### Motor ID assignment notes

Brand-new motors all ship as ID 1.  When several are chained, their replies
collide and lerobot's scanner sees nothing.  `make probe-follower` /
`make probe-leader` show the raw reply statistics and tell you whether the bus
is silent, colliding, or healthy.  To program one motor the chain must be
physically broken so only that motor reaches the board (unplug the cable from
the motor's *second* port too).  If the interactive script fails part way,
program the remaining motors individually:

```bash
make setup-one ARM=leader MOTOR=wrist_roll
```

### Calibration notes

Degrees are measured from the midpoint of each joint's recorded range, so
sweep every joint fully to both hard stops on both arms or leader and follower
will disagree by a constant offset.  A range of `0..4095` on any joint other
than `wrist_roll` means the joint was not centred before the first Enter; redo it.

Avoid centring a joint near the encoder's raw wrap point (a homing offset
close to +-2047 in the saved file is the tell).  Newer servo firmware (2819)
counts turns across that point, and the count is lost at power-up, so half
the joint's travel can read a full turn off after a restart.  The clean fix
is mechanical: detach the link from the horn, turn the shaft half a turn,
reattach, recalibrate.  `make hold-test` reports a joint resting outside its
calibrated range as SKIPPED rather than as a runaway.

## Troubleshooting

**`make` drops into an interactive Python prompt.**  Upstream's root Makefile
runs `.venv/bin/python` with no arguments to discover the interpreter path,
which opens the REPL when stdin is a terminal.  This fork resolves the path
with `command -v` instead; if you see `>>>` after a `git pull`, that line was
overwritten (press Ctrl-D to continue).


**A joint is "stuck" and pins against a stop at full load** (motor LED may
blink with an overload error).  Two servos did this on the follower.  In
position mode they drove *away* from any goal at full current, while speed mode
worked normally.  Cause: a stale internal multi-turn counter in the servo, which
makes every goal look a full revolution away.  Recovery:

```bash
make torque-off
make unstick IDS="2 3"       # speed-mode nudge, moves the joints a few degrees
make hold-test IDS="2 3"     # must report "holds ... TRACKS"
make eeprom-check            # confirm offsets/limits/phase still match the file
```

A power cycle of the follower motor supply clears it as well.  Do **not**
change the Phase register (18); every value other than 12 either runs away or
goes limp on these servos.

**Motor not found during setup.**  A completely silent bus (no bytes at any
ID or baudrate) is power or cabling: check the motor LED, the power supply
plug on the board, and reseat the 3-pin cable at both ends.

**"Incorrect status packet" errors.**  Usually collisions from duplicate IDs.
Run `make probe-follower` and look at the per-ID statistics.

**Voltage.**  The servos report the bus voltage in `make probe-*`; 12 V is
right for the 12 V ST3215 variant and wrong for the 7.4 V variant.
