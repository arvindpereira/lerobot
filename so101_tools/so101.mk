# ---------------------------------------------------------------------------
# SO-101 leader/follower helper targets.  Included from the root Makefile so
# every target works from the repo root:   make teleop
#
# Override any variable on the command line, e.g.
#   make teleop FOLLOWER_PORT=/dev/tty.usbmodemXXXX MAX_REL=20
# Port names embed the board's USB serial number, so they are stable across
# replugs on the same Mac.  `make ports` lists what is currently connected.
# ---------------------------------------------------------------------------

FOLLOWER_PORT ?= /dev/tty.usbmodem5B8E1123491
LEADER_PORT   ?= /dev/tty.usbmodem5B8E1133621
FOLLOWER_ID   ?= follower_1
LEADER_ID     ?= leader_1
ROBOT_TYPE    ?= so101_follower
TELEOP_TYPE   ?= so101_leader
FPS           ?= 30
MAX_REL       ?= 10          # --robot.max_relative_target (deg per step); empty disables the cap
CAMERAS       ?=             # e.g. "{ front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}"
HF_USER       ?= $(shell NO_COLOR=1 hf auth whoami 2>/dev/null | awk -F': *' 'NR==1 {print $$2}')
TASK          ?= my_task
TASK_DESC     ?= Describe the task in one sentence
NUM_EPISODES  ?= 50
EPISODE_TIME  ?= 30
RESET_TIME    ?= 10
EPISODE       ?= 0

TOOLS := so101_tools
RUN   := uv run

ROBOT_ARGS  := --robot.type=$(ROBOT_TYPE) --robot.port=$(FOLLOWER_PORT) --robot.id=$(FOLLOWER_ID)
TELEOP_ARGS := --teleop.type=$(TELEOP_TYPE) --teleop.port=$(LEADER_PORT) --teleop.id=$(LEADER_ID)
ifneq ($(strip $(MAX_REL)),)
ROBOT_ARGS += --robot.max_relative_target=$(MAX_REL)
endif
ifneq ($(strip $(CAMERAS)),)
ROBOT_ARGS += --robot.cameras="$(CAMERAS)"
endif

# Refuse to touch a serial port that another process (e.g. a running teleop) already has open.
# Two readers on one port corrupt each other's packets, and connecting through lerobot disables
# torque on disconnect, which would drop the follower mid-teleop.
define require_port_free
	@busy=$$(lsof -n -P -t $(1) 2>/dev/null </dev/null); if [ -n "$$busy" ]; then \
		echo "ERROR: $(1) is in use by PID $$busy:"; ps -o command= -p $$busy | head -1; \
		echo "Stop that process (Ctrl+C in its terminal) and retry."; exit 1; fi
endef

.PHONY: help ports find-port scan scan-follower scan-leader probe-follower probe-leader \
        setup-follower setup-leader setup-one calibrate-follower calibrate-leader \
        angles teleop record replay eeprom-check hold-test unstick torque-off env

help:  ## list SO-101 targets
	@echo "SO-101 targets (run from repo root):"
	@grep -hE '^[a-zA-Z0-9_-]+:.*## ' $(TOOLS)/so101.mk | sort | awk -F':.*## ' '{printf "  %-20s %s\n", $$1, $$2}'
	@echo
	@echo "Current settings: FOLLOWER_PORT=$(FOLLOWER_PORT) LEADER_PORT=$(LEADER_PORT) FOLLOWER_ID=$(FOLLOWER_ID) LEADER_ID=$(LEADER_ID)"

env:  ## install / refresh the uv environment (python 3.12, feetech + CLI extras)
	uv sync --locked -p 3.12 --extra feetech --extra core_scripts

ports:  ## list connected USB serial boards
	@ls /dev/tty.usbmodem* 2>/dev/null || echo "no /dev/tty.usbmodem* devices found"

find-port:  ## interactive: unplug one board when prompted to learn its port
	$(RUN) lerobot-find-port

scan: scan-follower scan-leader  ## list motor IDs answering on both arms

scan-follower:  ## list motor IDs on the follower bus
	$(call require_port_free,$(FOLLOWER_PORT))
	$(RUN) python $(TOOLS)/bus_probe.py $(FOLLOWER_PORT)

scan-leader:  ## list motor IDs on the leader bus
	$(call require_port_free,$(LEADER_PORT))
	$(RUN) python $(TOOLS)/bus_probe.py $(LEADER_PORT)

probe-follower:  ## deep probe: all baudrates + collision statistics (follower)
	$(call require_port_free,$(FOLLOWER_PORT))
	$(RUN) python $(TOOLS)/bus_probe.py $(FOLLOWER_PORT) --deep

probe-leader:  ## deep probe: all baudrates + collision statistics (leader)
	$(call require_port_free,$(LEADER_PORT))
	$(RUN) python $(TOOLS)/bus_probe.py $(LEADER_PORT) --deep

setup-follower:  ## interactive: assign motor IDs on the follower, one motor at a time
	$(call require_port_free,$(FOLLOWER_PORT))
	$(RUN) lerobot-setup-motors --robot.type=$(ROBOT_TYPE) --robot.port=$(FOLLOWER_PORT)

setup-leader:  ## interactive: assign motor IDs on the leader, one motor at a time
	$(call require_port_free,$(LEADER_PORT))
	$(RUN) lerobot-setup-motors --teleop.type=$(TELEOP_TYPE) --teleop.port=$(LEADER_PORT)

setup-one:  ## program ONE isolated motor: make setup-one ARM=leader MOTOR=wrist_roll
	$(call require_port_free,$(if $(filter leader,$(ARM)),$(LEADER_PORT),$(FOLLOWER_PORT)))
	@test -n "$(ARM)" -a -n "$(MOTOR)" || (echo "usage: make setup-one ARM=follower|leader MOTOR=<name>"; exit 1)
	$(RUN) python $(TOOLS)/setup_one_motor.py $(if $(filter leader,$(ARM)),$(LEADER_PORT),$(FOLLOWER_PORT)) $(ARM) $(MOTOR)

calibrate-follower:  ## interactive: calibrate the follower (type 'c' at the prompt to redo)
	$(call require_port_free,$(FOLLOWER_PORT))
	$(RUN) lerobot-calibrate --robot.type=$(ROBOT_TYPE) --robot.port=$(FOLLOWER_PORT) --robot.id=$(FOLLOWER_ID)

calibrate-leader:  ## interactive: calibrate the leader (type 'c' at the prompt to redo)
	$(call require_port_free,$(LEADER_PORT))
	$(RUN) lerobot-calibrate --teleop.type=$(TELEOP_TYPE) --teleop.port=$(LEADER_PORT) --teleop.id=$(LEADER_ID)

angles:  ## read both arms' joint angles through their calibrations
	$(call require_port_free,$(FOLLOWER_PORT))
	$(call require_port_free,$(LEADER_PORT))
	$(RUN) python $(TOOLS)/check_angles.py --follower-port $(FOLLOWER_PORT) --leader-port $(LEADER_PORT) \
		--follower-id $(FOLLOWER_ID) --leader-id $(LEADER_ID)

teleop:  ## leader drives follower (Ctrl+C to stop). Add CAMERAS=... to display cameras
	$(call require_port_free,$(FOLLOWER_PORT))
	$(call require_port_free,$(LEADER_PORT))
	$(RUN) lerobot-teleoperate $(ROBOT_ARGS) $(TELEOP_ARGS) --fps=$(FPS) $(if $(strip $(CAMERAS)),--display_data=true,)

record:  ## record a dataset: make record TASK=pick_cube TASK_DESC="Pick up the cube" CAMERAS=...
	$(call require_port_free,$(FOLLOWER_PORT))
	$(call require_port_free,$(LEADER_PORT))
	@test -n "$(HF_USER)" || (echo "not logged in to Hugging Face: run 'uv run hf auth login'"; exit 1)
	$(RUN) lerobot-record $(ROBOT_ARGS) $(TELEOP_ARGS) --fps=$(FPS) \
		--dataset.repo_id=$(HF_USER)/$(TASK) --dataset.single_task="$(TASK_DESC)" \
		--dataset.num_episodes=$(NUM_EPISODES) --dataset.episode_time_s=$(EPISODE_TIME) \
		--dataset.reset_time_s=$(RESET_TIME) --display_data=true

replay:  ## replay one recorded episode on the follower: make replay TASK=pick_cube EPISODE=0
	$(call require_port_free,$(FOLLOWER_PORT))
	$(RUN) lerobot-replay $(ROBOT_ARGS) --dataset.repo_id=$(HF_USER)/$(TASK) --dataset.episode=$(EPISODE)

eeprom-check:  ## verify follower motor EEPROM (offsets, limits, phase, mode) matches its calibration file
	$(call require_port_free,$(FOLLOWER_PORT))
	$(RUN) python $(TOOLS)/eeprom_check.py $(FOLLOWER_PORT) follower $(FOLLOWER_ID)

hold-test:  ## safe runaway test on follower motors (goal = current position): make hold-test IDS="2 3"
	$(call require_port_free,$(FOLLOWER_PORT))
	$(RUN) python $(TOOLS)/hold_test.py $(FOLLOWER_PORT) $(IDS)

unstick:  ## recover follower joints pinned at a stop at full load (speed-mode nudge): make unstick IDS="2 3"
	$(call require_port_free,$(FOLLOWER_PORT))
	$(RUN) python $(TOOLS)/unstick.py $(FOLLOWER_PORT) $(IDS)

torque-off:  ## disable torque on every follower motor
	$(call require_port_free,$(FOLLOWER_PORT))
	$(RUN) python $(TOOLS)/unstick.py $(FOLLOWER_PORT) --torque-off-only
