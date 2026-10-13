# Simulation workflow for Ubuntu / WSL. Run `make help` for available targets.
SHELL := /bin/bash
.SHELLFLAGS := -eo pipefail -c
.DEFAULT_GOAL := help
# Prevent cleanup and build recipes from racing when make is called with -j.
.NOTPARALLEL:

SIMULATION_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
ROS_DISTRO ?= jazzy
ROS_SETUP ?= /opt/ros/$(ROS_DISTRO)/setup.bash
WORLD ?= approach
GUI ?= true
FLIGHT ?= true

.PHONY: help check setup deps build run alpha beta gamma approach test verify benchmark clean rebuild

help:
	@printf '%s\n' \
	  'Simulation commands (Ubuntu / WSL, ROS 2 Jazzy)' \
	  '  make setup      Install dependencies and build (sudo/network required)' \
	  '  make check      Check installed simulation tools' \
	  '  make build      Build the three ROS packages' \
	  '  make run        Build and launch approach, GUI and flight enabled' \
	  '  make alpha      Build and launch a static Alpha demo' \
	  '  make beta       Build and launch a static Beta demo' \
	  '  make gamma      Build and launch a static Gamma demo' \
	  '  make test       Run offline simulation tests' \
	  '  make verify     Build and run scenario verification (forceful process cleanup)' \
	  '  make benchmark  Build and run the host inference benchmark' \
	  '  make clean      Delete ONLY simulation build/, install/ and log/' \
	  '  make rebuild    Clean, then build again' \
	  'Options: make run WORLD=beta GUI=false FLIGHT=false' \
	  '         make alpha GUI=false' \
	  'ROS_SETUP and ROS_DISTRO can be overridden; Jazzy is the supported distro.'

check:
	@test -f "$(ROS_SETUP)" || { printf 'ROS setup missing: %s\nInstall ROS 2 Jazzy first.\n' "$(ROS_SETUP)"; exit 1; }
	@source "$(ROS_SETUP)"; \
	for tool in python3 colcon ros2 gz; do \
	  command -v "$$tool" >/dev/null || { printf 'Missing tool: %s. Run make setup.\n' "$$tool"; exit 1; }; \
	done; \
	ros2 pkg prefix ros_gz_bridge >/dev/null; \
	ros2 pkg prefix ros_gz_sim >/dev/null; \
	printf 'Simulation tools available.\n'

# ROS itself and its apt sources must already be configured. Keep installations
# explicit: build/run/test/clean never install system packages.
deps:
	@test -f "$(ROS_SETUP)" || { printf 'Install ROS 2 Jazzy before make setup.\n'; exit 1; }
	@sudo apt-get update
	@sudo apt-get install -y python3-colcon-common-extensions python3-rosdep \
	  ros-$(ROS_DISTRO)-ros-gz-bridge ros-$(ROS_DISTRO)-ros-gz-sim \
	  python3-numpy python3-pandas python3-sklearn
	@source "$(ROS_SETUP)"; \
	if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then sudo rosdep init; fi; \
	rosdep update; \
	rosdep install --from-paths "$(SIMULATION_DIR)/src" --ignore-src --rosdistro "$(ROS_DISTRO)" -r -y

setup: deps build

build: check
	@cd "$(SIMULATION_DIR)"; source "$(ROS_SETUP)"; \
	colcon build --base-paths src --symlink-install

run: build
	@case "$(WORLD)" in alpha|beta|gamma|approach) ;; *) printf 'WORLD must be alpha, beta, gamma or approach.\n'; exit 1;; esac; \
	case "$(GUI):$(FLIGHT)" in true:true|true:false|false:true|false:false) ;; *) printf 'GUI and FLIGHT must be true or false.\n'; exit 1;; esac; \
	cd "$(SIMULATION_DIR)"; source "$(ROS_SETUP)"; source install/setup.bash; \
	ros2 launch launch/system_launch.py "world:=$(WORLD)" "gui:=$(GUI)" "flight:=$(FLIGHT)"

# Reuse run so presets follow the same build and environment setup.
alpha beta gamma:
	@$(MAKE) --no-print-directory -C "$(SIMULATION_DIR)" run WORLD=$@ FLIGHT=false GUI="$(GUI)"

approach:
	@$(MAKE) --no-print-directory -C "$(SIMULATION_DIR)" run WORLD=approach FLIGHT=true GUI="$(GUI)"

test:
	@cd "$(SIMULATION_DIR)"; python3 -m unittest discover -s tests -p 'test_*.py' -v

verify: build
	@cd "$(SIMULATION_DIR)"; source "$(ROS_SETUP)"; source install/setup.bash; \
	python3 scripts/verify_system.py

benchmark: build
	@cd "$(SIMULATION_DIR)"; source "$(ROS_SETUP)"; source install/setup.bash; \
	ros2 run intelligence_domain benchmark_inference

clean:
	@python3 "$(SIMULATION_DIR)/scripts/clean_workspace.py"

rebuild: clean build
