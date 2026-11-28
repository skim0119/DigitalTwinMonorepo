PYTHON ?= python3
VENV ?= .venv
SPIROB_USER ?= $(shell id -un)

SYSTEMD_DIR := /etc/systemd/system
PI_SERVICES := spirob-gpio spirob-mocap spirob-experiment
PI_UNITS := $(addsuffix @$(SPIROB_USER).service,$(PI_SERVICES))
PI_TEMPLATES := $(addprefix deploy/,$(addsuffix @.service,$(PI_SERVICES)))
PI_UNIT_FILES := $(addprefix $(SYSTEMD_DIR)/,$(notdir $(PI_TEMPLATES)))
INSTALLED_PI_UNITS = $(patsubst %@.service,%@$(SPIROB_USER).service,$(notdir $(wildcard $(PI_UNIT_FILES))))

.PHONY: help install setup-pi remove-pi
help:
	@echo 'make install   Set up the Python environment and install local packages in editable mode'
	@echo 'make setup-pi   Build and install the Pi software, then enable and start its services'
	@echo 'make remove-pi  Stop, disable, and uninstall the Pi services'

install: $(VENV)/bin/python
	"$(VENV)/bin/python" -m pip install -e ./control_core -e ./experiment-runner -e . -r debug/requirements.txt

$(VENV)/bin/python:
	$(PYTHON) -m venv --system-site-packages "$(VENV)"

setup-pi:
	$(MAKE) install VENV=.venv
	$(MAKE) -C stream-gpio
	sudo install -m 0644 $(PI_TEMPLATES) $(SYSTEMD_DIR)/
	sudo systemctl daemon-reload
	sudo systemctl enable $(PI_UNITS)
	sudo systemctl restart $(PI_UNITS)

remove-pi:
	$(if $(INSTALLED_PI_UNITS),sudo systemctl disable --now $(INSTALLED_PI_UNITS))
	sudo rm -f -- $(PI_UNIT_FILES)
	sudo systemctl daemon-reload
