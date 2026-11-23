PYTHON ?= python3
VENV ?= .venv
SPIROB_USER ?= $(shell id -un)

SENSOR_UNITS := spirob-gpio@$(SPIROB_USER).service spirob-mocap@$(SPIROB_USER).service
SENSOR_TEMPLATES := deploy/spirob-gpio@.service deploy/spirob-mocap@.service
SYSTEMD_DIR := /etc/systemd/system

.PHONY: help install setup-pi remove-pi
help:
	@echo 'make install   Set up the Python environment and install local packages in editable mode'
	@echo 'make setup-pi   Build the GPIO streamer and enable the Pi sensor services'
	@echo 'make remove-pi  Stop, disable, and uninstall the Pi sensor services'

install: $(VENV)/bin/python
	"$(VENV)/bin/python" -m pip install -e ./control_core -e ./experiment-runner -e . -r debug/requirements.txt

$(VENV)/bin/python:
	$(PYTHON) -m venv --system-site-packages "$(VENV)"

setup-pi:
	$(MAKE) -C stream-gpio
	sudo install -m 0644 $(SENSOR_TEMPLATES) $(SYSTEMD_DIR)/
	sudo systemctl daemon-reload
	sudo systemctl enable --now $(SENSOR_UNITS)

remove-pi:
	sudo systemctl disable --now $(SENSOR_UNITS)
	sudo rm -f -- $(addprefix $(SYSTEMD_DIR)/,$(notdir $(SENSOR_TEMPLATES)))
	sudo systemctl daemon-reload
