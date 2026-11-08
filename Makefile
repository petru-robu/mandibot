# Makefile
.PHONY: run upload python setup

run: upload python

setup:
	uv venv
	uv pip install -r host/requirements.txt

upload:
	pio run --target upload

python:
	uv run --no-project python host/controller.py
