# Makefile
.PHONY: run upload python

run: upload python

upload:
	pio run --target upload

python:
	python host/controller.py