RTL := rtl/mag_acc_engine.v rtl/mae_fifo.v

.PHONY: all test test-icarus test-verilator model lint clean

all: lint model test

test:            ## both simulators, all parameter sets
	cd tb && python3 -m pytest -v test_runner.py

test-icarus:
	cd tb && python3 -m pytest -v test_runner.py -k icarus

test-verilator:  ## needs Verilator >= 5.036 for cocotb 2.x
	cd tb && python3 -m pytest -v test_runner.py -k verilator

model:           ## reference-model self-checks, no simulator
	cd tb && python3 -m pytest -q test_model.py

lint:
	verilator --lint-only -Wall --top-module mag_acc_engine $(RTL)

clean:
	rm -rf tb/sim_build tb/__pycache__ tb/.pytest_cache model/__pycache__
