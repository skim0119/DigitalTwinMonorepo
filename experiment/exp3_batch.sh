# Exp 3 command sheet. Copy and run one line at a time on SpiRob.
# Copy the reset line between trials. Activate .venv before using it.
: <<'EXP3_COMMANDS'

# T0 fitting repeats
.venv/bin/python -u -m experiment.exp3_creep_recovery -N 0 --output-dir output/experiment/exp3_fit/T0/rep_1
python debug/homingV2.py; python debug/PID_multi.py --terminal
.venv/bin/python -u -m experiment.exp3_creep_recovery -N 0 --output-dir output/experiment/exp3_fit/T0/rep_2
python debug/homingV2.py; python debug/PID_multi.py --terminal
.venv/bin/python -u -m experiment.exp3_creep_recovery -N 0 --output-dir output/experiment/exp3_fit/T0/rep_3
python debug/homingV2.py; python debug/PID_multi.py --terminal

# T1 fitting repeats
.venv/bin/python -u -m experiment.exp3_creep_recovery -N 1 --output-dir output/experiment/exp3_fit/T1/rep_1
python debug/homingV2.py; python debug/PID_multi.py --terminal
.venv/bin/python -u -m experiment.exp3_creep_recovery -N 1 --output-dir output/experiment/exp3_fit/T1/rep_2
python debug/homingV2.py; python debug/PID_multi.py --terminal
.venv/bin/python -u -m experiment.exp3_creep_recovery -N 1 --output-dir output/experiment/exp3_fit/T1/rep_3
python debug/homingV2.py; python debug/PID_multi.py --terminal

# T2 fitting repeats
.venv/bin/python -u -m experiment.exp3_creep_recovery -N 2 --output-dir output/experiment/exp3_fit/T2/rep_1
python debug/homingV2.py; python debug/PID_multi.py --terminal
.venv/bin/python -u -m experiment.exp3_creep_recovery -N 2 --output-dir output/experiment/exp3_fit/T2/rep_2
python debug/homingV2.py; python debug/PID_multi.py --terminal
.venv/bin/python -u -m experiment.exp3_creep_recovery -N 2 --output-dir output/experiment/exp3_fit/T2/rep_3
python debug/homingV2.py; python debug/PID_multi.py --terminal

# T3 fitting repeats
.venv/bin/python -u -m experiment.exp3_creep_recovery -N 3 --output-dir output/experiment/exp3_fit/T3/rep_1
python debug/homingV2.py; python debug/PID_multi.py --terminal
.venv/bin/python -u -m experiment.exp3_creep_recovery -N 3 --output-dir output/experiment/exp3_fit/T3/rep_2
python debug/homingV2.py; python debug/PID_multi.py --terminal
.venv/bin/python -u -m experiment.exp3_creep_recovery -N 3 --output-dir output/experiment/exp3_fit/T3/rep_3
python debug/homingV2.py; python debug/PID_multi.py --terminal

# T4 fitting repeats
.venv/bin/python -u -m experiment.exp3_creep_recovery -N 4 --output-dir output/experiment/exp3_fit/T4/rep_1
python debug/homingV2.py; python debug/PID_multi.py --terminal
.venv/bin/python -u -m experiment.exp3_creep_recovery -N 4 --output-dir output/experiment/exp3_fit/T4/rep_2
python debug/homingV2.py; python debug/PID_multi.py --terminal
.venv/bin/python -u -m experiment.exp3_creep_recovery -N 4 --output-dir output/experiment/exp3_fit/T4/rep_3
python debug/homingV2.py; python debug/PID_multi.py --terminal

# T5 fitting repeats
.venv/bin/python -u -m experiment.exp3_creep_recovery -N 5 --output-dir output/experiment/exp3_fit/T5/rep_1
python debug/homingV2.py; python debug/PID_multi.py --terminal
.venv/bin/python -u -m experiment.exp3_creep_recovery -N 5 --output-dir output/experiment/exp3_fit/T5/rep_2
python debug/homingV2.py; python debug/PID_multi.py --terminal
.venv/bin/python -u -m experiment.exp3_creep_recovery -N 5 --output-dir output/experiment/exp3_fit/T5/rep_3
EXP3_COMMANDS
