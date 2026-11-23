# Exp 3 command sheet. Copy and run one line at a time on SpiRob.
# Copy the reset line between trials. Activate .venv before using it.
python -u -m experiment.exp3_creep_recovery -N 0 -F 1.0 --output-dir output/experiment/exp3_fit/T0/1.0kgf/rep_1
python debug/homingV2.py; python debug/PID_multi.py --terminal
python -u -m experiment.exp3_creep_recovery -N 0 -F 1.0 --output-dir output/experiment/exp3_fit/T0/1.0kgf/rep_2
python debug/homingV2.py; python debug/PID_multi.py --terminal
python -u -m experiment.exp3_creep_recovery -N 0 -F 2.5 --output-dir output/experiment/exp3_fit/T0/2.5kgf/rep_1
python debug/homingV2.py; python debug/PID_multi.py --terminal
python -u -m experiment.exp3_creep_recovery -N 0 -F 2.5 --output-dir output/experiment/exp3_fit/T0/2.5kgf/rep_2
