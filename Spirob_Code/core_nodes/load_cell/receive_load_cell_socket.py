import socket
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from numpy.char import lower

HOST = '172.16.97.100'    # The remote host
PORT = 50007              # The same port as used by the server

measurements = []
elapsed_times = []

# set up live plot
WINDOW_SIZE = 5 # seconds of data to show at one time

plt.ion()
fig, ax = plt.subplots()
live_graph, = ax.plot(elapsed_times, measurements)

graph_closed = False
def on_close(event): 
    global graph_closed
    graph_closed = True
fig.canvas.mpl_connect('close_event', on_close)

lower_window_index = 0

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    print(f"Listening to {HOST} at Port {PORT}...")
    s.connect((HOST, PORT))
    try:
        while not graph_closed:
            data = s.recv(1024)
            if data:
                print('Received', data.decode())
                try:
                    measurement, elapsed_time = data.decode().split(',')[:2]
                    measurement = int(measurement)
                    elapsed_time = float(elapsed_time)
                except:
                    print("WARNING: Failed to parse data.")
                    continue
                
                measurements.append(measurement)
                elapsed_times.append(elapsed_time)

                xlim_lower = max(0, elapsed_time - .1 - WINDOW_SIZE)

                while elapsed_times[lower_window_index] < xlim_lower:
                    lower_window_index += 1

                live_graph.set_xdata(elapsed_times[lower_window_index:])
                displayed_measurements = measurements[lower_window_index:]
                live_graph.set_ydata(displayed_measurements)

                ax.set_xlim(max(0, elapsed_time - .1 - WINDOW_SIZE), elapsed_time + .1)
                ax.set_ylim(min(displayed_measurements) - 1, max(displayed_measurements) + 1)

                fig.canvas.draw()
                fig.canvas.flush_events()
    except KeyboardInterrupt:
        pass
    finally:
        print(f"Collected {len(measurements)} measurements over {elapsed_times[-1]:.2f}s")
        with open(f"socket_measurements.txt", "w") as f:
            measurements_str = ",".join(map(str, measurements))
            elapsed_times_str = ",".join(map(str, elapsed_times))
            f.write(f"{measurements_str}\n{elapsed_times_str}\n")