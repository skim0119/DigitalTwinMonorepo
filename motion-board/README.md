# Motion Board

A local console for reviewing Spirob actuation experiments. To browse saved recordings:

```sh
npm ci
npm run build
npm start
```

Open [Motion Board on port 8082](http://127.0.0.1:8082). **Local results** scans
the checkout’s `output/` directory, including nested folders, for downloaded API
run bundles and combined Exp 1 or Exp 2 recordings. Choose a recording to load its
poses and controls. Use **Refresh** to find new files or start with
`npm start -- --output-dir /path/to/recordings` to open another directory.

For live operation, start [Experiment API](../experiment-api/README.md) on
Spirob, then launch with `npm start -- --ssh spirob` and choose **Experiment API**.
The launcher uses the existing OpenSSH host alias and forwards HTTP and WebSocket requests to the
Pi's loopback port. Start with `npm start` if Experiment API is running on
this computer, such as during simulation. Use `--port`, `--api-port`, or `--forward-port` to override the defaults.
The default SSH tunnel binds locally on port 15052.

1. Choose **New experiment**, select the mode and units, and upload the CSV.
   Validate the table before starting. Set the trial count to repeat the same
   sequence. Stop batch cancels the remaining trials.
2. Use the run list checkboxes to overlay trajectories. Click a trial to focus
   its timeline, commands, loads, and downloads. Drag the 3D view to orbit,
   scroll to zoom, and use **Fit view** to recenter. The timeline moves the
   orientation triads along each trajectory.
3. Select a rigid body and mark the start and end of a review interval. Flag
   it, exclude it, or interpolate if the surrounding gap is at most 250 ms.
   The raw pose download always contains the original recording. Undo removes
   an annotation. For individual samples, choose **Select samples** and drag a
   box in the 3D view. Drag again to add to the selection. Press **Delete**
   or use **Delete selected**, then choose **Undo last deletion** to restore
   the entire selection. These edits apply to the focused trial.
4. Select completed repeats with the same actuation table to show their mean and spread.
   Use **Include this trial in the mean** to exclude a trial while retaining
   its visible trajectory. The mean uses elapsed time, a 50 Hz grid, and a
   sign-aligned normalized quaternion mean.
5. Download the raw pose, control, or actuation CSV, the reviewed CSV, or the
   mean CSV. Downloads are ordinary files for post-processing. Experiment API
   also keeps the original data on Spirob.

Run `npm test` for review and averaging tests.

Use **Import bending CSV** in the Runs sidebar to open an
[Exp 1 bending recording](../experiment/README.md) from disk. Each cycle appears
as a trial with pose trajectories and the recorded tendon force traces.
