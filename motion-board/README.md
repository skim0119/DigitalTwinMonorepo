# Motion Board

Motion Board is a local viewer for saved experiment recordings. Run experiments separately using the experiment scripts.

```sh
npm ci
npm run build
npm start
```

Open [Motion Board on port 8082](http://127.0.0.1:8082). It scans the checkout's `output/` directory recursively for completed run bundles and combined Exp 1/Exp 2 bending CSVs. Copy recordings there, press **Refresh**, and choose one. To scan another directory, use `npm start -- --output-dir /path/to/recordings`; use `--port` to change the local port. You can also import an Exp 1 or Exp 2 bending CSV directly.

Select trials to overlay their 3D trajectories. Drag to orbit, scroll to zoom, and use **Fit view** to recenter. The timeline moves orientation markers and shows command and load traces for the selected tendon. Select completed repeats of the same sequence to display their mean and spread.

For the focused trial, select a rigid body and mark an interval to flag, exclude, or interpolate a gap of at most 250 ms. Alternatively, select samples in the 3D view and delete them; undo restores them. Use **Download reviewed CSV** to save the cleaned poses, or **Download mean CSV** for the displayed mean. Original files are never changed. Review annotations and trial inclusion choices remain in memory only and are lost on refresh or closing the page, so export the reviewed CSV before leaving.

Run `npm test` for review and averaging tests.
