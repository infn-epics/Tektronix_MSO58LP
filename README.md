# Tektronix MSO58LP IOC (ASYN)

EPICS IOC for Tektronix MSO58LP waveform acquisition with ASYN-based polling,
8 channel support, per-channel processing, OPI controls, and waveform session
storage to Tek-style `.csv` files.

## Latest Updates

- Waveform plots now use sample-index X axis in both main and channel OPI
	displays. This avoids distortion when X arrays are stale or mismatched.
- X-axis title in OPI updated to `Samples (window relative)`.
- Driver now pads trailing waveform/time buffers with `NaN` after each read,
	preventing stale samples from previous larger acquisitions.
- Added per-channel file storage engine via `tekStoreWaveform` aSub.
- Storage files now use Tek-style CSV layout (metadata + `TIME,<CH>` table).

## Build

```bash
cd /app/IOC-DEVEL/Tektronix_MSO58LP
make -j4
```

## Run IOC

```bash
cd /app/IOC-DEVEL/Tektronix_MSO58LP/iocBoot/iocTECKMSO58LP
./st-all-asyn.cmd
```

Default startup values in [iocBoot/iocTECKMSO58LP/st-all-asyn.cmd](iocBoot/iocTECKMSO58LP/st-all-asyn.cmd):

- Prefix: `SPARC:DIAG:TEK-TEST`
- Instrument endpoint: `ddsparctekmso001.lnf.infn.it:4004`
- Poll time: `0.1 s`
- Waveform NELM: `10000`
- Initial data start: `30000`
- Channels loaded: `CH1..CH8`

## OPI Guide

Main OPI file: [opi/mso_asyn.bob](opi/mso_asyn.bob)

Per-channel OPI file: [opi/channel_asyn_display.bob](opi/channel_asyn_display.bob)

### 1. Open the display

- Open the `.bob` file in Phoebus Display Builder.
- Provide macro `P` matching IOC prefix (example: `SPARC:DIAG:TEK-TEST`).

### 2. Basic waveform operation

- Enable desired channels and verify waveform traces are updating.
- Adjust each channel `DataStart` (`$(P):CHx:DataStart`) to shift the acquired
	window.
- Read back actual instrument value from `DataStart_RBV`.
- Use marker controls (`MarkerStart`, `MarkerEnd`, `MarkerStartPt`,
	`MarkerEndPt`) to inspect local waveform windows.

### 3. Centering and edge tracking

- Use channel-level edge/centering controls from the channel display.
- Centering logic places the edge with configurable pre-edge margin.
- `Copy CH1 Start->All` on main OPI replicates CH1 start marker to all channels.

### 4. Session storage from OPI

Global controls:

- `$(P):Store:Path` -> output directory
- `$(P):Store:Basename` -> filename prefix
- `$(P):Store:NumAcq` -> target acquisitions per enabled channel
- `$(P):Store:Start` -> start storage session
- `$(P):Store:Stop` -> stop and close files

Per-channel controls/status:

- `$(P):CHx:Store:Enable` -> include channel in next session
- `$(P):CHx:Store:Active` -> channel is currently writing
- `$(P):CHx:Store:Count` -> written acquisitions
- `$(P):CHx:Store:File` -> full output filename

Filename format:

```text
<Path>/<Basename>-<ChannelName>-<YYYYMMDDTHHMMSS>.csv
```

### 5. Stored CSV format

- Tek-compatible metadata block:
	- `Model,MSO58LP`
	- `Label,<channel>`
	- `Waveform Type,ANALOG`
	- `Horizontal Units,s`
	- `Sample Interval,<xinc>`
	- `Record Length,<npt>`
	- `Zero Index,<data_start-1>`
	- `Vertical Units,V`
	- `ANALOG_Thumbnail,`
	- `yOffset,<yzero>`
	- `yPosition,<yoff>`
- Then for each acquisition:
	- `Labels,<channel>`
	- `TIME,<channel>`
	- one row per sample: `<time_s>,<value_V>`

### 6. Save in the oscilloscope's native formats

The 5 Series `SAVe:WAVEform` command supports spreadsheet CSV and Tektronix
internal WFM files. Unlike `Store:*`, these files are created on the
oscilloscope filesystem:

- `$(P):ScopeSave:Source` -> `ALL` or `CH1` through `CH8`
- `$(P):ScopeSave:Format` -> `CSV` or `WFM`
- `$(P):ScopeSave:Destination` -> scope-side path/name; `.csv` or `.wfm` is
  added (or replaced) from the selected format
- `$(P):ScopeSave:NumAcq` -> number of consecutive waveform saves
- `$(P):ScopeSave:Interval` -> interval in seconds between save commands
- `$(P):ScopeSave` -> execute the save
- `$(P):ScopeSave:Status`, `$(P):ScopeSave:Count`, and
  `$(P):ScopeSave:File_RBV` -> progress and result

The source must be displayed on the scope. Relative destinations use the
scope's current `FILESystem:CWD`. When `ALL` is selected, the scope appends
the source name to each generated filename as described by the programmer
manual.

Each command uses an IOC-local timestamp and acquisition number:

```text
<Destination>_<acquisition-number>_<YYYYMMDDTHHMMSS>.<csv|wfm>
```

## Plot CSV Files in Python

Any standard CSV reader can be used. The file has a metadata preamble,
followed by a `TIME,<channel>` two-column table for each acquisition.

Requirements:

```bash
python3 -m pip install numpy matplotlib
```

Examples:

```bash
python3 testScript/plot_dat.py saved.csv
python3 testScript/plot_dat.py saved.csv --max 20 --mean
python3 testScript/plot_dat.py saved.csv --time
python3 testScript/plot_dat.py saved.csv --save out.png --no-show
```

## Key Files

- Driver: [TEKMSO58LPApp/src/drvTekMSO58LP.cpp](TEKMSO58LPApp/src/drvTekMSO58LP.cpp)
- Storage engine: [TEKMSO58LPApp/src/tekStoreWaveform.c](TEKMSO58LPApp/src/tekStoreWaveform.c)
- Storage registration: [TEKMSO58LPApp/src/tekStoreWaveform.dbd](TEKMSO58LPApp/src/tekStoreWaveform.dbd)
- Device template: [db/device_asyn.template](db/device_asyn.template)
- Channel template: [db/channel_asyn.template](db/channel_asyn.template)
- Main OPI: [opi/mso_asyn.bob](opi/mso_asyn.bob)
- Channel OPI: [opi/channel_asyn_display.bob](opi/channel_asyn_display.bob)

## Troubleshooting

- If traces look clipped or offset, verify `DataStart` and channel enable state.
- If storage does not start, check `Store:Path` write permission and per-channel
	`Store:Enable` flags.
- If `Store:Count` stops early, verify IOC is still running and not interrupted.
