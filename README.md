# Automated_Telescope_Tracking_Prototype

## Overview

Raspberry Pi telescope mount controller for Alt-Azimuth pointing and tracking.

Current implementation in `telescope_control.py` provides:

- manual motion control
- GoTo by Alt/Az entry
- offline star / DSO selection
- tracking modes
- encoder-based angle feedback
- optional LCD status output

The code is currently written to run with local astronomy files present in the same working directory as `telescope_control.py`.

## Repository Layout

```text
Automated_Telescope_Tracking_Prototype/
├── README.md
├── requirements.txt
├── .gitignore
├── FILE_PLACEMENT_GUIDE.txt
├── install_pi.sh
├── telescope_control.py
├── de421.bsp
├── hip_bright.csv
├── dso_catalog.csv
├── hip_main.dat
└── docs/
    └── architecture_manual.pdf
```

## Important File Placement Note

The current code checks for these files in the repository root:

- `de421.bsp`
- `hip_bright.csv`
- `dso_catalog.csv`

The current code also defines:

```python
HIP_MAIN_PATH = "./hip_main.csv"
```

but the uploaded catalog file is `hip_main.dat`.

That means:

- `hip_bright.csv` works now
- `dso_catalog.csv` works now
- `de421.bsp` works now
- `hip_main.dat` is included in the repository, but the current code will not automatically use it unless the path in `telescope_control.py` is changed from `./hip_main.csv` to `./hip_main.dat`

## Raspberry Pi Setup

Install system packages:

```bash
sudo apt update
sudo apt install -y python3-pip python3-rpi.gpio python3-gpiozero python3-smbus i2c-tools
```

Install Python packages:

```bash
pip3 install -r requirements.txt
```

## Run

From the repository root:

```bash
python3 telescope_control.py
```

## Upload Order

Upload these first:

1. `README.md`
2. `requirements.txt`
3. `.gitignore`
4. `FILE_PLACEMENT_GUIDE.txt`
5. `install_pi.sh`
6. `telescope_control.py`

Then upload the astronomy files to the repository root:

7. `de421.bsp`
8. `hip_bright.csv`
9. `dso_catalog.csv`
10. `hip_main.dat`

Then upload documentation:

11. `docs/architecture_manual.pdf`
