#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, sys, signal, curses, threading, math, os
from datetime import datetime

# ---- Optional: Offline startup check ----
def offline_check():
    now = datetime.utcnow()
    if not os.path.exists("de421.bsp"):
        print("[WARN] de421.bsp not found — Skyfield features (catalog & tracking) will be disabled.")
        print("        To fix: copy de421.bsp into this directory before running offline.")
    if now.year < 2020:
        print("[WARN] System clock appears incorrect (UTC year < 2020).")
        print("        Tracking and catalog features need an accurate clock.")
        print('        Example: sudo date -s "2025-10-10 16:30:00"')
    else:
        print(f"[INFO] System UTC time OK: {now.isoformat(timespec='seconds')}")
    print("-------------------------------------------------------------")

offline_check()

# ---- Optional (Hip/Catalog) deps guarded ----
HAVE_PANDAS = HAVE_SKYFIELD = False
try:
    import pandas as pd
    HAVE_PANDAS = True
except Exception:
    HAVE_PANDAS = False

try:
    from skyfield.api import load, wgs84, Star
    HAVE_SKYFIELD = True
except Exception:
    HAVE_SKYFIELD = False

# ===================== USER CONFIG =====================

# Motors (GPIO BCM)
A_PIN_A1, A_PIN_A2, A_PIN_B1, A_PIN_B2 = 17, 27, 22, 10   # Tilt
B_PIN_A1, B_PIN_A2, B_PIN_B1, B_PIN_B2 = 5, 6, 13, 19    # Pan

STEPS_PER_REV   = 200
STEPPING_MODE   = 'half'       # 'full' or 'half'
ACTIVE_LOW      = False
GEAR_RATIO      = 120.0        # motor revs per OUTPUT rev (worm)

RPM_STEP        = 1
MAX_RPM         = 200
UI_HZ           = 20

# Encoders
PAN_PIN_A, PAN_PIN_B   = 14, 15
TILT_PIN_A, TILT_PIN_B = 20, 21
PAN_PPR, TILT_PPR = 900, 900
PAN_CPR, TILT_CPR = PAN_PPR*4, TILT_PPR*4   # 3600 => 0.1° / count

# Limits
PAN_MIN_DEG, PAN_MAX_DEG   = -110.0, +110.0
TILT_MIN_DEG, TILT_MAX_DEG = -10.0, +90.0

SOFT_ZONE_DEG     = 4.0
HARD_MARGIN_DEG   = 0.3
SOFT_DECEL_RPMS   = 50.0

# Direction mapping
PAN_CMD_TO_ANGLE_SIGN  = -1
TILT_CMD_TO_ANGLE_SIGN = -1

# Display precision (UI only)
DECIMALS = 1
EPS      = 10**(-DECIMALS)

# LCD (optional)
USE_LCD = True
I2C_ADDR = 0x27
LCD_COLS, LCD_ROWS = 20, 4
UPDATE_INTERVAL_LCD = 0.10
DEG_SYM = "\xDF"
RPM_DECIMALS = 3

# Ramps
A_RAMP_RPMS = 50.0
B_RAMP_RPMS = 50.0

# GoTo controller
ROUND_DEG           = 0.05          # << you can go finer now (see startup banner)
GOTO_KP_RPM_PER_DEG = 5.0
GOTO_MIN_RPM        = 10.0
GOTO_MAX_RPM        = 180.0
GOTO_TOL_DEG        = ROUND_DEG
GOTO_ACCEL_RPMS     = 5.0
GOTO_START_RPM      = 1.0
GOTO_DIR_LATCH      = True

# Tracking controller
TRACK_KP_RPM_PER_DEG = 0.5
TRACK_MIN_RPM        = 0.01
TRACK_MAX_RPM        = 2.0
TRACK_TOL_DEG        = 0.01

# Site/time
LAT_DEG  = 34.0
LON_DEG  = -118.0
ELEV_M   = 250.0

TRACK_FACTORS = {
    "stars": 1.0,
    "sun":   0.9972695663,
    "moon":  (1.0 - 13.176358/360.0),
    "planet": 1.0,
}
SIDEREAL_DEG_PER_SEC = 360.0 / (23*3600 + 56*60 + 4.0905)

# Catalog files (created if missing)
HIP_MAIN_PATH   = "./hip_main.csv"
HIP_BRIGHT_PATH = "./hip_bright.csv"
DSO_PATH        = "./dso_catalog.csv"

# Visible list
MIN_LIST_ALT_DEG = -90.0
LIST_PAGE_SIZE   = 12
SORT_BY          = "alt"

# ===== Fusion (encoder + stepper) =====
FUSION_ENABLE                 = True
FUSION_ENCODER_TRUST_BAND_DEG = 0.20   # gentle bias when within this band
FUSION_CORRECT_GAIN           = 0.10   # 10% of error blended each UI tick
FUSION_HARD_RESET_DEG         = 0.75   # snap to encoder if exceeded

# ==========================================

# ---- Skyfield cache ----
_EPH=_EARTH=_TS=_OBSERVER=None
def _get_earth():
    global _EPH,_EARTH
    if _EARTH is None:
        _EPH = load('de421.bsp')
        _EARTH = _EPH['earth']
    return _EARTH

def _get_ts_observer():
    global _TS,_OBSERVER
    if _TS is None:
        _TS = load.timescale()
    if _OBSERVER is None:
        earth = _get_earth()
        _OBSERVER = earth + wgs84.latlon(LAT_DEG, LON_DEG, elevation_m=ELEV_M)
    return _TS,_OBSERVER

# ---- GPIO / devices ----
try:
    import RPi.GPIO as GPIO
except Exception as e:
    print("Error importing RPi.GPIO:", e); sys.exit(1)

from gpiozero import RotaryEncoder, Device
try:
    from gpiozero.pins.lgpio import LGPIOFactory
    Device.pin_factory = LGPIOFactory()
except Exception as e:
    print("Warning: lgpio backend not available; gpiozero may fallback.", e)

try:
    from RPLCD.i2c import CharLCD
except Exception:
    USE_LCD = False

# =======================================================
# Utility math & angle helpers
SEQ_FULL = [(1,0,1,0),(0,1,1,0),(0,1,0,1),(1,0,0,1)]
SEQ_HALF = [(1,0,0,0),(1,0,1,0),(0,0,1,0),(0,1,1,0),
            (0,1,0,0),(0,1,0,1),(0,0,0,1),(1,0,0,1)]

def _sign(x: float) -> int: return (x > 0) - (x < 0)
def d2r(d): return d * math.pi / 180.0
def r2d(r): return r * 180.0 / math.pi
def sind(d): return math.sin(d2r(d))
def cosd(d): return math.cos(d2r(d))
def asind(x): return r2d(math.asin(max(-1.0, min(1.0, x))))
def atan2d(y, x): return r2d(math.atan2(y, x))

def wrap_pm180(deg):
    x = ((deg + 180.0) % 360.0) - 180.0
    return 180.0 if x == -180.0 else x

def nearest_wrap(target_deg: float, current_deg: float) -> float:
    d = target_deg - current_deg
    while d > 180.0: d -= 360.0
    while d < -180.0: d += 360.0
    return current_deg + d

def pan_to_azN(pan_deg): return wrap_pm180(180.0 - pan_deg)
def azN_to_pan(azN_deg): return wrap_pm180(180.0 - azN_deg)

def wrap_0_360(deg):
    x = deg % 360.0
    return x if x >= 0 else x + 360.0

def equatorial_from_altaz(alt_deg, azN_deg, lat_deg):
    sinDec = sind(alt_deg) * sind(lat_deg) + cosd(alt_deg) * cosd(lat_deg) * cosd(azN_deg)
    dec = asind(sinDec)
    sinH = -sind(azN_deg) * cosd(alt_deg) / max(1e-9, cosd(dec))
    cosH = (sind(alt_deg) - sind(lat_deg) * sind(dec)) / max(1e-9, (cosd(lat_deg) * cosd(dec)))
    H = wrap_pm180(atan2d(sinH, cosH))
    return dec, H

def altaz_from_equatorial(dec_deg, H_deg, lat_deg):
    sinAlt = sind(lat_deg) * sind(dec_deg) + cosd(lat_deg) * cosd(dec_deg) * cosd(H_deg)
    alt = asind(sinAlt)
    sinAz = -sind(H_deg) * cosd(dec_deg) / max(1e-9, cosd(alt))
    cosAz = (sind(dec_deg) - sind(lat_deg) * sind(alt)) / max(1e-9, (cosd(lat_deg) * cosd(alt)))
    azN = wrap_pm180(atan2d(sinAz, cosAz))
    return alt, azN

def a_max_from_dec(dec_deg, lat_deg): return 90.0 - abs(lat_deg - dec_deg)

def _lcd_line(text: str, width: int) -> str: return (text[:width]).ljust(width)

def qround(x: float, q: float = None) -> float:
    if q is None: q = ROUND_DEG
    return round(x / q) * q

def set_targets_from_altaz(azN_deg: float, alt_deg: float, current_pan_deg: float):
    desired_pan = azN_to_pan(wrap_0_360(azN_deg))
    pan_target = nearest_wrap(desired_pan, current_pan_deg)
    return pan_target, alt_deg

# =======================================================
# Embedded default CSVs (same as before, trimmed for brevity)
DEFAULT_HIP_BRIGHT = """HIP,RA_deg,Dec_deg,Vmag,CommonName
32349,101.287155,-16.716116,-1.46,Sirius
30438,95.987875,-52.695661,-0.74,Canopus
24436,78.634467,-8.201640,0.18,Rigel
27989,88.792939,7.407064,0.45,Betelgeuse
37279,114.825493,5.225000,0.40,Procyon
24608,79.172327,45.997991,0.08,Capella
69673,213.915300,19.182410,-0.05,Arcturus
91262,279.234734,38.783688,0.03,Vega
97649,297.695827,8.868322,0.77,Altair
11767,37.954560,89.264108,1.98,Polaris
102098,310.357980,45.280338,1.25,Deneb
"""
DEFAULT_DSO = """ID,Name,Type,RA_deg,Dec_deg,Mag
M31,Andromeda,Galaxy,10.684708,41.268750,3.4
M33,Triangulum,Galaxy,23.462042,30.659941,5.7
M51,Whirlpool,Galaxy,202.469575,47.195258,8.4
M81,Bode's Galaxy,Galaxy,148.888213,69.065294,6.9
M82,Cigar Galaxy,Galaxy,148.968458,69.679704,8.4
M104,Sombrero,Galaxy,189.997500,-11.623100,8.0
M42,Orion Nebula,Nebula,83.822083,-5.391111,4.0
M45,Pleiades,Open Cluster,56.75,24.1167,1.6
"""

def _ensure_file(path: str, content: str):
    if not os.path.exists(path):
        with open(path, "w", newline="") as f: f.write(content)

def ensure_catalog_files():
    _ensure_file(HIP_BRIGHT_PATH, DEFAULT_HIP_BRIGHT)
    _ensure_file(DSO_PATH, DEFAULT_DSO)

_LAST_CATALOG_MSG = ""
NAME_TO_HIP = {"sirius":32349,"vega":91262,"polaris":11767,"betelgeuse":27989,"rigel":24436,
               "deneb":102098,"altair":97649,"capella":24608,"procyon":37279,"canopus":30438}
_HIP_MAIN_DF=_HIP_BRIGHT_DF=_DSO_DF=None

def _load_df_csv(path: str, required: list, rename_map=None):
    if not HAVE_PANDAS: raise RuntimeError("pandas not installed")
    df = pd.read_csv(path)
    if rename_map: df = df.rename(columns=rename_map)
    lower = {c.lower(): c for c in df.columns}; remap = {}
    for need in required:
        if need in df.columns: continue
        if need.lower() in lower: remap[lower[need.lower()]] = need
    if remap: df = df.rename(columns=remap)
    missing = [c for c in required if c not in df.columns]
    if missing: raise RuntimeError(f"Missing columns in {path}: {missing}")
    return df

def _load_catalogs():
    global _HIP_MAIN_DF,_HIP_BRIGHT_DF,_DSO_DF
    ensure_catalog_files()
    if _HIP_BRIGHT_DF is None:
        _HIP_BRIGHT_DF = _load_df_csv(HIP_BRIGHT_PATH,["HIP","RA_deg","Dec_deg","Vmag","CommonName"]).set_index("HIP")
    if _DSO_DF is None:
        _DSO_DF = _load_df_csv(DSO_PATH,["ID","Name","Type","RA_deg","Dec_deg","Mag"])
    if os.path.exists(HIP_MAIN_PATH) and _HIP_MAIN_DF is None:
        try:
            _HIP_MAIN_DF = _load_df_csv(HIP_MAIN_PATH,["HIP","RA_deg","Dec_deg"]).set_index("HIP")
        except Exception: _HIP_MAIN_DF=None

def _lookup_hip_or_name(q: str):
    _load_catalogs(); ql = q.strip().lower()
    if ql in NAME_TO_HIP:
        hip = NAME_TO_HIP[ql]; row = _HIP_BRIGHT_DF.loc[hip]
        return dict(kind="HIP", label=row.get("CommonName", f"HIP {hip}") or f"HIP {hip}",
                    ra=float(row["RA_deg"]), dec=float(row["Dec_deg"]))
    if ql.isdigit():
        hip = int(ql)
        df = _HIP_MAIN_DF if (_HIP_MAIN_DF is not None and hip in _HIP_MAIN_DF.index) else _HIP_BRIGHT_DF
        if hip in df.index:
            row = df.loc[hip]
            return dict(kind="HIP", label=str(row.get("CommonName", f"HIP {hip}") or f"HIP {hip}"),
                        ra=float(row["RA_deg"]), dec=float(row["Dec_deg"]))
        raise KeyError(f"HIP {hip} not in catalogs.")
    for _, row in _DSO_DF.iterrows():
        if str(row["ID"]).strip().lower() == ql:
            return dict(kind="DSO", label=row["Name"], ra=float(row["RA_deg"]), dec=float(row["Dec_deg"]))
    m = _DSO_DF[_DSO_DF["Name"].str.lower().str.contains(ql, na=False)]
    if len(m)>0:
        row=m.iloc[0]; return dict(kind="DSO", label=row["Name"], ra=float(row["RA_deg"]), dec=float(row["Dec_deg"]))
    raise KeyError(f"Unrecognized query: {q}")

def _radec_to_altaz_now(ra_deg: float, dec_deg: float):
    if not HAVE_SKYFIELD: raise RuntimeError("skyfield not installed")
    star = Star(ra_hours=(ra_deg/15.0), dec_degrees=dec_deg)
    ts, observer = _get_ts_observer(); t = ts.now()
    app = observer.at(t).observe(star).apparent()
    alt, az, _ = app.altaz()
    return float(alt.degrees), float(az.degrees)

def _build_visible_list():
    _load_catalogs(); rows=[]; skipped=0; last_err=""
    for hip, r in _HIP_BRIGHT_DF.iterrows():
        try:
            alt, az = _radec_to_altaz_now(float(r["RA_deg"]), float(r["Dec_deg"]))
            if alt >= MIN_LIST_ALT_DEG:
                rows.append(dict(Kind="Star",ID=f"HIP {hip}",Name=str(r.get("CommonName") or f"HIP {hip}"),
                                 Type="Star",Alt=alt,Az=az,Mag=float(r.get("Vmag",99.0)),Key=str(hip)))
        except Exception as e: skipped+=1; last_err=f"Star HIP {hip} err: {e}"
    for _, r in _DSO_DF.iterrows():
        try:
            alt, az = _radec_to_altaz_now(float(r["RA_deg"]), float(r["Dec_deg"]))
            if alt >= MIN_LIST_ALT_DEG:
                rows.append(dict(Kind="DSO",ID=str(r["ID"]),Name=str(r["Name"]),Type=str(r["Type"]),
                                 Alt=alt,Az=az,Mag=float(r.get("Mag",99.0)),Key=str(r["ID"])))
        except Exception as e: skipped+=1; last_err=f"DSO '{r.get('Name','?')}' err: {e}"
    if not HAVE_PANDAS:
        rows.sort(key=(lambda x:(-x["Alt"],x["Mag"])) if SORT_BY!="mag" else (lambda x:(x["Mag"],-x["Alt"])))
        return rows, f"Catalog: listed {len(rows)}, skipped {skipped}" + (f" (last err: {last_err})" if last_err else "")
    import pandas as pd
    if len(rows)==0:
        return [], f"Catalog: listed 0, skipped {skipped}" + (f" (last err: {last_err})" if last_err else "")
    df=pd.DataFrame(rows)
    df = df.sort_values(by=["Alt","Mag"], ascending=[False,True]) if SORT_BY!="mag" else df.sort_values(by=["Mag","Alt"], ascending=[True,False])
    rows2=df.to_dict(orient="records"); msg=f"Catalog: listed {len(rows2)}, skipped {skipped}" + (f" (last err: {last_err})" if last_err else "")
    return rows2,msg

# =======================================================
# Motors / Encoders / Fusion

class Stepper:
    def __init__(self, pins, steps_per_rev=200, stepping='full', active_low=False):
        self.pins=pins; self.steps_per_rev=steps_per_rev; self.active_low=active_low
        self.set_mode(stepping); self.idx=-1; self.hold_enabled=False; self._setup_gpio()
    def _setup_gpio(self):
        GPIO.setwarnings(False); GPIO.setmode(GPIO.BCM)
        for p in self.pins: GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)
    def set_mode(self, stepping):
        stepping=stepping.lower()
        if stepping not in ('full','half'): raise ValueError("stepping must be 'full' or 'half'")
        self.stepping=stepping; self.sequence = SEQ_HALF if stepping=='half' else SEQ_FULL
        self.steps_per_cycle = len(self.sequence)
    def _logic_write(self, pin, asserted: bool):
        try:
            if GPIO.getmode() is None: return
            level = GPIO.HIGH if asserted else GPIO.LOW
            if self.active_low: level = GPIO.LOW if asserted else GPIO.HIGH
            GPIO.output(pin, level)
        except RuntimeError: pass
    def _write_phase(self, phase):
        for pin, bit in zip(self.pins, phase): self._logic_write(pin, bool(bit))
    def step_no_sleep(self, direction: int):
        self.idx = (self.idx + direction) % self.steps_per_cycle
        self._write_phase(self.sequence[self.idx])
    def release(self):
        if not self.hold_enabled:
            for p in self.pins:
                try:
                    if GPIO.getmode() is None: return
                    self._logic_write(p, False)
                except RuntimeError: return
    def cleanup(self): self.release()
    def delay_from_rpm(self, rpm: float):
        microsteps_per_rev = self.steps_per_rev * (2 if self.stepping=='half' else 1)
        sps = (rpm * microsteps_per_rev) / 60.0
        return (1.0 / sps) if sps>0 else None

class StepperWorker:
    def __init__(self, stepper: Stepper, name="StepperWorker", step_cb=None):
        self.s=stepper; self.name=name; self._lock=threading.Lock()
        self._dir=0; self._delay=None; self._stop=False; self._cb=step_cb
        self._t=threading.Thread(target=self._run, daemon=True, name=name)
    def start(self): self._t.start()
    def stop(self):
        self.set_speed(0,0.0); self._stop=True; self._t.join()
    def set_speed(self, direction: int, rpm: float):
        with self._lock:
            self._dir=direction; self._delay=self.s.delay_from_rpm(rpm)
    def _run(self):
        last=time.monotonic()
        while not self._stop:
            with self._lock: d=self._dir; delay=self._delay
            if delay is None or d==0: time.sleep(0.001); continue
            now=time.monotonic(); delta=now-last
            if delta>=delay:
                self.s.step_no_sleep(d)
                if self._cb: 
                    try: self._cb(d)  # report every microstep
                    except Exception: pass
                last += delay
                if (now-last)>0.1: last=now
            else:
                time.sleep(max(0.0, (delay-delta)*0.6))

def wrap_angle_signed(deg: float) -> float:
    x = ((deg + 180.0) % 360.0) - 180.0
    return 180.0 if x == -180.0 else x

def signed_delta(curr: int, prev: int, period: int) -> int:
    d=curr-prev; half=period//2
    if d>half: d -= period
    if d<-half: d += period
    return d

class Axis:
    def __init__(self, name: str, pin_a: int, pin_b: int, cpr: int):
        self.name=name; self.cpr=cpr
        try:
            self.enc = RotaryEncoder(a=pin_a, b=pin_b, max_steps=cpr, wrap=True)
        except TypeError:
            self.enc = RotaryEncoder(a=pin_a, b=pin_b, max_steps=cpr)
        self.zero=self.enc.steps; self.last_steps=self.enc.steps
    def relative_steps(self) -> int:
        s=self.enc.steps; return signed_delta(s, self.zero, self.cpr)
    def angle_deg(self) -> float:
        steps_rel=self.relative_steps(); rev=steps_rel/self.cpr; ang=wrap_angle_signed(rev*360.0)
        a_val = round(ang, DECIMALS)
        d=signed_delta(self.enc.steps, self.last_steps, self.cpr)
        if abs(a_val)<EPS and d!=0: a_val = EPS if d>0 else -EPS
        if abs(a_val + 180.0) < EPS: a_val = 180.0
        self.last_steps=self.enc.steps
        return a_val

# ---------- Fusion estimator ----------
class FusedAngle:
    def __init__(self, axis: Axis, step_deg: float, cmd_to_angle_sign: int):
        self.axis=axis
        self.step_deg = float(step_deg)           # degrees at OUTPUT per microstep
        self.dir_to_angle = float(cmd_to_angle_sign)  # +1 or -1, map motor direction to +angle
        self.motor_est = self._read_encoder_raw() # initialize at encoder angle
        self._lock = threading.Lock()

    def _read_encoder_raw(self) -> float:
        # Use unrounded, as Axis.angle_deg() rounds for UI. Recompute here from steps.
        s_rel = self.axis.relative_steps()
        ang = wrap_angle_signed((s_rel/self.axis.cpr)*360.0)
        return ang

    def on_motor_step(self, step_dir: int):
        if not FUSION_ENABLE or step_dir==0: return
        with self._lock:
            self.motor_est = wrap_pm180(self.motor_est + (self.step_deg * self.dir_to_angle * step_dir))

    def update(self):
        """Return fused angle (deg) and a 'trusted' boolean (True if inside soft band)."""
        enc = self._read_encoder_raw()
        with self._lock:
            diff = wrap_pm180(enc - self.motor_est)
            trusted = abs(diff) <= FUSION_ENCODER_TRUST_BAND_DEG
            if abs(diff) > FUSION_HARD_RESET_DEG:
                # hard reset
                self.motor_est = enc
            else:
                # gentle pull toward encoder within band (and even a little outside)
                self.motor_est = wrap_pm180(self.motor_est + FUSION_CORRECT_GAIN * diff)
            fused = self.motor_est
        return fused, trusted

# Slew helper
def slew(current: float, target: float, max_rate: float, dt: float) -> float:
    return min(target, current + max_rate*dt) if target>current else max(target, current - max_rate*dt)

# =======================================================
# UI helpers

def _prompt(stdscr, y, x, prompt, maxlen=24):
    curses.echo(); stdscr.nodelay(False)
    stdscr.addstr(y, x, prompt); stdscr.clrtoeol()
    s = stdscr.getstr(y, x + len(prompt), maxlen).decode("utf-8").strip()
    curses.noecho(); stdscr.nodelay(True)
    return s

def visible_picker(stdscr, pan_angle_fn):
    global _LAST_CATALOG_MSG
    try:
        items, msg = _build_visible_list()
    except Exception as e:
        _LAST_CATALOG_MSG = f"Catalog error: {e}"; return None
    if msg: _LAST_CATALOG_MSG = msg
    if not items: return None
    page=0
    while True:
        stdscr.erase()
        stdscr.addstr(0,0,f"Visible now (Alt ≥ {MIN_LIST_ALT_DEG:.0f}°) — page {page+1}/{((len(items)-1)//LIST_PAGE_SIZE)+1}")
        stdscr.addstr(1,0,"Enter number to GoTo, 'n' next, 'p' prev, 'q' cancel, or type HIP/ID/name and press Enter")
        top=3; start=page*LIST_PAGE_SIZE; chunk=items[start:start+LIST_PAGE_SIZE]
        for i,it in enumerate(chunk, start=1):
            line=f"[{i:2d}] {it['Name']:<18.18} {it['Type']:<14.14} {it['ID']:<8.8} Alt {it['Alt']:5.1f}°  Az {wrap_0_360(it['Az']):5.1f}°"
            stdscr.addstr(top+i-1,0,line)
        stdscr.addstr(top+LIST_PAGE_SIZE+1,0,"Choice: "); stdscr.refresh()
        curses.echo(); stdscr.nodelay(False)
        try: s=stdscr.getstr(top+LIST_PAGE_SIZE+1,len("Choice: "),24).decode("utf-8").strip()
        except Exception: s=""
        finally: curses.noecho(); stdscr.nodelay(True)
        if s.lower()=='q': return None
        if s.lower()=='n':
            if start+LIST_PAGE_SIZE<len(items): page+=1
            continue
        if s.lower()=='p':
            if page>0: page-=1
            continue
        if s.isdigit():
            idx=int(s)
            if 1<=idx<=len(chunk):
                it=chunk[idx-1]
                try:
                    alt, azN = it["Alt"], it["Az"]
                    pan_tgt, tilt_tgt = set_targets_from_altaz(azN, alt, pan_angle_fn())
                    pan_tgt=qround(pan_tgt); tilt_tgt=qround(tilt_tgt)
                    _LAST_CATALOG_MSG=f"GoTo: {it['Name']} ({it['ID']})"
                    return pan_tgt, tilt_tgt
                except Exception as e:
                    _LAST_CATALOG_MSG=f"GoTo error: {e}"; return None
        try:
            target=_lookup_hip_or_name(s)
            alt, azN = _radec_to_altaz_now(target["ra"], target["dec"])
            pan_tgt, tilt_tgt = set_targets_from_altaz(azN, alt, pan_angle_fn())
            pan_tgt=qround(pan_tgt); tilt_tgt=qround(tilt_tgt)
            _LAST_CATALOG_MSG=f"GoTo: {target['label']}"
            return pan_tgt, tilt_tgt
        except Exception as e:
            _LAST_CATALOG_MSG=f"Lookup error: {e}"
            # loop

# =======================================================
# Main

pan_angle_fn=None

def run(stdscr):
    global pan_angle_fn, _LAST_CATALOG_MSG, _OBSERVER
    curses.curs_set(0); stdscr.nodelay(True); stdscr.keypad(True)

    # Mechanics-derived step angle at OUTPUT
    micro_mult = 2 if STEPPING_MODE.lower()=='half' else 1
    microsteps_per_motor_rev = STEPS_PER_REV * micro_mult
    microsteps_per_output_rev = microsteps_per_motor_rev * GEAR_RATIO
    STEP_DEG = 360.0 / float(microsteps_per_output_rev)
    enc_res_pan = 360.0 / PAN_CPR
    enc_res_tilt = 360.0 / TILT_CPR
    rec_min_round = max(3.0*STEP_DEG, 0.02)

    print(f"[FUSION] output step angle: {STEP_DEG:.5f}° per motor microstep; encoder res (pan/tilt): {enc_res_pan:.3f}°/{enc_res_tilt:.3f}°")
    print(f"[FUSION] suggested minimum ROUND_DEG ≈ {rec_min_round:.3f}° (current {ROUND_DEG:.3f}°)")

    quit_requested=False
    def _sigint(*_):
        nonlocal quit_requested
        quit_requested=True
    signal.signal(signal.SIGINT, _sigint)

    # Steppers
    A = Stepper((A_PIN_A1,A_PIN_A2,A_PIN_B1,A_PIN_B2), steps_per_rev=STEPS_PER_REV, stepping=STEPPING_MODE, active_low=ACTIVE_LOW)
    B = Stepper((B_PIN_A1,B_PIN_A2,B_PIN_B1,B_PIN_B2), steps_per_rev=STEPS_PER_REV, stepping=STEPPING_MODE, active_low=ACTIVE_LOW)

    # Encoders
    pan_axis  = Axis("Pan",  PAN_PIN_A, PAN_PIN_B, PAN_CPR)
    tilt_axis = Axis("Tilt", TILT_PIN_A, TILT_PIN_B, TILT_CPR)

    # Fusion
    if FUSION_ENABLE:
        pan_fused  = FusedAngle(pan_axis,  STEP_DEG, PAN_CMD_TO_ANGLE_SIGN)
        tilt_fused = FusedAngle(tilt_axis, STEP_DEG, TILT_CMD_TO_ANGLE_SIGN)
        def a_tick(dir_): tilt_fused.on_motor_step(dir_)
        def b_tick(dir_): pan_fused.on_motor_step(dir_)
        Aw = StepperWorker(A, "A_worker", step_cb=a_tick)
        Bw = StepperWorker(B, "B_worker", step_cb=b_tick)
        # expose helpers
        def fused_pan_angle():  ang,_ = pan_fused.update();  return ang
        pan_angle_fn = fused_pan_angle
        def fused_tilt_angle(): ang,_ = tilt_fused.update(); return ang
    else:
        pan_fused=tilt_fused=None
        Aw = StepperWorker(A, "A_worker")
        Bw = StepperWorker(B, "B_worker")
        pan_angle_fn = pan_axis.angle_deg
        def fused_tilt_angle(): return tilt_axis.angle_deg()

    Aw.start(); Bw.start()

    # LCD
    lcd=None
    if USE_LCD:
        try:
            lcd=CharLCD(i2c_expander='PCF8574', address=I2C_ADDR, port=1, cols=LCD_COLS, rows=LCD_ROWS, charmap='A00', auto_linebreaks=False)
            lcd.clear(); lcd.write_string(_lcd_line("Mount Ready", LCD_COLS))
        except Exception: lcd=None

    # State
    A_units=B_units=0; A_dir=B_dir=0; A_rpm_cmd=B_rpm_cmd=0.0; A_rpm=B_rpm=0.0
    mode="manual"; pan_target=0.0; tilt_target=0.0
    pan_goto_cap=tilt_goto_cap=0.0; pan_goto_dir=tilt_goto_dir=0
    track_active=False; track_dec_deg=0.0; track_H0_deg=0.0; track_t0=0.0
    track_kind="stars"; track_factor=TRACK_FACTORS["stars"]; track_a_max=None
    last_ui_time=0.0; ui_interval=1.0/float(UI_HZ); last_lcd_time=0.0; prev_lcd=("","","","")
    last_tick=time.monotonic()
    try:
        from zoneinfo import ZoneInfo
        LOCAL_TZ=ZoneInfo("America/Los_Angeles")
    except Exception:
        LOCAL_TZ=None
    if not os.path.exists(HIP_BRIGHT_PATH) or not os.path.exists(DSO_PATH):
        _LAST_CATALOG_MSG="Created default hip_bright.csv / dso_catalog.csv (first run)."
    if HAVE_SKYFIELD:
        _OBSERVER=None; _get_ts_observer()

    last_command=""

    try:
        while True:
            if quit_requested: break
            key = stdscr.getch()
            if key != -1:
                if key in (ord('q'), 27): break
                # Cancel GoTo/Track
                if key in (curses.KEY_DOWN,curses.KEY_UP,curses.KEY_LEFT,curses.KEY_RIGHT,ord(' '),ord('c'),ord('C')):
                    mode="manual"; pan_goto_cap=tilt_goto_cap=0.0; pan_goto_dir=tilt_goto_dir=0; track_active=False

                if key==curses.KEY_DOWN:   A_units -= 1
                elif key==curses.KEY_UP:   A_units += 1
                elif key==curses.KEY_LEFT: B_units -= 1
                elif key==curses.KEY_RIGHT:B_units += 1
                elif key==ord(' '):        A_units=B_units=0
                elif key==ord('1'):        A.hold_enabled = not A.hold_enabled
                elif key==ord('2'):        B.hold_enabled = not B.hold_enabled
                elif key==ord('r'):        A.hold_enabled=B.hold_enabled=False; A.release(); B.release()
                elif key==ord('p'):        pan_axis.zero = pan_axis.enc.steps
                elif key==ord('t'):        tilt_axis.zero = tilt_axis.enc.steps
                elif key==ord('z'):        pan_axis.zero = pan_axis.enc.steps; tilt_axis.zero = tilt_axis.enc.steps

                # Manual GoTo
                elif key in (ord('g'),ord('G')):
                    try:
                        az_str = _prompt(stdscr,12,0,"Enter target Az (0-360, N=0,E=90,S=180,W=270): ")
                        alt_str= _prompt(stdscr,13,0,"Enter target Alt (deg): ")
                        az_in=float(az_str); alt_in=float(alt_str)
                        az_in=qround(az_in%360.0); alt_in=qround(alt_in)
                        pan_target, tilt_target = set_targets_from_altaz(az_in,alt_in, pan_angle_fn())
                        pan_target=qround(pan_target); tilt_target=qround(tilt_target)
                        mode="goto"; last_command="goto"
                        A_units=B_units=0; pan_goto_cap=tilt_goto_cap=0.0; pan_goto_dir=tilt_goto_dir=0; track_active=False
                    except Exception: mode="manual"

                # Visible (Shift+K)
                elif key==ord('K'):
                    res=visible_picker(stdscr, pan_angle_fn)
                    if res:
                        pan_target,tilt_target=res
                        mode="goto"; last_command="goto"
                        A_units=B_units=0; pan_goto_cap=tilt_goto_cap=0.0; pan_goto_dir=tilt_goto_dir=0; track_active=False

                # Home
                elif key in (ord('h'),ord('H')):
                    pan_target,tilt_target = set_targets_from_altaz(180.0,0.0, pan_angle_fn())
                    pan_target=qround(pan_target); tilt_target=qround(tilt_target)
                    mode="goto"; last_command="home"
                    A_units=B_units=0; pan_goto_cap=tilt_goto_cap=0.0; pan_goto_dir=tilt_goto_dir=0; track_active=False

                # Track toggles
                elif key==ord('T'):
                    curr_pan,_  = (pan_fused.update() if FUSION_ENABLE else (pan_axis.angle_deg(),True))
                    curr_tilt,_ = (tilt_fused.update() if FUSION_ENABLE else (tilt_axis.angle_deg(),True))
                    pan_target=qround(curr_pan); tilt_target=qround(curr_tilt)
                    alt_now=tilt_target; azN_now=pan_to_azN(pan_target)
                    dec_now,H_now = equatorial_from_altaz(alt_now, azN_now, LAT_DEG)
                    track_dec_deg=dec_now; track_H0_deg=H_now
                    track_t0=time.monotonic(); track_kind="stars"; track_factor=TRACK_FACTORS["stars"]
                    track_a_max=a_max_from_dec(track_dec_deg, LAT_DEG)
                    pan_goto_cap=tilt_goto_cap=0.0; pan_goto_dir=tilt_goto_dir=0
                    mode="track"; last_command="track"; A_units=B_units=0; track_active=True

                elif key in (ord('U'),ord('M'),ord('P')):
                    kind={ord('U'):'sun', ord('M'):'moon', ord('P'):'planet'}[key]
                    curr_pan,_  = (pan_fused.update() if FUSION_ENABLE else (pan_axis.angle_deg(),True))
                    curr_tilt,_ = (tilt_fused.update() if FUSION_ENABLE else (tilt_axis.angle_deg(),True))
                    pan_target=qround(curr_pan); tilt_target=qround(curr_tilt)
                    alt_now=tilt_target; azN_now=pan_to_azN(pan_target)
                    dec_now,H_now = equatorial_from_altaz(alt_now, azN_now, LAT_DEG)
                    track_dec_deg=dec_now; track_H0_deg=H_now
                    track_t0=time.monotonic(); track_kind=kind; track_factor=TRACK_FACTORS[kind]
                    track_a_max=a_max_from_dec(track_dec_deg, LAT_DEG)
                    pan_goto_cap=tilt_goto_cap=0.0; pan_goto_dir=tilt_goto_dir=0
                    mode="track"; last_command="track"; A_units=B_units=0; track_active=True

                elif key==ord('['):
                    if mode=="track": track_factor=max(0.90, track_factor-0.002)
                elif key==ord(']'):
                    if mode=="track": track_factor=min(1.10, track_factor+0.002)

            # ----- Time step -----
            now=time.monotonic(); dt=max(0.0, now-last_tick); last_tick=now

            # ----- Read angles (fused or raw) -----
            if FUSION_ENABLE:
                pan_deg,_  = pan_fused.update()
                tilt_deg,_ = tilt_fused.update()
            else:
                pan_deg = pan_axis.angle_deg()
                tilt_deg = tilt_axis.angle_deg()

            azN_now = pan_to_azN(pan_deg); alt_now=tilt_deg

            # ----- TRACK update -----
            if mode=="track" and track_active:
                dts=now-track_t0
                H = wrap_pm180(track_H0_deg + (SIDEREAL_DEG_PER_SEC * track_factor) * dts)
                alt_t, azN_t = altaz_from_equatorial(track_dec_deg, H, LAT_DEG)
                tilt_target = alt_t
                raw_pan_target = azN_to_pan(azN_t)
                pan_target = nearest_wrap(raw_pan_target, pan_deg)

            # ----- Commands -----
            max_units=max(1, MAX_RPM//RPM_STEP)
            A_units=max(-max_units, min(max_units, A_units))
            B_units=max(-max_units, min(max_units, B_units))

            if mode=="manual":
                if A_units==0:
                    A_dir=0; A_rpm_cmd=0.0
                    if not A.hold_enabled: A.release()
                else:
                    A_rpm_cmd=min(MAX_RPM, abs(A_units)*RPM_STEP)
                    A_dir=_sign(A_units) * TILT_CMD_TO_ANGLE_SIGN
                if B_units==0:
                    B_dir=0; B_rpm_cmd=0.0
                    if not B.hold_enabled: B.release()
                else:
                    B_rpm_cmd=min(MAX_RPM, abs(B_units)*RPM_STEP)
                    B_dir=_sign(B_units)

            elif mode=="goto":
                pan_err  = wrap_pm180(pan_target - pan_deg)
                tilt_err = (tilt_target - tilt_deg)
                if abs(tilt_err)<=GOTO_TOL_DEG:
                    A_dir=0; A_rpm_cmd=0.0; tilt_goto_cap=0.0; tilt_goto_dir=0
                else:
                    desired_angle_dir=_sign(tilt_err)
                    tilt_goto_dir = (desired_angle_dir*TILT_CMD_TO_ANGLE_SIGN) if (tilt_goto_dir==0 or not GOTO_DIR_LATCH) else tilt_goto_dir
                    tilt_goto_cap=min(GOTO_MAX_RPM, tilt_goto_cap + GOTO_ACCEL_RPMS*dt)
                    tilt_req=max(GOTO_START_RPM, abs(tilt_err)*GOTO_KP_RPM_PER_DEG)
                    A_rpm_cmd=max(GOTO_MIN_RPM, min(tilt_req, tilt_goto_cap, GOTO_MAX_RPM))
                    A_dir=tilt_goto_dir
                if abs(pan_err)<=GOTO_TOL_DEG:
                    B_dir=0; B_rpm_cmd=0.0; pan_goto_cap=0.0; pan_goto_dir=0
                else:
                    desired_angle_dir=_sign(pan_err)
                    pan_goto_dir = (desired_angle_dir*PAN_CMD_TO_ANGLE_SIGN) if (pan_goto_dir==0 or not GOTO_DIR_LATCH) else pan_goto_dir
                    pan_goto_cap=min(GOTO_MAX_RPM, pan_goto_cap + GOTO_ACCEL_RPMS*dt)
                    pan_req=max(GOTO_START_RPM, abs(pan_err)*GOTO_KP_RPM_PER_DEG)
                    B_rpm_cmd=max(GOTO_MIN_RPM, min(pan_req, pan_goto_cap, GOTO_MAX_RPM))
                    B_dir=pan_goto_dir
                if (abs(tilt_err)<=GOTO_TOL_DEG) and (abs(pan_err)<=GOTO_TOL_DEG):
                    pan_target = pan_deg; tilt_target = tilt_deg
                    A_dir=B_dir=0; A_rpm_cmd=B_rpm_cmd=0.0
                    Aw.set_speed(0,0.0); Bw.set_speed(0,0.0)
                    mode="manual"; A_units=B_units=0
                    pan_goto_cap=tilt_goto_cap=0.0; pan_goto_dir=tilt_goto_dir=0

            else:  # track
                pan_err  = wrap_pm180(pan_target - pan_deg)
                tilt_err = tilt_target - tilt_deg
                if abs(tilt_err)<=TRACK_TOL_DEG: A_dir=0; A_rpm_cmd=0.0
                else:
                    desired_angle_dir=_sign(tilt_err)
                    A_dir=desired_angle_dir * TILT_CMD_TO_ANGLE_SIGN
                    tilt_req=abs(tilt_err)*TRACK_KP_RPM_PER_DEG
                    A_rpm_cmd=max(TRACK_MIN_RPM, min(tilt_req, TRACK_MAX_RPM))
                if abs(pan_err)<=TRACK_TOL_DEG: B_dir=0; B_rpm_cmd=0.0
                else:
                    desired_angle_dir=_sign(pan_err)
                    B_dir=desired_angle_dir * PAN_CMD_TO_ANGLE_SIGN
                    pan_req=abs(pan_err)*TRACK_KP_RPM_PER_DEG
                    B_rpm_cmd=max(TRACK_MIN_RPM, min(pan_req, TRACK_MAX_RPM))

            # Soft/hard stops
            tilt_intent=(1 if A_dir>0 else (-1 if A_dir<0 else 0)) * TILT_CMD_TO_ANGLE_SIGN
            a_target=A_rpm_cmd; a_rate=A_RAMP_RPMS
            if tilt_intent!=0 and a_target>0.0:
                dist=(TILT_MAX_DEG-tilt_deg) if tilt_intent>0 else (tilt_deg-TILT_MIN_DEG)
                if dist<=SOFT_ZONE_DEG:
                    dist=max(0.0, min(SOFT_ZONE_DEG,dist)); a_target*= (dist/SOFT_ZONE_DEG); a_rate=SOFT_DECEL_RPMS
            if tilt_intent>0 and tilt_deg>=(TILT_MAX_DEG-HARD_MARGIN_DEG): A_units=0; A_dir=0; a_target=0.0; A_rpm=0.0
            elif tilt_intent<0 and tilt_deg<=(TILT_MIN_DEG+HARD_MARGIN_DEG): A_units=0; A_dir=0; a_target=0.0; A_rpm=0.0

            pan_intent=(1 if B_dir>0 else (-1 if B_dir<0 else 0)) * PAN_CMD_TO_ANGLE_SIGN
            b_target=B_rpm_cmd; b_rate=B_RAMP_RPMS
            if pan_intent!=0 and b_target>0.0:
                dist_p=(PAN_MAX_DEG-pan_deg) if pan_intent>0 else (pan_deg-PAN_MIN_DEG)
                if dist_p<=SOFT_ZONE_DEG:
                    dist_p=max(0.0, min(SOFT_ZONE_DEG,dist_p)); b_target*=(dist_p/SOFT_ZONE_DEG); b_rate=SOFT_DECEL_RPMS
            if pan_intent>0 and pan_deg>=(PAN_MAX_DEG-HARD_MARGIN_DEG): B_units=0; B_dir=0; b_target=0.0; B_rpm=0.0
            elif pan_intent<0 and pan_deg<=(PAN_MIN_DEG+HARD_MARGIN_DEG): B_units=0; B_dir=0; b_target=0.0; B_rpm=0.0

            # Slew + Apply
            A_rpm = slew(A_rpm, a_target, a_rate, dt); B_rpm = slew(B_rpm, b_target, b_rate, dt)
            Aw.set_speed(A_dir, A_rpm); Bw.set_speed(B_dir, B_rpm)

            # UI
            if (now-last_ui_time)>=ui_interval:
                stdscr.erase()
                stdscr.addstr(0,0,"A: ↓ faster LEFT | ↑ slower/RIGHT   B: ← faster LEFT | → faster RIGHT")
                stdscr.addstr(1,0,"SPACE stop/cancel | 1 HOLD A | 2 HOLD B | r release | p zero PAN | t zero TILT | z zero")
                stdscr.addstr(2,0,"g GoTo | h Home | T/U/M/P track | [/] trim | K Visible (HIP/DSO)")
                stdscr.addstr(3,0,f"Mode={A.stepping}  ActiveLow={ACTIVE_LOW}  MAX_RPM={MAX_RPM}  UI={UI_HZ} Hz")
                lt = datetime.now().strftime("%Y-%m-%d %H:%M:%S") if not 'LOCAL_TZ' in locals() or LOCAL_TZ is None else datetime.now(LOCAL_TZ).strftime("%Y-%m-%d %H:%M:%S %Z")
                stdscr.addstr(4,0,f"Local time: {lt}")

                row0=6
                if mode=="goto":
                    stdscr.addstr(row0,0,f"MODE: GOTO  Targets  Pan={pan_target:.2f}°  Tilt={tilt_target:.2f}°   (SPACE/arrow/C to cancel)")
                elif mode=="track":
                    stdscr.addstr(row0,0,f"MODE: TRACK [{track_kind}]  factor={track_factor:.5f}  Amax={a_max_from_dec(track_dec_deg,LAT_DEG):5.1f}°  KP={TRACK_KP_RPM_PER_DEG:.2f}")
                else:
                    stdscr.addstr(row0,0,"MODE: MANUAL")

                row=row0+2
                A_dir_str = 'LEFT' if A_dir==-1 else ('RIGHT' if A_dir==1 else 'NEUTRAL')
                B_dir_str = 'LEFT' if B_dir==-1 else ('RIGHT' if B_dir==1 else 'NEUTRAL')
                stdscr.addstr(row+0,0,f"A/TILT: Dir={A_dir_str:7}  RPM={A_rpm:7.3f}/{A_rpm_cmd:7.3f}   Lim[{TILT_MIN_DEG:.1f},{TILT_MAX_DEG:.1f}]")
                stdscr.addstr(row+1,0,f"B/PAN : Dir={B_dir_str:7}  RPM={B_rpm:7.3f}/{B_rpm_cmd:7.3f}   Lim[{PAN_MIN_DEG:.1f},{PAN_MAX_DEG:.1f}]")

                pan_err_dbg=wrap_pm180(pan_target-pan_deg); tilt_err_dbg=(tilt_target-tilt_deg)
                stdscr.addstr(row+3,0,f"PAN : cur={pan_deg:7.{DECIMALS}f}°  tgt={pan_target:7.3f}°  err={pan_err_dbg:8.4f}°")
                stdscr.addstr(row+4,0,f"TILT: cur={tilt_deg:7.{DECIMALS}f}°  tgt={tilt_target:7.3f}°  err={tilt_err_dbg:8.4f}°")

                tgt_azN=pan_to_azN(pan_target); tgt_az360=wrap_0_360(tgt_azN)
                err_azN=wrap_pm180(tgt_azN - azN_now); err_alt=tilt_target - alt_now
                stdscr.addstr(row+6,0,f"Az : cur={wrap_0_360(azN_now):6.{DECIMALS}f}°  tgt={tgt_az360:6.1f}°  err={err_azN:8.4f}°  (N=0,E=90,S=180,W=270)")
                stdscr.addstr(row+7,0,f"Alt: cur={alt_now:6.{DECIMALS}f}°  tgt={tilt_target:6.1f}°  err={err_alt:8.4f}°")

                if _LAST_CATALOG_MSG: stdscr.addstr(row+9,0,_LAST_CATALOG_MSG[:LCD_COLS*2])
                stdscr.refresh(); last_ui_time=now

            # LCD
            if USE_LCD and lcd and (now-last_lcd_time)>=UPDATE_INTERVAL_LCD:
                alt_txt=f"Altitude: {alt_now:5.{DECIMALS}f}{DEG_SYM}"
                az_txt =f"Azimuth:  {wrap_0_360(azN_now):5.{DECIMALS}f}{DEG_SYM}"
                if mode=="manual":
                    mode_txt="Mode: Manual"; line4=""
                elif mode=="track":
                    mode_txt="Mode: Tracking"; line4=f"Amax {a_max_from_dec(track_dec_deg,LAT_DEG):5.{DECIMALS}f}{DEG_SYM}"
                else:
                    mode_txt="Mode: GoTo"; line4=f"Alt {tilt_target:5.{DECIMALS}f}{DEG_SYM}, Azm {wrap_0_360(pan_to_azN(pan_target)):5.{DECIMALS}f}{DEG_SYM}"
                l1=_lcd_line(alt_txt,LCD_COLS); l2=_lcd_line(az_txt,LCD_COLS); l3=_lcd_line(mode_txt,LCD_COLS); l4=_lcd_line(line4,LCD_COLS)
                p1,p2,p3,p4=prev_lcd
                if l1!=p1: lcd.cursor_pos=(0,0); lcd.write_string(l1)
                if l2!=p2: lcd.cursor_pos=(1,0); lcd.write_string(l2)
                if l3!=p3: lcd.cursor_pos=(2,0); lcd.write_string(l3)
                if l4!=p4: lcd.cursor_pos=(3,0); lcd.write_string(l4)
                prev_lcd=(l1,l2,l3,l4); last_lcd_time=now

            time.sleep(0)

    finally:
        try:
            Aw.set_speed(0,0.0); Bw.set_speed(0,0.0)
            Aw.stop(); Bw.stop()
            A.hold_enabled=False; B.hold_enabled=False
            A.release(); B.release()
            GPIO.cleanup()
            try:
                if USE_LCD and lcd: lcd.close(clear=False)
            except Exception: pass
        finally:
            curses.endwin()

def main():
    ensure_catalog_files()
    curses.wrapper(run)

if __name__=="__main__":
    main()
