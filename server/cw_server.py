#!/usr/bin/env python3
# coding: utf-8
"""
cw_server.py - web front end for the CosmicWatch observation scheduler.

Usage:  python3 cw_server.py [port]          (default port 8080)
then open  http://<raspberry-pi-address>:8080/  in any browser.

Work flow in the browser
  1. Enter name, surname and initials -> a new data directory
     <DATA_ROOT>/<YYYYMMDD_HHMM>_<INITIALS> is created.
  2. Add observations (angle, time, file name) one by one or paste several
     plan lines; rows can be deleted. Every entry is checked with the same
     rules as cw_scheduler.py (angle -90..90, time 10 s..24 h with h/m/s,
     file names letters/digits/_-.+, unique).
  3. "Start plan": the plan is written to plan.txt in the data directory,
     checked again, and executed in the background exactly as by
     cw_scheduler.py (motor_control.py, then cw_reader, per line).
     Screen output of all programs goes to plan_<date>.log in the directory.
  4. Progress page: one row per observation, green = done, yellow = running,
     red = failed/stopped. It reloads itself only every REFRESH_RUNNING s
     (10 min) - the "Refresh" button shows the current state at once.
     Finished data files, plan and log are download links.
  5. At the end (completed or stopped) everything is packed into
     <DATA_ROOT>/<directory>.tar.gz, offered for download.
  "Stop plan" ends the current step cleanly (motor switched off, data file
  closed) and skips the rest. "New session" returns to step 1.

There is one plan at a time (one detector set-up); every browser sees the
same state. No passwords - meant for the lab network only.

The server uses only the Python standard library. The plan is run in a
thread of this process; the data taking itself is done by cw_reader in its
own process (real-time priority), so serving pages does not disturb it.
"""

import datetime
import html
import importlib.util
import json
import os
import re
import signal
import sys
import tarfile
import threading
import time
import traceback
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

#------------------------------------------------------------------------
# Configuration (environment variables override the defaults)

HERE = os.path.dirname(os.path.abspath(__file__))
PORT = int(os.environ.get("CW_SERVER_PORT", "8080"))
DATA_ROOT = os.path.expanduser(os.environ.get("CW_DATA_ROOT", "~/cw_data"))
REFRESH_RUNNING = int(os.environ.get("CW_REFRESH", "600"))     # s, page reload while running
# cw_scheduler.py: next to this file, or in ../scheduler, or given explicitly
SCHEDULER_CANDIDATES = [os.environ.get("CW_SCHEDULER", ""),
                        os.path.join(HERE, "cw_scheduler.py"),
                        os.path.join(HERE, "..", "scheduler", "cw_scheduler.py")]
INITIALS_RE = re.compile(r"^[A-Za-z]{2,4}$")
MAX_STEPS = 200
RESERVED_NAMES = {"plan.txt", "info.txt"}          # plus plan_*.log

#------------------------------------------------------------------------
# Load cw_scheduler.py as a module (its SETUP paths are relative to its own file)

def load_scheduler():
    for path in SCHEDULER_CANDIDATES:
        if path and os.path.isfile(path):
            spec = importlib.util.spec_from_file_location("cw_scheduler", path)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            return mod, os.path.abspath(path)
    sys.exit("cw_scheduler.py not found (put it next to cw_server.py or set CW_SCHEDULER)")

S, SCHEDULER_PATH = load_scheduler()

#------------------------------------------------------------------------
# Session state (one at a time), protected by LOCK

LOCK = threading.Lock()
STATE = {
    "phase": "idle",        # idle -> editing -> running -> finished
    "name": "", "surname": "", "initials": "",
    "dir": "",              # data directory (absolute)
    "steps": [],            # dicts: angle, seconds, time_txt, name (+ run results)
    "message": "",          # last error/info message for the page
    "msg_ok": True,
    "t_start": None, "t_end": None, "current": -1,
    "stop": False, "result": "", "tar": "", "log": "",
    "setup_warnings": [], "setup_errors": [],
}

def msg(text, ok=False):
    STATE["message"], STATE["msg_ok"] = text, ok

#------------------------------------------------------------------------
# Checking of one observation (same rules as cw_scheduler.check_plan)

def check_entry(a_txt, t_txt, name, existing):
    errors = []
    a_txt, t_txt, name = a_txt.strip(), t_txt.strip(), name.strip()
    angle = sec = None
    if not S.ANGLE_RE.match(a_txt):
        errors.append(f"angle '{a_txt}' is not a number (degrees without unit, e.g. 30 or -45.5)")
    else:
        angle = float(a_txt)
        if not S.ANGLE_MIN <= angle <= S.ANGLE_MAX:
            errors.append(f"angle {a_txt} outside {S.ANGLE_MIN:g} .. {S.ANGLE_MAX:g} deg")
    sec = S.parse_time(t_txt)
    if sec is None:
        errors.append(f"time '{t_txt}' has wrong format (number with unit h, m or s, e.g. 45s, 20m, 1.5h, 1h30m)")
    elif not S.TIME_MIN <= sec <= S.TIME_MAX:
        errors.append(f"time '{t_txt}' outside {S.TIME_MIN:g} s .. {S.TIME_MAX / 3600:g} h")
    if not name:
        errors.append("file name missing")
    elif not S.NAME_RE.match(name):
        errors.append(f"file name '{name}': only letters, digits, _ - . + allowed, "
                      f"must start with a letter or digit, no spaces")
    elif len(name) > S.MAX_NAME_LEN:
        errors.append(f"file name '{name}' too long (max {S.MAX_NAME_LEN})")
    elif name in RESERVED_NAMES or (name.startswith("plan_") and name.endswith(".log")):
        errors.append(f"file name '{name}' is reserved, choose another one")
    elif name in existing:
        errors.append(f"file name '{name}' already used in this plan")
    elif STATE["dir"] and os.path.lexists(os.path.join(STATE["dir"], name)):
        errors.append(f"file '{name}' already exists in the data directory")
    if errors:
        return None, errors
    return {"angle": angle, "seconds": sec, "time_txt": t_txt, "name": name}, []

#------------------------------------------------------------------------
# Plan execution (background thread)

def write_plan(path, steps):
    with open(path, "w") as f:
        f.write(f"# Observation plan - {STATE['name']} {STATE['surname']} ({STATE['initials']}), "
                f"created {datetime.datetime.now():%Y-%m-%d %H:%M} by cw_server.py\n")
        f.write("# angle   time    file\n")
        for st in steps:
            f.write(f"  {st['angle']:g}  {st['time_txt']}  {st['name']}\n")

def make_tar():
    d = STATE["dir"]
    tar_path = d.rstrip("/") + ".tar.gz"
    with tarfile.open(tar_path, "w:gz") as tar:
        tar.add(d, arcname=os.path.basename(d))
    return tar_path

def run_plan():
    """Runs the plan like cw_scheduler.main() step 4, but driven by STATE."""
    d = STATE["dir"]
    setup, _ = S.resolve_setup()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join(d, f"plan_{stamp}.log")
    with LOCK:
        STATE["log"] = os.path.basename(log_path)
    S.out.open_log(log_path)
    out = S.out
    steps = STATE["steps"]
    total = S.estimate(steps)
    out(f"Plan {os.path.join(d, 'plan.txt')} started {time.strftime('%Y-%m-%d %H:%M:%S')} "
        f"from the web server, log {log_path}")
    out(f"User: {STATE['name']} {STATE['surname']} ({STATE['initials']})")
    out(f"Expected total time {S.fmt_time(total)}")
    for w in setup["warnings"]:
        out(f"WARNING: {w}")
    t_start = time.time()
    interrupted = False
    try:
        for i, st in enumerate(steps):
            with LOCK:
                if STATE["stop"]:
                    interrupted = True
                    break
                STATE["current"] = i
                st["state"] = "moving"
                st["t0"] = time.time()
            out()
            out(f"--- observation {i + 1}/{len(steps)} (plan line {st['line']}): angle {st['angle']:g} deg, "
                f"time {S.fmt_time(st['seconds'])}, file {st['name']}")
            out(f"positioning to {st['angle']:g} deg", stamp=True)
            go, intr = S.do_position(st, setup)
            if STATE["stop"]:
                st["status"] = "stopped by user during positioning"
                interrupted = True
            if not go or interrupted:
                st["state"] = "failed"
                out(f"STOP: {st['status']}", stamp=True)
                break
            with LOCK:
                st["state"] = "reading"
                st["t_read"] = time.time()
            out(f"position {st['reached']:.2f} deg; data taking for {S.fmt_time(st['seconds'])}", stamp=True)
            go, intr = S.do_readout(st, setup)
            if STATE["stop"]:
                st["status"] = "stopped by user during readout (data file kept)"
                interrupted = True
            if not go or interrupted:
                st["state"] = "failed"
                out(f"STOP: {st['status']}", stamp=True)
                break
            st["state"] = "done"
            out(f"observation {i + 1} done: {st.get('events', '?')} events", stamp=True)
    except Exception:
        out("ERROR in the server while running the plan:\n" + traceback.format_exc())
    t_end = time.time()
    S.summary(steps, t_start, t_end, interrupted)
    out(f"Log written to {log_path}")
    S.out.close()
    done = sum(1 for st in steps if st.get("state") == "done")
    with LOCK:
        STATE["t_end"] = t_end
        STATE["current"] = -1
        STATE["result"] = ("completed" if done == len(steps)
                           else "stopped by user" if interrupted else "stopped because of an error")
    try:
        tar_path = make_tar()
        with LOCK:
            STATE["tar"] = os.path.basename(tar_path)
    except Exception as e:
        with LOCK:
            msg(f"could not create the tar file: {e}")
    with LOCK:
        STATE["phase"] = "finished"

def stop_plan():
    """Ctrl-C equivalent for the running child program (cw_reader / motor_control)."""
    STATE["stop"] = True
    proc = S.CURRENT.get("proc")
    if proc is not None and proc.poll() is None:
        try:
            proc.send_signal(signal.SIGINT)
        except OSError:
            pass

#------------------------------------------------------------------------
# HTML pages

CSS = """
body{font-family:system-ui,sans-serif;margin:1.5em auto;max-width:60em;padding:0 1em;color:#222}
h1{font-size:1.4em} h2{font-size:1.1em;margin-top:1.5em}
table{border-collapse:collapse;width:100%} th,td{border:1px solid #bbb;padding:.3em .5em;text-align:left}
th{background:#eee} td.num{text-align:right}
tr.done td{background:#c8f0c8} tr.run td{background:#fff3b0} tr.fail td{background:#f7c4c4}
.msg{padding:.5em;border-radius:4px;margin:.5em 0} .err{background:#f7c4c4} .ok{background:#c8f0c8}
.warn{background:#fff3b0;padding:.5em;border-radius:4px;margin:.5em 0}
input[type=text]{padding:.25em} button{padding:.3em .9em;margin:.2em}
.small{font-size:.85em;color:#555} textarea{width:100%;font-family:monospace}
form.inline{display:inline}
"""

def page(title, body, refresh=None):
    meta = f'<meta http-equiv="refresh" content="{refresh}">' if refresh else ""
    return (f"<!DOCTYPE html><html><head><meta charset='utf-8'>"
            f"<meta name='viewport' content='width=device-width, initial-scale=1'>{meta}"
            f"<title>{html.escape(title)}</title><style>{CSS}</style></head>"
            f"<body><h1>CosmicWatch – {html.escape(title)}</h1>{body}</body></html>")

def message_html():
    m = STATE["message"]
    if not m:
        return ""
    STATE["message"] = ""
    cls = "ok" if STATE["msg_ok"] else "err"
    return f"<div class='msg {cls}'>{html.escape(m).replace(chr(10), '<br>')}</div>"

def user_line():
    return (f"<p>User: <b>{html.escape(STATE['name'])} {html.escape(STATE['surname'])}</b> "
            f"({html.escape(STATE['initials'])}) &nbsp; Data directory: "
            f"<code>{html.escape(STATE['dir'])}</code></p>")

def page_idle():
    body = message_html() + """
<p>Start a new measurement session. Your data will be stored in a new directory named
after the date and your initials.</p>
<form method="post" action="/session">
<table style="width:auto">
<tr><td>Name</td><td><input type="text" name="name" required maxlength="40"></td></tr>
<tr><td>Surname</td><td><input type="text" name="surname" required maxlength="60"></td></tr>
<tr><td>Initials</td><td><input type="text" name="initials" required maxlength="4" size="5"
 pattern="[A-Za-z]{2,4}" title="2-4 letters A-Z"> <span class="small">2–4 letters A–Z (used in the directory name)</span></td></tr>
</table>
<button type="submit">Start session</button>
</form>"""
    return page("new session", body)

def plan_table(editing):
    steps = STATE["steps"]
    if not steps:
        return "<p><i>The plan is empty.</i></p>"
    rows = []
    for i, st in enumerate(steps):
        cls = {"done": "done", "moving": "run", "reading": "run", "failed": "fail"}.get(st.get("state"), "")
        name = html.escape(st["name"])
        if st.get("state") in ("done", "failed") and os.path.exists(os.path.join(STATE["dir"], st["name"])):
            name = f"<a href='/download/{urllib.parse.quote(st['name'])}'>{name}</a>"
        if editing:
            extra = (f"<td><form class='inline' method='post' action='/delete'>"
                     f"<input type='hidden' name='i' value='{i}'><button>delete</button></form></td>")
        else:
            state = st.get("state", "")
            status = {"moving": "positioning…", "reading": "taking data…", "": "waiting"}.get(state, "")
            if state in ("done", "failed"):
                status = st.get("status", "")
                if st.get("notes"):
                    status += " (" + "; ".join(st["notes"]) + ")"
            if state == "reading":
                left = st["seconds"] - (time.time() - st["t_read"])
                status += f" about {S.fmt_time(max(left, 0))} left"
            reached = f"{st['reached']:.2f}" if st.get("reached") is not None else ""
            meas = S.fmt_time(st["read_time"]) if "read_time" in st else ""
            ev = str(st["events"]) if "events" in st else ""
            extra = (f"<td class='num'>{reached}</td><td class='num'>{meas}</td>"
                     f"<td class='num'>{ev}</td><td>{html.escape(status)}</td>")
        rows.append(f"<tr class='{cls}'><td class='num'>{i + 1}</td><td class='num'>{st['angle']:g}</td>"
                    f"<td class='num'>{S.fmt_time(st['seconds'])}</td><td>{name}</td>{extra}</tr>")
    head = ("<th></th>" if editing else
            "<th>reached [deg]</th><th>measured</th><th>events</th><th>status</th>")
    return (f"<table><tr><th>#</th><th>angle [deg]</th><th>time</th><th>file</th>{head}</tr>"
            + "".join(rows) + "</table>")

def page_editing():
    steps = STATE["steps"]
    est = ""
    if steps:
        total = S.estimate([dict(st) for st in steps])
        data = sum(st["seconds"] for st in steps)
        est = (f"<p>Data taking time <b>{S.fmt_time(data)}</b>, expected total time "
               f"<b>{S.fmt_time(total)}</b> (incl. positioning).</p>")
    warn = "".join(f"<div class='warn'>WARNING: {html.escape(w)}</div>" for w in STATE["setup_warnings"])
    errs = "".join(f"<div class='msg err'>SETUP ERROR: {html.escape(e)}</div>" for e in STATE["setup_errors"])
    start_btn = ("" if not steps or STATE["setup_errors"] else
                 "<form method='post' action='/run' onsubmit=\"return confirm('Start the plan now?')\">"
                 "<button type='submit'><b>Start plan</b></button></form>")
    body = (message_html() + user_line() + "<h2>Observation plan</h2>" + plan_table(True) + est + """
<h2>Add observation</h2>
<form method="post" action="/add">
angle <input type="text" name="angle" size="6" required placeholder="-90..90"> deg &nbsp;
time <input type="text" name="time" size="7" required placeholder="e.g. 2h"> &nbsp;
file <input type="text" name="file" size="24" required placeholder="e.g. run_p30.dat">
<button type="submit">Add</button>
</form>
<p class="small">Time: number with unit h, m or s (e.g. 45s, 20m, 1.5h, 1h30m), 10 s .. 24 h.
File names: letters, digits, _ - . + only, no spaces.</p>
<details><summary>Paste several plan lines</summary>
<form method="post" action="/addmany">
<textarea name="lines" rows="6" placeholder="0   2h   run_000.dat&#10;30  90m  run_p30.dat"></textarea>
<button type="submit">Add all lines</button>
</form></details>
<h2>Run</h2>""" + errs + warn + start_btn + """
<form method="post" action="/new" onsubmit="return confirm('Discard this plan and start a new session?')">
<button type="submit">Cancel session</button></form>""")
    return page("observation plan", body)

def page_running():
    st_cur = STATE["steps"][STATE["current"]] if STATE["current"] >= 0 else None
    now = time.time()
    done = sum(1 for st in STATE["steps"] if st.get("state") == "done")
    total_left = sum(st["seconds"] for st in STATE["steps"] if st.get("state") in (None, "", "moving"))
    if st_cur is not None and st_cur.get("state") == "reading":
        total_left += max(0, st_cur["seconds"] - (now - st_cur["t_read"]))
    body = (message_html() + user_line()
            + f"<p>Plan running since {datetime.datetime.fromtimestamp(STATE['t_start']):%H:%M:%S} – "
              f"{done} of {len(STATE['steps'])} observations done, about {S.fmt_time(total_left)} "
              f"of data taking left.</p>"
            + f"<p class='small'>This page reloads itself every {REFRESH_RUNNING // 60} min "
              f"(last update {datetime.datetime.now():%H:%M:%S}).</p>"
            + "<form method='get' action='/'><button>Refresh now</button></form>"
            + plan_table(False)
            + (f"<p>Log: <a href='/download/{urllib.parse.quote(STATE['log'])}'>{html.escape(STATE['log'])}</a></p>"
               if STATE["log"] else "")
            + ("<p><b>Stopping…</b></p>" if STATE["stop"] else
               "<form method='post' action='/stop' onsubmit=\"return confirm('Stop the running plan? "
               "The current observation is ended and the rest is skipped.')\">"
               "<button>Stop plan</button></form>"))
    return page("plan running", body, refresh=REFRESH_RUNNING)

def page_finished():
    result = STATE["result"]
    cls = "ok" if result == "completed" else "err"
    tar = (f"<p>All data in one file: <a href='/download/{urllib.parse.quote(STATE['tar'])}'>"
           f"<b>{html.escape(STATE['tar'])}</b></a></p>" if STATE["tar"] else "")
    files = " &nbsp; ".join(f"<a href='/download/{urllib.parse.quote(n)}'>{html.escape(n)}</a>"
                            for n in ("plan.txt", STATE["log"]) if n)
    body = (message_html() + user_line()
            + f"<div class='msg {cls}'>Plan {html.escape(result)} at "
              f"{datetime.datetime.fromtimestamp(STATE['t_end']):%Y-%m-%d %H:%M:%S}.</div>"
            + plan_table(False) + tar + f"<p>Plan and log: {files}</p>"
            + "<form method='post' action='/new'><button>New session</button></form>")
    return page("plan finished", body)

#------------------------------------------------------------------------
# HTTP handler

class Handler(BaseHTTPRequestHandler):
    server_version = "cw_server/1.0"

    def log_message(self, fmt, *args):       # quiet: one short line per request to stderr
        sys.stderr.write(f"{time.strftime('%H:%M:%S')} {self.client_address[0]} {fmt % args}\n")

    def send_html(self, text, code=200):
        data = text.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def redirect(self, where="/"):
        self.send_response(303)
        self.send_header("Location", where)
        self.end_headers()

    def do_GET(self):
        path = urllib.parse.urlparse(self.path).path
        if path == "/":
            with LOCK:
                ph = STATE["phase"]
                text = {"idle": page_idle, "editing": page_editing,
                        "running": page_running, "finished": page_finished}[ph]()
            return self.send_html(text)
        if path.startswith("/download/"):
            return self.download(urllib.parse.unquote(path[len("/download/"):]))
        if path == "/status.json":            # for scripts / debugging
            with LOCK:
                s = {k: STATE[k] for k in ("phase", "name", "surname", "initials", "dir",
                                           "current", "result", "tar", "log")}
                s["steps"] = [{k: v for k, v in st.items() if k in
                               ("angle", "seconds", "name", "state", "status", "reached",
                                "read_time", "events", "notes")} for st in STATE["steps"]]
            data = json.dumps(s, indent=1).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            return self.wfile.write(data)
        self.send_html(page("not found", "<p>Page not found. <a href='/'>Back</a></p>"), 404)

    def download(self, name):
        with LOCK:
            d, tar = STATE["dir"], STATE["tar"]
            allowed = {st["name"] for st in STATE["steps"]} | {"plan.txt", STATE["log"]}
        if not d or not name or "/" in name or name.startswith("."):
            return self.send_html(page("not found", "<p>No such file.</p>"), 404)
        if name == tar and tar:
            path = os.path.join(os.path.dirname(d), tar)
        elif name in allowed:
            path = os.path.join(d, name)
        else:
            return self.send_html(page("not found", "<p>No such file.</p>"), 404)
        if not os.path.isfile(path):
            return self.send_html(page("not found", "<p>File not (yet) available.</p>"), 404)
        size = os.path.getsize(path)
        self.send_response(200)
        ctype = "application/gzip" if name.endswith(".gz") else "text/plain; charset=utf-8"
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(size))
        self.send_header("Content-Disposition", f'attachment; filename="{name}"')
        self.end_headers()
        with open(path, "rb") as f:
            while True:
                chunk = f.read(1 << 16)
                if not chunk:
                    break
                self.wfile.write(chunk)

    def form(self):
        n = int(self.headers.get("Content-Length", "0") or 0)
        raw = self.rfile.read(min(n, 1 << 20)).decode("utf-8", "replace")
        return {k: v[0] for k, v in urllib.parse.parse_qs(raw, keep_blank_values=True).items()}

    def do_POST(self):
        path = urllib.parse.urlparse(self.path).path
        f = self.form()
        with LOCK:
            ph = STATE["phase"]
            if path == "/session" and ph == "idle":
                self.new_session(f)
            elif path == "/add" and ph == "editing":
                self.add_lines([(f.get("angle", ""), f.get("time", ""), f.get("file", ""))])
            elif path == "/addmany" and ph == "editing":
                entries, bad = [], []
                for k, line in enumerate(f.get("lines", "").splitlines(), 1):
                    text = line.split("#", 1)[0].strip()
                    if not text:
                        continue
                    parts = text.split()
                    if len(parts) != 3:
                        bad.append(f"line {k}: expected 3 fields (angle time file): '{text}'")
                    else:
                        entries.append(tuple(parts))
                if bad:
                    msg("Nothing added:\n" + "\n".join(bad))
                else:
                    self.add_lines(entries)
            elif path == "/delete" and ph == "editing":
                try:
                    del STATE["steps"][int(f.get("i", "-1"))]
                except (ValueError, IndexError):
                    pass
            elif path == "/run" and ph == "editing":
                self.start_run()
            elif path == "/stop" and ph == "running":
                stop_plan()
                msg("Stopping: the current observation is being ended cleanly.", ok=True)
            elif path == "/new" and ph in ("editing", "finished"):
                self.reset()
            # anything else (e.g. double click, stale page): just show the current state
        self.redirect("/")

    # --- actions (called with LOCK held)

    def new_session(self, f):
        name, surname = f.get("name", "").strip(), f.get("surname", "").strip()
        initials = f.get("initials", "").strip().upper()
        errs = []
        if not name or not surname:
            errs.append("please give name and surname")
        if not INITIALS_RE.match(initials):
            errs.append("initials: 2-4 letters A-Z")
        if errs:
            return msg("; ".join(errs))
        d = os.path.join(DATA_ROOT, f"{datetime.datetime.now():%Y%m%d_%H%M}_{initials}")
        k = 1
        while os.path.exists(d) or os.path.exists(d + ".tar.gz"):
            k += 1
            d = os.path.join(DATA_ROOT, f"{datetime.datetime.now():%Y%m%d_%H%M}_{initials}_{k}")
        try:
            os.makedirs(d)
            with open(os.path.join(d, "info.txt"), "w") as fi:
                fi.write(f"name: {name}\nsurname: {surname}\ninitials: {initials}\n"
                         f"created: {datetime.datetime.now():%Y-%m-%d %H:%M:%S}\n")
        except OSError as e:
            return msg(f"cannot create the data directory {d}: {e}")
        setup, setup_errors = S.resolve_setup()
        STATE.update(phase="editing", name=name, surname=surname, initials=initials, dir=d,
                     steps=[], stop=False, result="", tar="", log="", current=-1,
                     setup_warnings=[w.replace("\n", " ") for w in setup["warnings"]],
                     setup_errors=setup_errors)

    def add_lines(self, entries):
        names = {st["name"] for st in STATE["steps"]}
        new, errs = [], []
        for a, t, n in entries:
            st, e = check_entry(a, t, n, names)
            if e:
                errs += [f"'{a} {t} {n}': {x}" for x in e]
            else:
                new.append(st)
                names.add(st["name"])
        if errs:
            return msg("Nothing added:\n" + "\n".join(errs))
        if len(STATE["steps"]) + len(new) > MAX_STEPS:
            return msg(f"too many observations (max {MAX_STEPS})")
        STATE["steps"] += new

    def start_run(self):
        d = STATE["dir"]
        plan = os.path.join(d, "plan.txt")
        write_plan(plan, STATE["steps"])
        steps, errors = S.check_plan(plan)            # final check, exactly as the command line
        setup, setup_errors = S.resolve_setup()
        if errors or setup_errors:
            return msg("Plan not started:\n" + "\n".join(errors + setup_errors))
        for st in steps:                              # check_plan steps carry line/path
            st["notes"] = []
        STATE.update(steps=steps, phase="running", t_start=time.time(), stop=False, current=-1)
        threading.Thread(target=run_plan, name="plan", daemon=True).start()

    def reset(self):
        d = STATE["dir"]
        if STATE["phase"] == "editing" and d and os.path.isdir(d) and set(os.listdir(d)) <= {"info.txt", "plan.txt"}:
            for n in os.listdir(d):                    # cancelled before any data: remove the empty directory
                os.remove(os.path.join(d, n))
            os.rmdir(d)
        STATE.update(phase="idle", name="", surname="", initials="", dir="", steps=[],
                     t_start=None, t_end=None, current=-1, stop=False, result="", tar="", log="")

#------------------------------------------------------------------------

def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else PORT
    os.makedirs(DATA_ROOT, exist_ok=True)
    srv = ThreadingHTTPServer(("", port), Handler)
    srv.daemon_threads = True
    print(f"cw_server: http://<this-host>:{port}/   data in {DATA_ROOT}   scheduler {SCHEDULER_PATH}",
          flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if STATE["phase"] == "running":
            print("server stopped while a plan was running - stopping the plan", flush=True)
            stop_plan()
            for _ in range(30):
                if STATE["phase"] != "running":
                    break
                time.sleep(1)
        srv.server_close()

if __name__ == "__main__":
    main()
