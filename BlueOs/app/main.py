import json
import os
import re
import subprocess
import threading
import time
from collections import deque
from copy import deepcopy

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, HTMLResponse
from pydantic import BaseModel

app = FastAPI()

EXTENSION_PREFIX = "/extensionv2/smartcontrolallocation"

DATA_DIR = "/data"
GENERATED_DIR = os.path.join(DATA_DIR, "generated")

CONFIG_PATH = os.path.join(DATA_DIR, "actuator_config.json")
GENERATED_HEADER_PATH = os.path.join(GENERATED_DIR, "actuator_specs.h")
GENERATED_SOURCE_PATH = os.path.join(GENERATED_DIR, "actuator_specs.c")
GENERATED_VEHICLE_SETTINGS_PATH = os.path.join(GENERATED_DIR, "generated_vehicle_settings.h")
CONTROL_RUNTIME_CFG_PATH = os.path.join(DATA_DIR, "control_runtime.cfg")

TORPEDO_BIN_DEFAULT = "/workspace/build/torpedo_main"
TORPEDO_CWD_DEFAULT = "/workspace"

ALLOWED_DOWNLOADS = {
    "actuator_specs.h": GENERATED_HEADER_PATH,
    "actuator_specs.c": GENERATED_SOURCE_PATH,
    "generated_vehicle_settings.h": GENERATED_VEHICLE_SETTINGS_PATH,
    "control_runtime.cfg": CONTROL_RUNTIME_CFG_PATH,
}

runtime_process = None
runtime_process_lock = threading.Lock()
runtime_logs = deque(maxlen=2000)


class ConfigPayload(BaseModel):
    config: dict


def ensure_dirs():
    os.makedirs(DATA_DIR, exist_ok=True)
    os.makedirs(GENERATED_DIR, exist_ok=True)


def default_config():
    return {
        "vehicle": {
            "water_density_kg_m3": 1025.0,
            "axis_weight_surge": 1.0,
            "axis_weight_roll": 1.5,
            "axis_weight_pitch": 1.5,
            "axis_weight_yaw": 2.0,
        },
        "pid": {
            "roll": {"kp": 1.0, "ki": 0.0, "kd": 0.0},
            "pitch": {"kp": 1.0, "ki": 0.0, "kd": 0.0},
            "yaw": {"kp": 1.0, "ki": 0.0, "kd": 0.0},
        },
        "runtime": {
            "desired_roll_deg": 0.0,
            "desired_pitch_deg": 0.0,
            "desired_yaw_deg": 0.0,
            "desired_surge_force_N": 20.0,
        },
        "actuators": [
            {
                "name": "roll_left_front",
                "kind": "fin",
                "enabled": True,
                "inverted": False,
                "channel": 8,
                "cmd_min": -30.0,
                "cmd_neutral": 0.0,
                "cmd_max": 30.0,
                "pwm_min": 1000,
                "pwm_max": 2000,
                "position": {"x": 0.00, "y": -0.20, "z": 0.00},
                "force_dir": {"x": 0.0, "y": 0.0, "z": 1.0},
                "area_m2": 0.0060,
            },
            {
                "name": "main_thruster",
                "kind": "thruster",
                "enabled": True,
                "inverted": False,
                "channel": 1,
                "cmd_min": 0.0,
                "cmd_neutral": 0.0,
                "cmd_max": 100.0,
                "pwm_min": 1000,
                "pwm_max": 2000,
                "position": {"x": -0.48, "y": 0.0, "z": 0.0},
                "force_dir": {"x": 1.0, "y": 0.0, "z": 0.0},
                "thrust_gain_pos_N_per_cmd": 0.45,
                "thrust_gain_neg_N_per_cmd": 0.0,
            },
        ],
    }


def deep_merge(defaults, current):
    if isinstance(defaults, dict) and isinstance(current, dict):
        result = deepcopy(defaults)
        for key, value in current.items():
            if key in result:
                result[key] = deep_merge(result[key], value)
            else:
                result[key] = deepcopy(value)
        return result
    return deepcopy(current)


def load_config():
    ensure_dirs()

    if not os.path.exists(CONFIG_PATH):
        cfg = default_config()
        save_config(cfg)
        return cfg

    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            raw = json.load(f)
        return deep_merge(default_config(), raw)
    except Exception:
        cfg = default_config()
        save_config(cfg)
        return cfg


def save_config(config):
    ensure_dirs()
    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        json.dump(config, f, indent=2)


def safe_name(name, fallback):
    cleaned = re.sub(r"[^a-zA-Z0-9_]+", "_", (name or "").strip().lower())
    cleaned = re.sub(r"_+", "_", cleaned).strip("_")
    if not cleaned:
        cleaned = fallback
    if cleaned[0].isdigit():
        cleaned = f"act_{cleaned}"
    return cleaned


def fmt_float(value, digits=6):
    try:
        v = float(value)
    except Exception:
        v = 0.0

    s = f"{v:.{digits}f}"
    s = s.rstrip("0").rstrip(".")
    if "." not in s:
        s += ".0"
    return s + "f"


def fmt_bool(value):
    return "true" if bool(value) else "false"


def get_number(data, key, default):
    try:
        return float(data.get(key, default))
    except Exception:
        return float(default)


def get_int(data, key, default):
    try:
        return int(float(data.get(key, default)))
    except Exception:
        return int(default)


def get_vec3(data, key, default):
    raw = data.get(key, {})
    if not isinstance(raw, dict):
        raw = {}
    return {
        "x": get_number(raw, "x", default["x"]),
        "y": get_number(raw, "y", default["y"]),
        "z": get_number(raw, "z", default["z"]),
    }


def normalize_actuator(actuator, index):
    kind = str(actuator.get("kind", "fin")).strip().lower()
    if kind not in ("fin", "thruster"):
        kind = "fin"

    if kind == "fin":
        defaults = {
            "enabled": True,
            "inverted": False,
            "channel": index,
            "cmd_min": -30.0,
            "cmd_neutral": 0.0,
            "cmd_max": 30.0,
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "force_dir": {"x": 0.0, "y": 0.0, "z": 1.0},
            "area_m2": 0.0035,
            "cl_alpha_per_rad": 3.80,
            "deflection_rad_per_cmd": 0.01745329252,
            "min_speed_mps": 0.20,
            "pwm_min": 1000,
            "pwm_max": 2000,
            "rate_limit_units_per_s": 360.0,
            "deadzone": 0.10,
            "efficiency": 1.0,
        }
        driver_type = "servo"
    else:
        defaults = {
            "enabled": True,
            "inverted": False,
            "channel": index,
            "cmd_min": 0.0,
            "cmd_neutral": 0.0,
            "cmd_max": 100.0,
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "force_dir": {"x": 1.0, "y": 0.0, "z": 0.0},
            "thrust_gain_pos_N_per_cmd": 0.45,
            "thrust_gain_neg_N_per_cmd": 0.0,
            "pwm_min": 1000,
            "pwm_max": 2000,
            "rate_limit_units_per_s": 250.0,
            "deadzone": 0.0,
            "efficiency": 1.0,
        }
        driver_type = "motor"

    merged = deep_merge(defaults, actuator)

    return {
        "name": actuator.get("name", f"{kind}_{index}"),
        "kind": kind,
        "driver_type": driver_type,
        "enabled": bool(merged.get("enabled", True)),
        "inverted": bool(merged.get("inverted", False)),
        "channel": get_int(merged, "channel", index),
        "cmd_min": get_number(merged, "cmd_min", defaults["cmd_min"]),
        "cmd_neutral": get_number(merged, "cmd_neutral", defaults["cmd_neutral"]),
        "cmd_max": get_number(merged, "cmd_max", defaults["cmd_max"]),
        "position": get_vec3(merged, "position", defaults["position"]),
        "force_dir": get_vec3(merged, "force_dir", defaults["force_dir"]),
        "pwm_min": get_int(merged, "pwm_min", defaults["pwm_min"]),
        "pwm_max": get_int(merged, "pwm_max", defaults["pwm_max"]),
        "rate_limit_units_per_s": get_number(
            merged, "rate_limit_units_per_s", defaults["rate_limit_units_per_s"]
        ),
        "deadzone": get_number(merged, "deadzone", defaults["deadzone"]),
        "efficiency": get_number(merged, "efficiency", defaults["efficiency"]),
        "area_m2": get_number(merged, "area_m2", defaults.get("area_m2", 0.0)),
        "cl_alpha_per_rad": get_number(
            merged, "cl_alpha_per_rad", defaults.get("cl_alpha_per_rad", 0.0)
        ),
        "deflection_rad_per_cmd": get_number(
            merged, "deflection_rad_per_cmd", defaults.get("deflection_rad_per_cmd", 0.0)
        ),
        "min_speed_mps": get_number(
            merged, "min_speed_mps", defaults.get("min_speed_mps", 0.0)
        ),
        "thrust_gain_pos_N_per_cmd": get_number(
            merged, "thrust_gain_pos_N_per_cmd", defaults.get("thrust_gain_pos_N_per_cmd", 0.0)
        ),
        "thrust_gain_neg_N_per_cmd": get_number(
            merged, "thrust_gain_neg_N_per_cmd", defaults.get("thrust_gain_neg_N_per_cmd", 0.0)
        ),
    }


def generate_header_text():
    return """#ifndef ACTUATOR_SPECS_H
#define ACTUATOR_SPECS_H

#include <stdbool.h>
#include <stdint.h>

#include "common_Control/actuator_model.h"

typedef struct
{
    const char *name;

    actuator_type_t type;
    actuator_driver_type_t driver_type;

    bool enabled;
    bool inverted;

    uint8_t channel;

    float cmd_min;
    float cmd_max;
    float cmd_neutral;

    uint16_t pwm_min;
    uint16_t pwm_max;

    vector3 position_m;
    vector3 force_dir_body;
    vector3 direct_moment_per_cmd_body;

    float rate_limit_units_per_s;
    float deadzone;
    float efficiency;

    union
    {
        struct
        {
            float area_m2;
            float cl_alpha_per_rad;
            float deflection_rad_per_cmd;
            float min_speed_mps;
        } fin;

        struct
        {
            float thrust_gain_pos_N_per_cmd;
            float thrust_gain_neg_N_per_cmd;
        } thruster;
    } model;
} actuator_spec_t;

extern const actuator_spec_t g_actuator_specs[];
extern const uint8_t g_actuator_specs_count;

#endif
"""


def actuator_to_c_block(actuator, index):
    a = normalize_actuator(actuator, index)
    name = a["name"].replace('"', '\\"')
    fallback_name = safe_name(a["name"], f"actuator_{index}")

    block = [
        "    {",
        f'        .name = "{name}",',
        f'        .type = {"ACTUATOR_FIN" if a["kind"] == "fin" else "ACTUATOR_THRUSTER"},',
        f'        .driver_type = {"ACTUATOR_DRIVER_SERVO" if a["kind"] == "fin" else "ACTUATOR_DRIVER_MOTOR"},',
        f"        .enabled = {fmt_bool(a['enabled'])},",
        f"        .inverted = {fmt_bool(a['inverted'])},",
        f"        .channel = {a['channel']},",
        "",
        f"        .cmd_min = {fmt_float(a['cmd_min'])},",
        f"        .cmd_max = {fmt_float(a['cmd_max'])},",
        f"        .cmd_neutral = {fmt_float(a['cmd_neutral'])},",
        "",
        f"        .pwm_min = {a['pwm_min']},",
        f"        .pwm_max = {a['pwm_max']},",
        "",
        f"        .position_m = {{ {fmt_float(a['position']['x'])}, {fmt_float(a['position']['y'])}, {fmt_float(a['position']['z'])} }},",
        f"        .force_dir_body = {{ {fmt_float(a['force_dir']['x'])}, {fmt_float(a['force_dir']['y'])}, {fmt_float(a['force_dir']['z'])} }},",
        "        .direct_moment_per_cmd_body = { 0.0f, 0.0f, 0.0f },",
        "",
        f"        .rate_limit_units_per_s = {fmt_float(a['rate_limit_units_per_s'])},",
        f"        .deadzone = {fmt_float(a['deadzone'])},",
        f"        .efficiency = {fmt_float(a['efficiency'])},",
        "",
        f"        /* generated id: {fallback_name} */",
    ]

    if a["kind"] == "fin":
        block.extend(
            [
                "        .model.fin = {",
                f"            .area_m2 = {fmt_float(a['area_m2'])},",
                f"            .cl_alpha_per_rad = {fmt_float(a['cl_alpha_per_rad'])},",
                f"            .deflection_rad_per_cmd = {fmt_float(a['deflection_rad_per_cmd'])},",
                f"            .min_speed_mps = {fmt_float(a['min_speed_mps'])}",
                "        }",
            ]
        )
    else:
        block.extend(
            [
                "        .model.thruster = {",
                f"            .thrust_gain_pos_N_per_cmd = {fmt_float(a['thrust_gain_pos_N_per_cmd'])},",
                f"            .thrust_gain_neg_N_per_cmd = {fmt_float(a['thrust_gain_neg_N_per_cmd'])}",
                "        }",
            ]
        )

    block.append("    }")
    return "\n".join(block)


def generate_source_text(config):
    actuators = config.get("actuators", [])
    if actuators:
        blocks = [actuator_to_c_block(a, i) for i, a in enumerate(actuators)]
        joined = ",\n\n".join(blocks)
    else:
        joined = ""

    return f'''#include "actuator_specs.h"

const actuator_spec_t g_actuator_specs[] =
{{
{joined}
}};

const uint8_t g_actuator_specs_count =
    (uint8_t)(sizeof(g_actuator_specs) / sizeof(g_actuator_specs[0]));
'''


def generate_vehicle_settings_text(config):
    vehicle = config.get("vehicle", {})

    water_density = get_number(vehicle, "water_density_kg_m3", 1025.0)
    surge = get_number(vehicle, "axis_weight_surge", 1.0)
    roll = get_number(vehicle, "axis_weight_roll", 1.5)
    pitch = get_number(vehicle, "axis_weight_pitch", 1.5)
    yaw = get_number(vehicle, "axis_weight_yaw", 2.0)

    return f"""#ifndef GENERATED_VEHICLE_SETTINGS_H
#define GENERATED_VEHICLE_SETTINGS_H

#define GENERATED_WATER_DENSITY_KG_M3 {fmt_float(water_density)}
#define GENERATED_AXIS_WEIGHT_SURGE   {fmt_float(surge)}
#define GENERATED_AXIS_WEIGHT_ROLL    {fmt_float(roll)}
#define GENERATED_AXIS_WEIGHT_PITCH   {fmt_float(pitch)}
#define GENERATED_AXIS_WEIGHT_YAW     {fmt_float(yaw)}

#endif
"""


def generate_runtime_cfg_text(config):
    pid = config.get("pid", {})
    runtime = config.get("runtime", {})

    roll = deep_merge({"kp": 1.0, "ki": 0.0, "kd": 0.0}, pid.get("roll", {}))
    pitch = deep_merge({"kp": 1.0, "ki": 0.0, "kd": 0.0}, pid.get("pitch", {}))
    yaw = deep_merge({"kp": 1.0, "ki": 0.0, "kd": 0.0}, pid.get("yaw", {}))

    desired_roll = get_number(runtime, "desired_roll_deg", 0.0)
    desired_pitch = get_number(runtime, "desired_pitch_deg", 0.0)
    desired_yaw = get_number(runtime, "desired_yaw_deg", 0.0)
    desired_surge = get_number(runtime, "desired_surge_force_N", 20.0)

    lines = [
        "# EXPO60 live control runtime config",
        f"roll_kp={roll['kp']}",
        f"roll_ki={roll['ki']}",
        f"roll_kd={roll['kd']}",
        f"pitch_kp={pitch['kp']}",
        f"pitch_ki={pitch['ki']}",
        f"pitch_kd={pitch['kd']}",
        f"yaw_kp={yaw['kp']}",
        f"yaw_ki={yaw['ki']}",
        f"yaw_kd={yaw['kd']}",
        f"desired_roll_deg={desired_roll}",
        f"desired_pitch_deg={desired_pitch}",
        f"desired_yaw_deg={desired_yaw}",
        f"desired_surge_force_N={desired_surge}",
        "",
    ]
    return "\n".join(lines)


def generate_files(config):
    ensure_dirs()

    with open(GENERATED_HEADER_PATH, "w", encoding="utf-8") as f:
        f.write(generate_header_text())

    with open(GENERATED_SOURCE_PATH, "w", encoding="utf-8") as f:
        f.write(generate_source_text(config))

    with open(GENERATED_VEHICLE_SETTINGS_PATH, "w", encoding="utf-8") as f:
        f.write(generate_vehicle_settings_text(config))

    with open(CONTROL_RUNTIME_CFG_PATH, "w", encoding="utf-8") as f:
        f.write(generate_runtime_cfg_text(config))


def append_log(line):
    runtime_logs.append(line.rstrip("\n"))


def runtime_bin_path():
    return os.environ.get("TORPEDO_BIN", TORPEDO_BIN_DEFAULT)


def runtime_cwd():
    return os.environ.get("TORPEDO_CWD", TORPEDO_CWD_DEFAULT)


def process_alive(proc):
    return (proc is not None) and (proc.poll() is None)


def runtime_status():
    with runtime_process_lock:
        proc = runtime_process
        return {
            "running": process_alive(proc),
            "pid": proc.pid if process_alive(proc) else None,
            "binary": runtime_bin_path(),
            "cwd": runtime_cwd(),
        }


def _reader_thread(proc):
    try:
        if proc.stdout is not None:
            for line in proc.stdout:
                append_log(line)
    finally:
        code = proc.poll()
        append_log(f"[runtime] process exited with code {code}")


def start_runtime_process():
    global runtime_process

    with runtime_process_lock:
        if process_alive(runtime_process):
            return runtime_status()

        bin_path = runtime_bin_path()
        cwd = runtime_cwd()

        if not os.path.exists(bin_path):
            raise HTTPException(
                status_code=400,
                detail=f"Binary not found: {bin_path}. Build torpedo_main first and mount /workspace."
            )

        runtime_logs.clear()
        append_log(f"[runtime] starting {bin_path}")

        env = os.environ.copy()
        env["CONTROL_RUNTIME_CONFIG_PATH"] = CONTROL_RUNTIME_CFG_PATH

        runtime_process = subprocess.Popen(
            [bin_path],
            cwd=cwd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            text=True,
            bufsize=1,
            env=env,
        )

        thread = threading.Thread(target=_reader_thread, args=(runtime_process,), daemon=True)
        thread.start()

        return runtime_status()


def stop_runtime_process():
    global runtime_process

    with runtime_process_lock:
        if not process_alive(runtime_process):
            runtime_process = None
            return {"running": False}

        append_log("[runtime] stopping process")
        runtime_process.terminate()

        try:
            runtime_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            append_log("[runtime] terminate timeout, killing")
            runtime_process.kill()
            runtime_process.wait(timeout=5)

        runtime_process = None
        return {"running": False}


def page_html():
    return """
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>Smart Control Allocation</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background: #0f172a;
            color: #e2e8f0;
            margin: 0;
            padding: 24px;
        }
        h1, h2, h3 {
            color: #38bdf8;
        }
        .panel {
            background: #1e293b;
            border-radius: 12px;
            padding: 16px;
            margin-bottom: 16px;
            border: 1px solid #334155;
        }
        .toolbar button, .card button, .downloads a {
            margin-right: 8px;
            margin-bottom: 8px;
            padding: 10px 14px;
            border: 0;
            border-radius: 8px;
            cursor: pointer;
            text-decoration: none;
            display: inline-block;
            background: #38bdf8;
            color: #082f49;
            font-weight: bold;
        }
        .secondary {
            background: #475569 !important;
            color: white !important;
        }
        .danger {
            background: #ef4444 !important;
            color: white !important;
        }
        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
            gap: 12px;
        }
        .card {
            background: #1e293b;
            border-radius: 12px;
            padding: 16px;
            margin-bottom: 16px;
            border: 1px solid #334155;
        }
        label {
            display: block;
            font-size: 13px;
            color: #cbd5e1;
            margin-bottom: 4px;
        }
        input, select {
            width: 100%;
            box-sizing: border-box;
            padding: 10px;
            border-radius: 8px;
            border: 1px solid #475569;
            background: #0f172a;
            color: white;
        }
        .status {
            margin-top: 10px;
            font-weight: bold;
        }
        .small {
            color: #94a3b8;
            font-size: 13px;
        }
        pre {
            background: #020617;
            color: #e2e8f0;
            padding: 16px;
            border-radius: 12px;
            min-height: 240px;
            overflow: auto;
            white-space: pre-wrap;
            border: 1px solid #334155;
        }
    </style>
</head>
<body>
    <h1>Smart Control Allocation</h1>

    <div class="panel toolbar">
        <button onclick="loadConfig()">Reload</button>
        <button onclick="saveConfig()">Save</button>
        <button onclick="generateFiles()">Generate C Files</button>
        <button class="secondary" onclick="addFin()">Add Fin</button>
        <button class="secondary" onclick="addThruster()">Add Thruster</button>
        <div id="status" class="status"></div>
    </div>

    <div class="panel">
        <h2>Runtime</h2>
        <button onclick="startRuntime()">Start Code</button>
        <button class="danger" onclick="stopRuntime()">Stop Code</button>
        <div id="runtime-status" class="status"></div>
        <p class="small">Requires the extension container to be started with <code>--network host</code> and <code>-v /home/pi/torpedo_EX60_Main:/workspace</code>.</p>
        <pre id="runtime-console"></pre>
    </div>

    <div class="panel">
        <h2>Vehicle</h2>
        <div class="grid">
            <div>
                <label>Water density kg/m³</label>
                <input id="water_density_kg_m3" type="number" step="0.1">
            </div>
            <div>
                <label>Axis weight surge</label>
                <input id="axis_weight_surge" type="number" step="0.1">
            </div>
            <div>
                <label>Axis weight roll</label>
                <input id="axis_weight_roll" type="number" step="0.1">
            </div>
            <div>
                <label>Axis weight pitch</label>
                <input id="axis_weight_pitch" type="number" step="0.1">
            </div>
            <div>
                <label>Axis weight yaw</label>
                <input id="axis_weight_yaw" type="number" step="0.1">
            </div>
        </div>
    </div>

    <div class="panel">
        <h2>Live PID gains</h2>
        <div class="grid">
            <div><label>Roll Kp</label><input id="roll_kp" type="number" step="0.01"></div>
            <div><label>Roll Ki</label><input id="roll_ki" type="number" step="0.01"></div>
            <div><label>Roll Kd</label><input id="roll_kd" type="number" step="0.01"></div>

            <div><label>Pitch Kp</label><input id="pitch_kp" type="number" step="0.01"></div>
            <div><label>Pitch Ki</label><input id="pitch_ki" type="number" step="0.01"></div>
            <div><label>Pitch Kd</label><input id="pitch_kd" type="number" step="0.01"></div>

            <div><label>Yaw Kp</label><input id="yaw_kp" type="number" step="0.01"></div>
            <div><label>Yaw Ki</label><input id="yaw_ki" type="number" step="0.01"></div>
            <div><label>Yaw Kd</label><input id="yaw_kd" type="number" step="0.01"></div>
        </div>
    </div>

    <div class="panel">
        <h2>Live desired setpoints</h2>
        <div class="grid">
            <div><label>Desired roll (deg)</label><input id="desired_roll_deg" type="number" step="0.1"></div>
            <div><label>Desired pitch (deg)</label><input id="desired_pitch_deg" type="number" step="0.1"></div>
            <div><label>Desired yaw (deg)</label><input id="desired_yaw_deg" type="number" step="0.1"></div>
            <div><label>Desired surge force (N)</label><input id="desired_surge_force_N" type="number" step="0.1"></div>
        </div>
    </div>

    <div class="panel downloads">
        <h2>Generated files</h2>
        <a href="generated/actuator_specs.h" target="_blank">Download actuator_specs.h</a>
        <a href="generated/actuator_specs.c" target="_blank">Download actuator_specs.c</a>
        <a href="generated/generated_vehicle_settings.h" target="_blank">Download generated_vehicle_settings.h</a>
        <a href="generated/control_runtime.cfg" target="_blank">Download control_runtime.cfg</a>
    </div>

    <div id="actuator-list"></div>

    <script>
        let state = null;

        function apiBase() {
            const p = window.location.pathname;
            if (p.startsWith("/extensionv2/smartcontrolallocation")) {
                return "/extensionv2/smartcontrolallocation";
            }
            return "";
        }

        function status(msg, isError=false) {
            const el = document.getElementById("status");
            el.textContent = msg;
            el.style.color = isError ? "#f87171" : "#4ade80";
        }

        function num(v, fallback=0) {
            const n = Number(v);
            return Number.isFinite(n) ? n : fallback;
        }

        function defaultFin() {
            return {
                name: "new_fin",
                kind: "fin",
                enabled: true,
                inverted: false,
                channel: 1,
                cmd_min: -30,
                cmd_neutral: 0,
                cmd_max: 30,
                pwm_min: 1000,
                pwm_max: 2000,
                position: { x: 0, y: 0, z: 0 },
                force_dir: { x: 0, y: 0, z: 1 },
                area_m2: 0.0035
            };
        }

        function defaultThruster() {
            return {
                name: "new_thruster",
                kind: "thruster",
                enabled: true,
                inverted: false,
                channel: 1,
                cmd_min: 0,
                cmd_neutral: 0,
                cmd_max: 100,
                pwm_min: 1000,
                pwm_max: 2000,
                position: { x: 0, y: 0, z: 0 },
                force_dir: { x: 1, y: 0, z: 0 },
                thrust_gain_pos_N_per_cmd: 0.45,
                thrust_gain_neg_N_per_cmd: 0.0
            };
        }

        async function loadConfig() {
            try {
                const res = await fetch(`${apiBase()}/api/config`);
                if (!res.ok) throw new Error(await res.text());
                state = await res.json();

                document.getElementById("water_density_kg_m3").value = state.vehicle.water_density_kg_m3;
                document.getElementById("axis_weight_surge").value = state.vehicle.axis_weight_surge;
                document.getElementById("axis_weight_roll").value = state.vehicle.axis_weight_roll;
                document.getElementById("axis_weight_pitch").value = state.vehicle.axis_weight_pitch;
                document.getElementById("axis_weight_yaw").value = state.vehicle.axis_weight_yaw;

                document.getElementById("roll_kp").value = state.pid.roll.kp;
                document.getElementById("roll_ki").value = state.pid.roll.ki;
                document.getElementById("roll_kd").value = state.pid.roll.kd;

                document.getElementById("pitch_kp").value = state.pid.pitch.kp;
                document.getElementById("pitch_ki").value = state.pid.pitch.ki;
                document.getElementById("pitch_kd").value = state.pid.pitch.kd;

                document.getElementById("yaw_kp").value = state.pid.yaw.kp;
                document.getElementById("yaw_ki").value = state.pid.yaw.ki;
                document.getElementById("yaw_kd").value = state.pid.yaw.kd;

                document.getElementById("desired_roll_deg").value = state.runtime.desired_roll_deg;
                document.getElementById("desired_pitch_deg").value = state.runtime.desired_pitch_deg;
                document.getElementById("desired_yaw_deg").value = state.runtime.desired_yaw_deg;
                document.getElementById("desired_surge_force_N").value = state.runtime.desired_surge_force_N;

                renderActuators();
                status("Config loaded");
            } catch (err) {
                status(`Failed to load config: ${err}`, true);
            }
        }

        function renderActuators() {
            const root = document.getElementById("actuator-list");
            root.innerHTML = "";

            state.actuators.forEach((a, i) => {
                const card = document.createElement("div");
                card.className = "card";

                const extra =
                    a.kind === "fin"
                    ? `
                        <div class="grid">
                            <div>
                                <label>Area m²</label>
                                <input type="number" step="0.0001" value="${a.area_m2 ?? 0.0035}" onchange="updateActuator(${i}, 'area_m2', this.value)">
                            </div>
                        </div>
                      `
                    : `
                        <div class="grid">
                            <div>
                                <label>Thrust gain +N/cmd</label>
                                <input type="number" step="0.01" value="${a.thrust_gain_pos_N_per_cmd ?? 0.45}" onchange="updateActuator(${i}, 'thrust_gain_pos_N_per_cmd', this.value)">
                            </div>
                            <div>
                                <label>Thrust gain -N/cmd</label>
                                <input type="number" step="0.01" value="${a.thrust_gain_neg_N_per_cmd ?? 0.0}" onchange="updateActuator(${i}, 'thrust_gain_neg_N_per_cmd', this.value)">
                            </div>
                        </div>
                      `;

                card.innerHTML = `
                    <h2>Actuator ${i + 1}</h2>
                    <div class="grid">
                        <div>
                            <label>Name</label>
                            <input value="${a.name ?? ""}" onchange="updateActuator(${i}, 'name', this.value)">
                        </div>
                        <div>
                            <label>Type</label>
                            <select onchange="changeKind(${i}, this.value)">
                                <option value="fin" ${a.kind === "fin" ? "selected" : ""}>fin</option>
                                <option value="thruster" ${a.kind === "thruster" ? "selected" : ""}>thruster</option>
                            </select>
                        </div>
                        <div>
                            <label>Channel</label>
                            <input type="number" step="1" value="${a.channel ?? 0}" onchange="updateActuator(${i}, 'channel', this.value)">
                        </div>
                        <div>
                            <label>Enabled</label>
                            <select onchange="updateActuator(${i}, 'enabled', this.value === 'true')">
                                <option value="true" ${a.enabled ? "selected" : ""}>true</option>
                                <option value="false" ${!a.enabled ? "selected" : ""}>false</option>
                            </select>
                        </div>
                        <div>
                            <label>Inverted</label>
                            <select onchange="updateActuator(${i}, 'inverted', this.value === 'true')">
                                <option value="false" ${!a.inverted ? "selected" : ""}>false</option>
                                <option value="true" ${a.inverted ? "selected" : ""}>true</option>
                            </select>
                        </div>
                    </div>

                    <div class="grid">
                        <div>
                            <label>Cmd min</label>
                            <input type="number" step="0.1" value="${a.cmd_min}" onchange="updateActuator(${i}, 'cmd_min', this.value)">
                        </div>
                        <div>
                            <label>Cmd neutral</label>
                            <input type="number" step="0.1" value="${a.cmd_neutral}" onchange="updateActuator(${i}, 'cmd_neutral', this.value)">
                        </div>
                        <div>
                            <label>Cmd max</label>
                            <input type="number" step="0.1" value="${a.cmd_max}" onchange="updateActuator(${i}, 'cmd_max', this.value)">
                        </div>
                    </div>

                    <div class="grid">
                        <div>
                            <label>PWM min</label>
                            <input type="number" step="1" value="${a.pwm_min ?? 1000}" onchange="updateActuator(${i}, 'pwm_min', this.value)">
                        </div>
                        <div>
                            <label>PWM max</label>
                            <input type="number" step="1" value="${a.pwm_max ?? 2000}" onchange="updateActuator(${i}, 'pwm_max', this.value)">
                        </div>
                    </div>

                    <h3>Position (m)</h3>
                    <div class="grid">
                        <div>
                            <label>X</label>
                            <input type="number" step="0.01" value="${a.position?.x ?? 0}" onchange="updateVec(${i}, 'position', 'x', this.value)">
                        </div>
                        <div>
                            <label>Y</label>
                            <input type="number" step="0.01" value="${a.position?.y ?? 0}" onchange="updateVec(${i}, 'position', 'y', this.value)">
                        </div>
                        <div>
                            <label>Z</label>
                            <input type="number" step="0.01" value="${a.position?.z ?? 0}" onchange="updateVec(${i}, 'position', 'z', this.value)">
                        </div>
                    </div>

                    <h3>Force direction</h3>
                    <div class="grid">
                        <div>
                            <label>X</label>
                            <input type="number" step="0.1" value="${a.force_dir?.x ?? 0}" onchange="updateVec(${i}, 'force_dir', 'x', this.value)">
                        </div>
                        <div>
                            <label>Y</label>
                            <input type="number" step="0.1" value="${a.force_dir?.y ?? 0}" onchange="updateVec(${i}, 'force_dir', 'y', this.value)">
                        </div>
                        <div>
                            <label>Z</label>
                            <input type="number" step="0.1" value="${a.force_dir?.z ?? 0}" onchange="updateVec(${i}, 'force_dir', 'z', this.value)">
                        </div>
                    </div>

                    ${extra}

                    <button class="danger" onclick="removeActuator(${i})">Delete</button>
                `;

                root.appendChild(card);
            });
        }

        function updateActuator(index, key, value) {
            if (typeof state.actuators[index] === "undefined") return;
            if (typeof value === "string" && value !== "" && !isNaN(value)) {
                state.actuators[index][key] = Number(value);
            } else {
                state.actuators[index][key] = value;
            }
        }

        function updateVec(index, vecKey, axis, value) {
            if (!state.actuators[index][vecKey]) {
                state.actuators[index][vecKey] = { x: 0, y: 0, z: 0 };
            }
            state.actuators[index][vecKey][axis] = num(value, 0);
        }

        function changeKind(index, kind) {
            const oldName = state.actuators[index]?.name || "";
            state.actuators[index] = kind === "fin" ? defaultFin() : defaultThruster();
            state.actuators[index].name = oldName || state.actuators[index].name;
            renderActuators();
        }

        function addFin() {
            state.actuators.push(defaultFin());
            renderActuators();
        }

        function addThruster() {
            state.actuators.push(defaultThruster());
            renderActuators();
        }

        function removeActuator(index) {
            state.actuators.splice(index, 1);
            renderActuators();
        }

        function collectConfig() {
            return {
                vehicle: {
                    water_density_kg_m3: num(document.getElementById("water_density_kg_m3").value, 1025),
                    axis_weight_surge: num(document.getElementById("axis_weight_surge").value, 1.0),
                    axis_weight_roll: num(document.getElementById("axis_weight_roll").value, 1.5),
                    axis_weight_pitch: num(document.getElementById("axis_weight_pitch").value, 1.5),
                    axis_weight_yaw: num(document.getElementById("axis_weight_yaw").value, 2.0)
                },
                pid: {
                    roll: {
                        kp: num(document.getElementById("roll_kp").value, 1.0),
                        ki: num(document.getElementById("roll_ki").value, 0.0),
                        kd: num(document.getElementById("roll_kd").value, 0.0)
                    },
                    pitch: {
                        kp: num(document.getElementById("pitch_kp").value, 1.0),
                        ki: num(document.getElementById("pitch_ki").value, 0.0),
                        kd: num(document.getElementById("pitch_kd").value, 0.0)
                    },
                    yaw: {
                        kp: num(document.getElementById("yaw_kp").value, 1.0),
                        ki: num(document.getElementById("yaw_ki").value, 0.0),
                        kd: num(document.getElementById("yaw_kd").value, 0.0)
                    }
                },
                runtime: {
                    desired_roll_deg: num(document.getElementById("desired_roll_deg").value, 0.0),
                    desired_pitch_deg: num(document.getElementById("desired_pitch_deg").value, 0.0),
                    desired_yaw_deg: num(document.getElementById("desired_yaw_deg").value, 0.0),
                    desired_surge_force_N: num(document.getElementById("desired_surge_force_N").value, 20.0)
                },
                actuators: state.actuators
            };
        }

        async function saveConfig() {
            try {
                const response = await fetch(`${apiBase()}/api/config`, {
                    method: "POST",
                    headers: { "Content-Type": "application/json" },
                    body: JSON.stringify({ config: collectConfig() })
                });

                if (!response.ok) {
                    throw new Error(await response.text());
                }

                state = collectConfig();
                status("Config saved");
            } catch (err) {
                status(`Failed to save config: ${err}`, true);
            }
        }

        async function generateFiles() {
            try {
                const response = await fetch(`${apiBase()}/api/generate`, {
                    method: "POST",
                    headers: { "Content-Type": "application/json" },
                    body: JSON.stringify({ config: collectConfig() })
                });

                if (!response.ok) {
                    throw new Error(await response.text());
                }

                state = collectConfig();
                status("Generated actuator files and live runtime config");
            } catch (err) {
                status(`Failed to generate files: ${err}`, true);
            }
        }

        async function startRuntime() {
            try {
                const response = await fetch(`${apiBase()}/api/runtime/start`, {
                    method: "POST"
                });
                if (!response.ok) {
                    throw new Error(await response.text());
                }
                await refreshRuntime();
                status("Runtime started");
            } catch (err) {
                status(`Failed to start runtime: ${err}`, true);
            }
        }

        async function stopRuntime() {
            try {
                const response = await fetch(`${apiBase()}/api/runtime/stop`, {
                    method: "POST"
                });
                if (!response.ok) {
                    throw new Error(await response.text());
                }
                await refreshRuntime();
                status("Runtime stopped");
            } catch (err) {
                status(`Failed to stop runtime: ${err}`, true);
            }
        }

        async function refreshRuntime() {
            try {
                const statusRes = await fetch(`${apiBase()}/api/runtime/status`);
                const logsRes = await fetch(`${apiBase()}/api/runtime/logs`);

                const runtimeStatus = await statusRes.json();
                const runtimeLogs = await logsRes.json();

                document.getElementById("runtime-status").textContent =
                    runtimeStatus.running
                    ? `Running (pid ${runtimeStatus.pid})`
                    : "Stopped";

                document.getElementById("runtime-status").style.color =
                    runtimeStatus.running ? "#4ade80" : "#f87171";

                document.getElementById("runtime-console").textContent =
                    runtimeLogs.lines.join("\\n");
            } catch (err) {
                document.getElementById("runtime-status").textContent =
                    `Runtime status error: ${err}`;
                document.getElementById("runtime-status").style.color = "#f87171";
            }
        }

        loadConfig();
        refreshRuntime();
        setInterval(refreshRuntime, 1000);
    </script>
</body>
</html>
"""


@app.get("/register_service")
def register_service():
    return {
        "name": "Smart Control Allocation",
        "description": "Configure smart control allocation for the EXPO60 underwater drone.",
        "icon": "mdi-tune-variant",
        "company": "EXPO60",
        "version": "0.1.0",
        "webpage": "/",
        "api": "/",
        "new_page": False,
        "works_in_relative_paths": True,
    }


@app.get("/", response_class=HTMLResponse)
def index():
    return page_html()


@app.get(EXTENSION_PREFIX, response_class=HTMLResponse)
@app.get(EXTENSION_PREFIX + "/", response_class=HTMLResponse)
def extension_index():
    return page_html()


@app.get("/api/config")
@app.get(EXTENSION_PREFIX + "/api/config")
def api_get_config():
    return load_config()


@app.post("/api/config")
@app.post(EXTENSION_PREFIX + "/api/config")
def api_save_config(payload: ConfigPayload):
    config = payload.config
    save_config(config)
    generate_files(config)
    return {"ok": True}


@app.post("/api/generate")
@app.post(EXTENSION_PREFIX + "/api/generate")
def api_generate(payload: ConfigPayload):
    config = payload.config
    save_config(config)
    generate_files(config)
    return {"ok": True}


@app.post("/api/runtime/start")
@app.post(EXTENSION_PREFIX + "/api/runtime/start")
def api_runtime_start():
    return start_runtime_process()


@app.post("/api/runtime/stop")
@app.post(EXTENSION_PREFIX + "/api/runtime/stop")
def api_runtime_stop():
    return stop_runtime_process()


@app.get("/api/runtime/status")
@app.get(EXTENSION_PREFIX + "/api/runtime/status")
def api_runtime_status():
    return runtime_status()


@app.get("/api/runtime/logs")
@app.get(EXTENSION_PREFIX + "/api/runtime/logs")
def api_runtime_logs():
    return {"lines": list(runtime_logs)}


@app.get("/generated/{filename}")
@app.get(EXTENSION_PREFIX + "/generated/{filename}")
def get_generated_file(filename: str):
    if filename not in ALLOWED_DOWNLOADS:
        raise HTTPException(status_code=404, detail="File not found")

    path = ALLOWED_DOWNLOADS[filename]
    if not os.path.exists(path):
        cfg = load_config()
        generate_files(cfg)

    return FileResponse(path, filename=filename)


ensure_dirs()
generate_files(load_config())