import json
import os
import re
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

ALLOWED_DOWNLOADS = {
    "actuator_specs.h": GENERATED_HEADER_PATH,
    "actuator_specs.c": GENERATED_SOURCE_PATH,
    "generated_vehicle_settings.h": GENERATED_VEHICLE_SETTINGS_PATH,
}


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
        "actuators": [
            {
                "name": "roll_left_front",
                "kind": "fin",
                "enabled": True,
                "inverted": False,
                "channel": 5,
                "cmd_min": -30.0,
                "cmd_neutral": 0.0,
                "cmd_max": 30.0,
                "position": {"x": 0.20, "y": -0.10, "z": 0.00},
                "force_dir": {"x": 0.0, "y": 0.0, "z": 1.0},
                "area_m2": 0.0035,
            },
            {
                "name": "main_thruster",
                "kind": "thruster",
                "enabled": True,
                "inverted": False,
                "channel": 0,
                "cmd_min": 0.0,
                "cmd_neutral": 0.0,
                "cmd_max": 100.0,
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


def generate_files(config):
    ensure_dirs()

    with open(GENERATED_HEADER_PATH, "w", encoding="utf-8") as f:
        f.write(generate_header_text())

    with open(GENERATED_SOURCE_PATH, "w", encoding="utf-8") as f:
        f.write(generate_source_text(config))

    with open(GENERATED_VEHICLE_SETTINGS_PATH, "w", encoding="utf-8") as f:
        f.write(generate_vehicle_settings_text(config))


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
        .toolbar, .vehicle, .downloads {
            background: #1e293b;
            border-radius: 12px;
            padding: 16px;
            margin-bottom: 16px;
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
        .toolbar button.secondary, .card button.secondary {
            background: #475569;
            color: white;
        }
        .toolbar button.danger, .card button.danger {
            background: #ef4444;
            color: white;
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
    </style>
</head>
<body>
    <h1>Smart Control Allocation</h1>

    <div class="toolbar">
        <button onclick="loadConfig()">Reload</button>
        <button onclick="saveConfig()">Save</button>
        <button onclick="generateFiles()">Generate C Files</button>
        <button class="secondary" onclick="addFin()">Add Fin</button>
        <button class="secondary" onclick="addThruster()">Add Thruster</button>
        <div id="status" class="status"></div>
    </div>

    <div class="vehicle">
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
        <p class="small">For seawater use 1025.</p>
    </div>

    <div class="downloads">
        <h2>Generated files</h2>
        <a href="generated/actuator_specs.h" target="_blank">Download actuator_specs.h</a>
        <a href="generated/actuator_specs.c" target="_blank">Download actuator_specs.c</a>
        <a href="generated/generated_vehicle_settings.h" target="_blank">Download generated_vehicle_settings.h</a>
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
                channel: 0,
                cmd_min: 0,
                cmd_neutral: 0,
                cmd_max: 100,
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
                    axis_weight_yaw: num(document.getElementById("axis_weight_yaw").value, 2.0),
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

                status("Generated actuator_specs.h/.c and vehicle settings");
            } catch (err) {
                status(`Failed to generate files: ${err}`, true);
            }
        }

        loadConfig();
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
        "version": "0.0.2",
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