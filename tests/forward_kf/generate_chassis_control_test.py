"""Use the current firmware's bodies/layouts in the portable control-chain test."""
from pathlib import Path
import re
import sys


def extract(text, marker):
    start = text.index(marker)
    masked = re.sub(r'//[^\n]*|/\*[\s\S]*?\*/|"(?:\\.|[^"\\])*"',
                    lambda match: " " * len(match.group()), text)
    opening = masked.index("{", start)
    depth = 0
    for index in range(opening, len(masked)):
        if masked[index] == "{":
            depth += 1
        elif masked[index] == "}":
            depth -= 1
            if depth == 0:
                return text[start:index + 1]
    raise ValueError(f"Unterminated body: {marker}")


root = Path(__file__).resolve().parents[2]
chassis = root / "Robot/infantry1/Chassis"
out = Path(sys.argv[1])
out.mkdir(parents=True, exist_ok=True)
header = (chassis / "pyro_wl_chassis.h").read_text(encoding="utf-8")
types = ["struct wl_chassis_deps_t", "enum class chassis_active_state_t",
         "enum class chassis_function_state_t", "enum class jump_phase_t",
         "enum class jump_trigger_source_t", "struct wl_chassis_cmd_t",
         "struct leg_ctx_t", "struct wheel_ctx_t", "struct yaw_ctx_t",
         "struct state_vec_t", "struct control_vec_t", "struct chassis_aux_state_t",
         "struct roll_pid_data_t", "struct mean_leg_length_pid_data_t", "struct odom_t",
         "struct ins_data_t", "struct airborne_data_t", "struct flag_data_t",
         "struct wl_chassis_data_ctx_t", "struct wl_chassis_ctx_t"]
(out / "chassis_types.inc").write_text(
    "\n\n".join(extract(header, marker) + ";" for marker in types), encoding="utf-8")

source = (chassis / "pyro_wl_chassis.cpp").read_text(encoding="utf-8")
functions = []
instrument = {"_update_forward_kf", "_balance_control"}
for result, name in (("void", "_update_feedback"), ("void", "_gain_calculate"),
                     ("void", "_calc_ddot_x_imu"), ("void", "_reset_forward_kf"),
                     ("void", "_update_forward_kf"), ("void", "_apply_forward_estimate"),
                     ("float", "_calc_roll_force_trim"), ("float", "_calc_mean_leg_length_force_trim"),
                     ("void", "_balance_control")):
    body = extract(source, f"{result} wl_chassis_t::{name}(")
    if name in instrument:
        body = body.replace(f"wl_chassis_t::{name}(", f"wl_chassis_t::{name}_impl(", 1)
    functions.append(body)
for state in ("balance", "spin"):
    source = (chassis / "fsm/active/normal" / f"pyro_wl_{state}_state.cpp").read_text(encoding="utf-8")
    namespace_start = source.index("{", source.index("namespace pyro")) + 1
    functions.append(source[namespace_start:source.index("void wl_chassis_t::")])
    prefix = f"void wl_chassis_t::fsm_active_t::state_normal_t::state_{state}_t::"
    for name in ("enter", "execute", "exit"):
        functions.append(extract(source, prefix + name + "("))
(out / "chassis_functions.inc").write_text(
    "namespace pyro\n{\n" + "\n\n".join(functions) + "\n}\n", encoding="utf-8")
