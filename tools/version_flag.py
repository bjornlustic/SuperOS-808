# version_flag.py -- pre-script: expose the project version + git revision to
# the firmware as SUPEROS_VERSION ("0.3 159f80e"), reported over SysEx
# (CMD_REQ_VERSION / CMD_VERSION, see src/midi.cpp) and shown in the web editor.
Import("env")
import subprocess

ver = env.GetProjectOption("custom_project_version")
try:
    rev = subprocess.check_output(["git", "rev-parse", "--short", "HEAD"]).decode().strip()
    dirty = subprocess.check_output(["git", "status", "-s", "--untracked-files=no"]).decode()
    if dirty:
        rev += "*"
except Exception:
    rev = "unknown"

env.Append(CPPDEFINES=[("SUPEROS_VERSION", env.StringifyMacro(f"{ver} {rev}"))])
