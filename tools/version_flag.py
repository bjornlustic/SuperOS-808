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

# Same version as two integers, for the SysEx device-info reply (0x61). It used
# to be written out by hand there and drifted from this file the moment the
# version moved, which is the sort of thing nobody notices until a released
# binary reports the previous version to the editor.
maj, _, minor = str(ver).partition(".")
try:
    maj_i, min_i = int(maj), int(minor or 0)
except ValueError:
    maj_i, min_i = 0, 0
env.Append(CPPDEFINES=[("SUPEROS_VER_MAJOR", maj_i), ("SUPEROS_VER_MINOR", min_i)])
