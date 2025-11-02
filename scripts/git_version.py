import os
import subprocess

from SCons.Script import Import

Import("env")


def _git_output(args):
    try:
        return subprocess.check_output(
            args,
            cwd=env["PROJECT_DIR"],
            stderr=subprocess.DEVNULL,
        ).decode("utf-8").strip()
    except (subprocess.CalledProcessError, FileNotFoundError, OSError):
        return None


def _escape(value: str) -> str:
    return value.replace('"', '\\"')


def _ensure_define(name, value, quoted=True):
    if quoted:
        env.Append(CPPDEFINES=[f'{name}="{_escape(value)}"'])
    else:
        env.Append(CPPDEFINES=[f"{name}={value}"])


hash_value = _git_output(["git", "rev-parse", "--short", "HEAD"]) or "unknown"
tag_value = _git_output(["git", "describe", "--tags", "--exact-match"])
if tag_value:
    ref_value = tag_value
else:
    ref_value = _git_output(["git", "rev-parse", "--abbrev-ref", "HEAD"]) or "unknown"

commit_time = _git_output(["git", "show", "-s", "--format=%cI", "HEAD"]) or "unknown"

_ensure_define("GIT_HASH", hash_value)
_ensure_define("GIT_REF", ref_value)
_ensure_define("GIT_COMMIT_TIME", commit_time)
_ensure_define("GIT_IS_TAG", "1" if bool(tag_value) else "0", quoted=False)
