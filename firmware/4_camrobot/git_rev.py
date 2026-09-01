# PlatformIO pre-build hook: stamp the short git revision into the firmware as
# GIT_REV, so a running robot can say exactly which commit it was built from.
#
# Degrades quietly on purpose. A missing git binary, a tarball with no .git, or
# a shallow CI checkout must not fail the build -- the firmware already defines
# GIT_REV as "nogit" when we don't append anything here, and an honest "unknown"
# beats a stale or wrong hash.
Import("env")
import subprocess

def git(*args):
    try:
        return subprocess.check_output(
            ["git"] + list(args),
            stderr=subprocess.DEVNULL,
            cwd=env.subst("$PROJECT_DIR"),
        ).decode().strip()
    except Exception:
        return ""

rev = git("rev-parse", "--short", "HEAD")
if rev:
    # Uncommitted changes mean the hash alone no longer identifies the binary,
    # so say so rather than implying a clean build. The control page renders a
    # -dirty suffix in amber.
    if git("status", "--porcelain"):
        rev += "-dirty"
    env.Append(CPPDEFINES=[("GIT_REV", env.StringifyMacro(rev))])
    print("git_rev: GIT_REV=%s" % rev)
else:
    print("git_rev: no git metadata available, firmware will report 'nogit'")
