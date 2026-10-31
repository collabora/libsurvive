import subprocess


def get_version():
    try:
        version = subprocess.check_output(["git", "describe", "--tags", "--long"], stderr=subprocess.DEVNULL)
        version = version.strip().decode('utf-8').replace("-", ".")[1:]
        return version[:version.rfind("g")-1]
    except (subprocess.CalledProcessError, OSError):
        pass

    # No tags (e.g. a fork that hasn't fetched upstream tags) or no git at all
    try:
        count = subprocess.check_output(["git", "rev-list", "--count", "HEAD"], stderr=subprocess.DEVNULL)
        return "0.0.0." + count.strip().decode('utf-8')
    except (subprocess.CalledProcessError, OSError):
        return "0.0.0"


print(get_version())
