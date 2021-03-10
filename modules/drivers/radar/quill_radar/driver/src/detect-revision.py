import os
import sys
from subprocess import check_output

# force chdir to be the folder containing this script
os.chdir(os.path.dirname(os.path.abspath(__file__)))

fname = "rra-revdesc.h"

try:
    if os.path.exists("../.hg_archival.txt"):
        with open("../.hg_archival.txt", "r") as fp:
            fields = {}
            for line in fp.read().splitlines():
                var, value = line.split(": ")
                fields[var] = value
        if "tag" in fields:
            tag = fields["tag"]
            distance = "0"
        else:
            tag = fields["latesttag"]
            distance = fields["latesttagdistance"]
        node = fields["node"]
    else:
        out = check_output(
            [
                "hg",
                "log",
                "--rev=.",
                "--template",
                "{latesttag}\n{latesttagdistance}\n{node}",
            ]
        )
        tag, distance, node = out.decode("utf-8").split("\n")
except Exception as e:
    if os.path.exists(fname):
        # revision file already exists, do not overwrite with garbage
        sys.exit(0)
    print('Error detecting software version, using "unknown": %s' % str(e))
    tag, distance, node = "N/A", "0", "unknown"

version = "%s+%s-%s" % (tag, distance, node[:12])
contents = (
    "\n".join(
        [
            "// Machine generated file, do not edit",
            'const char* swrev = "hg-tag-string:%s";' % version,
        ]
    )
    + "\n"
)

if not os.path.exists(fname) or open(fname).read() != contents:
    open(fname, "w").write(contents)

srsfname = "../system-radar-software/engine/common/detect-revision.py"
if os.path.exists(srsfname):
    check_output([sys.executable, srsfname])
