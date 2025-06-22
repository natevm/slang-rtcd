#!/usr/bin/env python3
"""
Build script for Chapter 4 tests using Slang.

Removes any existing ch4Tests binary, invokes the Slang compiler to build
Chapter 4 test harness, logs all output to ch4build.log, and exits with an
error code if the build fails.
"""
import os
import sys
import subprocess


def main():
    bin_name = "ch4Tests"
    src_file = "ch4Tests.slang"
    log_file = "ch4build.log"

    try:
        os.remove(bin_name)
    except FileNotFoundError:
        pass

    cmd = ["slangc", "-target", "exe", 
           "-Xgenericcpp", "-Wno-format-security", 
           "-Xgenericcpp", "-Wno-pragma-once-outside-header", 
           "-Xgenericcpp", "-Wno-assume", 
           "-Xgenericcpp", "-Wno-invalid-noreturn",
           src_file, "-o", bin_name]
    process = subprocess.Popen(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
    )

    with open(log_file, "w") as logf:
        for line in process.stdout:
            sys.stdout.write(line)
            sys.stdout.flush()
            logf.write(line)
    process.wait()
    if process.returncode != 0:
        sys.exit(process.returncode)


if __name__ == "__main__":
    main()