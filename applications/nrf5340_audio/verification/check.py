#
# Copyright (c) 2022 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

"""
Temporary script to perform checks in the applications/nrf5340_audio folder
"""

import subprocess
import glob
import os
import sys
from pathlib import Path


def check_run():
    """ Run checks"""
    print("=== Running checks ===")
    NRF_BASE = str(Path(__file__).absolute().parents[3])
    folders =\
    [NRF_BASE+'/applications/nrf5340_audio/nrf5340_audio_app/src/**/',
        NRF_BASE+'/tests/nrf5340_audio/**/']
    filetypes = ('*.c', '*.h')
    files = []
    files_failed = []

    for folder in folders:
        for filetype in filetypes:
            files.extend(glob.glob(folder+filetype, recursive=True))

        if not files:
            print("No files found")
            sys.exit(1)

    os.chdir(NRF_BASE)

    for file in files:
        print("======================")
        if file.find('build') != -1:
            print("Excluding" + file)
            continue

        print("Checking: "+file)
        file_passed = True
        cmd = "clang-format --Werror --dry-run " + file

        try:
            po = subprocess.Popen(cmd, shell=True, stderr=subprocess.PIPE,
                                  stdout=subprocess.PIPE)
            err, out = po.communicate()
        except Exception as exep:
            print(exep)
            sys.exit(1)
        else:
            if po.returncode != 0:
                print("FAIL: Clang")
                print(out.decode('utf-8'))
                file_passed = False

        # Hash from: git hash-object -t tree /dev/null. Cross-platform support
        cmd = "git diff 4b825dc642cb6eb9a060e54bf8d69288fbee4904 " + file +\
        " | perl "+os.getenv('ZEPHYR_BASE')+"/scripts/checkpatch.pl --no-tree"
        try:
            po = subprocess.Popen(cmd, shell=True, stderr=subprocess.PIPE)
            po.communicate()
        except Exception as exep:
            print(exep)
            sys.exit(1)
        else:
            if po.returncode != 0:
                print("FAIL: Checkpatch")
                file_passed = False

        if not file_passed:
            files_failed.append(file)

    cmd = os.getenv('ZEPHYR_BASE')+"/scripts/twister -v -T tests/ -t " +\
    "nrf5340_audio_unit_tests -p qemu_cortex_m3 -i"
    twister_failed = True
    try:
        po = subprocess.Popen(cmd, shell=True, stderr=subprocess.PIPE)
        err, out = po.communicate()
    except Exception as exep:
        print(exep)
        sys.exit(1)
    else:
        print(out.decode('utf-8'))
        if po.returncode != 0:
            print("Failed Twister")
        else:
            twister_failed = False

    if files_failed:
        print("Num files failed: " +
              str(len(files_failed)) + " of " + str(len(files)))

    if twister_failed:
        print("Twister failed")

    if files_failed or twister_failed:
        sys.exit(1)

    print("===Check.py passed!=== " + str(len(files)) + " files checked")
    sys.exit(0)

if __name__ == '__main__':
    check_run()
