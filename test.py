import re
import os
import subprocess
import argparse
import shutil
import sys
        
UNIT_NAME = "Top"
SRC_PATH = os.getenv("SRC_HOME")
SRC_UNIT_PATH = os.path.join(SRC_PATH, UNIT_NAME)
INCLUDE_PATH = os.path.join(SRC_PATH, "include")
HLS_PATH = f"hls/{UNIT_NAME}/"

AWS_PATH = os.getenv("AWS_HOME")

def systemc_sim():
    test_list = []
    test_status = []
    # Create the reports directory if it doesn't exist
    reports_dir = "reports/hls"
    os.makedirs(reports_dir, exist_ok=True)
    log_file = os.path.join(reports_dir, "systemc.log.txt")

    # List of directories to run 'make' in
    make_dirs = [
        "src/Top/PEPartition/PEModule/ActUnit",
        "src/Top/PEPartition/PEModule/PECore",
        "src/Top/PEPartition/PEModule",
        "src/Top/PEPartition",
        "src/Top/GBPartition/GBModule/NMP",
        "src/Top/GBPartition/GBModule/GBCore",
        "src/Top/GBPartition/GBModule/GBControl",
        "src/Top/GBPartition/GBModule",
        "src/Top/GBPartition",
        "src/Top",
    ]

    # Clear the log file
    with open(log_file, "w") as f:
        f.write("")

    for directory in make_dirs:
        testname = directory.split("/")[-1]
        test_list.append(testname)
        result = None
        try:
            # Run the make command
            result = subprocess.run(
                "make",
                cwd=directory,
                capture_output=True,
                text=True,
                check=True,
            )
            # Write the output to the log file
            with open(log_file, "a") as f:
                f.write(f"--- Output for {directory} ---\n")
                f.write(result.stdout)
                f.write(result.stderr)
                f.write("\n")
            print(result.stdout)

            if "TESTBENCH PASS" in result.stdout:
                test_status.append("PASSED")
            elif "TESTBENCH FAIL" in result.stdout:
                test_status.append("FAILED")
            else:
                test_status.append("UNKNOWN")

        except (subprocess.CalledProcessError, FileNotFoundError) as e:
            with open(log_file, "a") as f:
                f.write(f"--- Error for {directory} ---\n")
                f.write(str(e))
                f.write("\n")
            test_status.append("FAILED")

    print("--- SystemC Simulation Results ---")
    print("{:<20} {:<10}".format("Test", "Status"))
    print("-" * 30)
    for test, status in zip(test_list, test_status):
        print("{:<20} {:<10}".format(test, status))
    print("-" * 30)

def rtl_sim():
    test_list = []
    test_status = []
    # Create the reports directory if it doesn't exist
    reports_dir = "reports/hls"
    os.makedirs(reports_dir, exist_ok=True)
    log_file = os.path.join(reports_dir, "rtl_sim.log.txt")

    # List of directories to run 'make' in
    make_dirs = [
        "hls/Top/PEPartition/PEModule/ActUnit",
        "hls/Top/PEPartition/PEModule/PECore",
        "hls/Top/PEPartition/PEModule",
        "hls/Top/PEPartition",
        "hls/Top/GBPartition/GBModule/NMP",
        "hls/Top/GBPartition/GBModule/GBCore",
        "hls/Top/GBPartition/GBModule/GBControl",
        "hls/Top/GBPartition/GBModule",
        "hls/Top/GBPartition",
        "hls/Top",
    ]

    # Clear the log file
    with open(log_file, "w") as f:
        f.write("")

    for directory in make_dirs:
        result = None
        try:
            # Run the make command
            result = subprocess.run(
                ["make", "hls"],
                cwd=directory,
                capture_output=True,
                text=True,
                check=True,
            )
            # Write the output to the log file
            with open(log_file, "a") as f:
                f.write(f"--- Output for {directory} ---\n")
                f.write(result.stdout)
                f.write(result.stderr)
                f.write("\n")
            print(result.stdout)

        except (subprocess.CalledProcessError, FileNotFoundError) as e:
            with open(log_file, "a") as f:
                f.write(f"--- Error for {directory} ---\n")
                f.write(str(e))
                f.write("\n")

        testname = directory.split("/")[-1]
        test_list.append(testname)

        if result and "TESTBENCH PASS" in result.stdout:
            test_status.append("PASSED")
        elif result and "TESTBENCH FAIL" in result.stdout:
            test_status.append("FAILED")
        else:
            test_status.append("UNKNOWN")

    print("--- RTL Simulation Results ---")
    print("{:<20} {:<10}".format("Test", "Status"))
    print("-" * 30)
    for test, status in zip(test_list, test_status):
        print("{:<20} {:<10}".format(test, status))
    print("-" * 30)

def clean():
    """
    Cleans up generated files from simulations and synthesis.
    """
    print("\n--- Cleaning up generated files ---")
    
    # The top-level Makefile's 'clean' target handles cleaning subdirectories.
    # We run it from the script's directory to ensure correct relative paths.
    repo_top = os.path.dirname(os.path.abspath(__file__))
    print(f"Running 'make clean' in {repo_top}")
    subprocess.run(["make", "clean"], check=False, cwd=repo_top)

def copy_rtl():
    cmd = "make copy_rtl"
    subprocess.run(cmd, shell=True, check=True)
    return

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run simulation and other tasks.")
    parser.add_argument(
        "--action",
        choices=["systemc_sim", "rtl_sim", "clean", "copy_rtl"],
        required=True,
        help="The action to perform.",
    )
    args = parser.parse_args()

    if args.action == "systemc_sim":
        systemc_sim()
    elif args.action == "rtl_sim":
        rtl_sim()
        copy_rtl()
    elif args.action == "clean":
        clean()
    elif args.action == "copy_rtl":
        copy_rtl()

