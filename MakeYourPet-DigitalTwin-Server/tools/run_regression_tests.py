#!/usr/bin/env python3
"""
Master Regression Test Runner for MakeYourPet-DigitalTwin-Server
Runs all unit, integration, invariant, and closed-loop full-simulation test suites.

Usage:
  python tools/run_regression_tests.py [--all | --unit | --integration | --full-loop]
"""

import os
import sys
import time
import unittest
import argparse

# Ensure UTF-8 output under Windows cp950 environment
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8")

SERVER_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
TESTS_DIR = os.path.join(SERVER_DIR, "tests")

if SERVER_DIR not in sys.path:
    sys.path.insert(0, SERVER_DIR)
if TESTS_DIR not in sys.path:
    sys.path.insert(0, TESTS_DIR)

# ANSI terminal colors
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
BOLD = "\033[1m"
RESET = "\033[0m"

UNIT_TEST_MODULES = [
    ("Protocol Parser & Telemetry Frame", "test_protocol_parser"),
    ("Command Processor & Sanitization", "test_command_processor"),
    ("Deadman Switch Safety Watchdog", "test_deadman_watchdog"),
    ("Mock Telemetry & Contact Sensors", "test_mock_telemetry"),
    ("Kinematics & ONNX Model Invariants", "test_kinematics_and_onnx"),
    ("Web Tactical HUD & HTTP Server", "test_web_assets_and_http"),
]

INTEGRATION_TEST_MODULES = [
    ("WebSocket & Network Resilience", "test_network_and_websocket"),
]


def run_test_module(display_name: str, module_name: str, verbose: bool = False):
    print(f"\n{CYAN}▸ Running Suite: {BOLD}{display_name}{RESET} ({module_name}.py)")
    loader = unittest.TestLoader()
    suite = loader.loadTestsFromName(module_name)
    runner = unittest.TextTestRunner(verbosity=2 if verbose else 1)
    start_t = time.time()
    result = runner.run(suite)
    elapsed = time.time() - start_t

    total = result.testsRun
    failures = len(result.failures)
    errors = len(result.errors)
    passed = total - failures - errors

    if result.wasSuccessful():
        print(f"  {GREEN}✔ PASSED{RESET}: {passed}/{total} tests passed ({elapsed:.2f}s)")
    else:
        print(f"  {RED}✘ FAILED{RESET}: {passed}/{total} passed, {failures} failures, {errors} errors ({elapsed:.2f}s)")

    return {
        "name": display_name,
        "module": module_name,
        "total": total,
        "passed": passed,
        "failures": failures,
        "errors": errors,
        "elapsed": elapsed,
        "success": result.wasSuccessful()
    }


def run_full_loop_suite():
    print(f"\n{CYAN}▸ Running Suite: {BOLD}End-to-End Full-Loop Simulation{RESET} (test_full_loop_simulation.py)")
    from test_full_loop_simulation import run_full_simulation
    start_t = time.time()
    try:
        success = run_full_simulation()
    except Exception as e:
        print(f"{RED}Full loop test raised exception: {e}{RESET}")
        success = False
    elapsed = time.time() - start_t

    if success:
        print(f"  {GREEN}✔ PASSED{RESET}: All 8 full-loop integration phases completed ({elapsed:.2f}s)")
    else:
        print(f"  {RED}✘ FAILED{RESET}: Full-loop simulation encountered failures ({elapsed:.2f}s)")

    return {
        "name": "Full-Loop Simulation Test",
        "module": "test_full_loop_simulation",
        "total": 8,
        "passed": 8 if success else 0,
        "failures": 0 if success else 1,
        "errors": 0,
        "elapsed": elapsed,
        "success": success
    }


def main():
    parser = argparse.ArgumentParser(description="MakeYourPet DigitalTwin-Server Regression Test Runner")
    parser.add_argument("--all", action="store_true", help="Run all suites (Unit, Integration, and Full-Loop)")
    parser.add_argument("--unit", action="store_true", help="Run only fast unit tests")
    parser.add_argument("--integration", action="store_true", help="Run unit and network integration tests")
    parser.add_argument("--full-loop", action="store_true", help="Run only full-loop end-to-end simulation")
    parser.add_argument("--verbose", "-v", action="store_true", help="Verbose test runner output")
    args = parser.parse_args()

    # Default to running all if no specific filter is given
    run_all = args.all or (not args.unit and not args.integration and not args.full_loop)

    print("=" * 74)
    print(f"{BOLD}   MAKE YOUR PET - DIGITAL TWIN SERVER REGRESSION TEST HARNESS{RESET}")
    print("=" * 74)
    suite_start = time.time()

    reports = []

    # 1. Unit Tests
    if run_all or args.unit or args.integration:
        print(f"\n{YELLOW}{BOLD}[CATEGORY 1] Fast Unit & Safety Invariant Tests{RESET}")
        for display_name, mod_name in UNIT_TEST_MODULES:
            rep = run_test_module(display_name, mod_name, verbose=args.verbose)
            reports.append(rep)

    # 2. Integration Tests
    if run_all or args.integration:
        print(f"\n{YELLOW}{BOLD}[CATEGORY 2] Network & Async Integration Tests{RESET}")
        for display_name, mod_name in INTEGRATION_TEST_MODULES:
            rep = run_test_module(display_name, mod_name, verbose=args.verbose)
            reports.append(rep)

    # 3. Full-Loop Simulation Test
    if run_all or args.full_loop:
        print(f"\n{YELLOW}{BOLD}[CATEGORY 3] Closed-Loop End-to-End Simulation Test{RESET}")
        rep = run_full_loop_suite()
        reports.append(rep)

    total_time = time.time() - suite_start
    total_tests = sum(r["total"] for r in reports)
    total_passed = sum(r["passed"] for r in reports)
    total_failed = sum(r["failures"] + r["errors"] for r in reports)
    all_success = all(r["success"] for r in reports)

    # Final Summary Card
    print("\n" + "=" * 74)
    print(f"{BOLD}                    REGRESSION SUMMARY REPORT{RESET}")
    print("=" * 74)
    print(f" {'Suite Name':<38} | {'Passed/Total':<14} | {'Time':<8} | {'Status'}")
    print("-" * 74)
    for r in reports:
        status_str = f"{GREEN}PASS{RESET}" if r["success"] else f"{RED}FAIL{RESET}"
        count_str = f"{r['passed']}/{r['total']}"
        time_str = f"{r['elapsed']:.2f}s"
        print(f" {r['name']:<38} | {count_str:<14} | {time_str:<8} | {status_str}")
    print("-" * 74)
    print(f" Total Suites: {len(reports)} | Total Tests: {total_tests} | Passed: {total_passed} | Failed: {total_failed}")
    print(f" Total Duration: {total_time:.2f}s")

    if all_success:
        print(f"\n{GREEN}{BOLD}🎉 ALL REGRESSION TESTS PASSED SUCCESSFULLY! (100% SUCCESS){RESET}")
        sys.exit(0)
    else:
        print(f"\n{RED}{BOLD}❌ SOME TESTS FAILED. PLEASE CHECK DETAILED REPORT ABOVE.{RESET}")
        sys.exit(1)


if __name__ == "__main__":
    main()
