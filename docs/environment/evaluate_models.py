#!/usr/bin/env python3
"""
RobertUN Rover Command — LLM Model Evaluation Script
Tests all 4 models against 12 standardized test cases via llama-server HTTP API.
Scores each response on 4 dimensions and produces a comparison report.
"""

import json
import time
import subprocess
import requests
import os
import sys

# =============================================================================
# CONFIGURATION
# =============================================================================

MODELS = [
    {
        "name": "Phi-3.5-mini-instruct",
        "path": "/data/models/Phi-3.5-mini-instruct-Q4_K_M.gguf",
        "extra_args": ["-b", "512", "-ub", "256"]
    },
    {
        "name": "Qwen2.5-3B-Instruct",
        "path": "/data/models/Qwen2.5-3B-Instruct-Q4_K_M.gguf",
        "extra_args": []
    },
    {
        "name": "Gemma-3-4B-it",
        "path": "/data/models/gemma-3-4b-it-Q4_K_M.gguf",
        "extra_args": ["-b", "512", "-ub", "256"]
    },
    {
        "name": "Qwen2.5-Coder-3B-Instruct",
        "path": "/data/models/Qwen2.5-Coder-3B-Instruct-Q4_K_M.gguf",
        "extra_args": []
    },
]

LLAMA_SERVER = "/home/talos/github/llama.cpp/build/bin/llama-server"
SERVER_HOST = "127.0.0.1"
SERVER_PORT = 8080
SERVER_URL = f"http://{SERVER_HOST}:{SERVER_PORT}"
TEST_CASES_FILE = "/data/rover_eval/test_cases.json"
from datetime import datetime as _dt
_timestamp = _dt.now().strftime("%Y%m%d_%H%M")
RESULTS_FILE = f"/data/rover_eval/results_{_timestamp}.json"
REPORT_FILE = f"/data/rover_eval/report_{_timestamp}.txt"

VALID_ACTIONS = {
    "move_forward", "move_backward", "turn_left",
    "turn_right", "stop", "take_photo", "analyze_sample"
}

# =============================================================================
# SERVER MANAGEMENT
# =============================================================================

def start_server(model):
    """Start llama-server for the given model. Returns the process handle."""
    cmd = [
        LLAMA_SERVER,
        "-m", model["path"],
        "--host", SERVER_HOST,
        "--port", str(SERVER_PORT),
        "-ngl", "99",
        "-fa", "1",
        "-c", "2048",
        "-t", "4",
        "-np", "1",
        "--alias", model["name"],
    ] + model["extra_args"]

    print(f"  Starting server for {model['name']}...")
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    return proc


def wait_for_server(timeout=60):
    """Wait until the server is ready to accept requests."""
    start = time.time()
    while time.time() - start < timeout:
        try:
            r = requests.get(f"{SERVER_URL}/health", timeout=2)
            if r.status_code == 200:
                return True
        except Exception:
            pass
        time.sleep(2)
    return False


def stop_server(proc):
    """Gracefully stop the server process."""
    proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
    time.sleep(2)


# =============================================================================
# INFERENCE
# =============================================================================

def query_model(system_prompt, user_input, temperature=0.3):
    """Send a single query to llama-server and return the response text and elapsed time."""
    payload = {
        "messages": [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_input}
        ],
        "temperature": temperature,
        "max_tokens": 512,
        "stream": False
    }
    try:
        t0 = time.time()
        r = requests.post(
            f"{SERVER_URL}/v1/chat/completions",
            json=payload,
            timeout=60
        )
        elapsed = round(time.time() - t0, 2)
        r.raise_for_status()
        return r.json()["choices"][0]["message"]["content"].strip(), elapsed
    except Exception as e:
        return f"ERROR: {e}", 0.0


# =============================================================================
# SCORING
# =============================================================================

def parse_response(text):
    """
    Try to parse the model response as JSON.
    Returns (parsed_object, is_valid_json).
    """
    # Strip markdown code blocks if present
    clean = text.strip()
    if clean.startswith("```"):
        lines = clean.split("\n")
        clean = "\n".join(lines[1:-1]) if len(lines) > 2 else clean
    try:
        return json.loads(clean), True
    except Exception:
        return None, False


def score_response(test_case, response_text):
    """
    Score a model response on 4 dimensions (0-1 each, total 0-4).

    Dimensions:
      1. json_valid      — response is parseable JSON (0 or 1)
      2. action_compliant — only uses valid actions from the list (0 or 1)
      3. correct_type    — commands vs clarification matches expected (0 or 1)
      4. quality         — semantic correctness / clarification usefulness (0, 0.5, or 1)
    """
    scores = {
        "json_valid": 0,
        "action_compliant": 0,
        "correct_type": 0,
        "quality": 0,
        "total": 0
    }
    notes = []

    parsed, is_valid = parse_response(response_text)

    # --- Dimension 1: JSON validity ---
    if is_valid:
        scores["json_valid"] = 1
    else:
        notes.append("Invalid JSON")
        return scores, notes

    # --- Determine response type ---
    is_clarification = (
        isinstance(parsed, dict) and
        parsed.get("status") == "CLARIFICATION_NEEDED"
    )
    is_commands = isinstance(parsed, list)

    # --- Dimension 2: Action compliance ---
    if is_commands:
        invalid_actions = []
        for cmd in parsed:
            if isinstance(cmd, dict):
                action = cmd.get("action", "")
                if action not in VALID_ACTIONS:
                    invalid_actions.append(action)
        if not invalid_actions:
            scores["action_compliant"] = 1
        else:
            notes.append(f"Invalid actions: {invalid_actions}")
    elif is_clarification:
        scores["action_compliant"] = 1  # clarification has no actions to check

    # --- Dimension 3: Correct response type ---
    expected = test_case["expected_type"]
    if expected == "commands" and is_commands:
        scores["correct_type"] = 1
    elif expected == "clarification" and is_clarification:
        scores["correct_type"] = 1
    else:
        notes.append(f"Wrong type: expected {expected}, got {'commands' if is_commands else 'clarification' if is_clarification else 'unknown'}")

    # --- Dimension 4: Quality ---
    if expected == "commands" and is_commands:
        expected_actions = set(test_case.get("expected_actions", []))
        found_actions = {cmd.get("action") for cmd in parsed if isinstance(cmd, dict)}
        if expected_actions and expected_actions.issubset(found_actions):
            scores["quality"] = 1.0
            # Check required fields
            for cmd in parsed:
                action = cmd.get("action", "")
                if action in ("move_forward", "move_backward") and "distance" not in cmd:
                    scores["quality"] = 0.5
                    notes.append(f"Missing distance in {action}")
                if action in ("turn_left", "turn_right") and "angle" not in cmd:
                    scores["quality"] = 0.5
                    notes.append(f"Missing angle in {action}")
        else:
            scores["quality"] = 0.5
            notes.append(f"Missing expected actions: {expected_actions - found_actions}")

    elif expected == "clarification" and is_clarification:
        reason = parsed.get("reason", "")
        if len(reason) > 10 and reason != "<specific description of what information is missing>":
            scores["quality"] = 1.0
        else:
            scores["quality"] = 0.5
            notes.append("Clarification reason is generic or placeholder")

    scores["total"] = (
        scores["json_valid"] +
        scores["action_compliant"] +
        scores["correct_type"] +
        scores["quality"]
    )

    return scores, notes


# =============================================================================
# MAIN EVALUATION LOOP
# =============================================================================

def evaluate_model(model, test_cases, system_prompt):
    """Run all test cases against a single model. Returns list of result dicts."""
    results = []
    for tc in test_cases:
        print(f"    [{tc['id']}] {tc['input'][:50]}...")
        response, elapsed = query_model(system_prompt, tc["input"])
        scores, notes = score_response(tc, response)
        results.append({
            "test_id": tc["id"],
            "category": tc["category"],
            "input": tc["input"],
            "response": response,
            "elapsed_s": elapsed,
            "scores": scores,
            "notes": notes
        })
        time.sleep(0.5)
    return results


def print_report(all_results):
    """Print and save a formatted comparison report."""
    lines = []
    lines.append("=" * 70)
    lines.append("ROBERTUN ROVER COMMAND — LLM MODEL EVALUATION REPORT")
    lines.append("=" * 70)

    # Summary table
    lines.append("\n--- SUMMARY (scores out of 4.0) ---\n")
    header = f"{'Model':<30} {'Precise':>8} {'Ambig':>8} {'Invalid':>8} {'TOTAL':>8}"
    lines.append(header)
    lines.append("-" * 65)

    for model_name, results in all_results.items():
        precise = [r for r in results if r["category"] == "precise"]
        ambiguous = [r for r in results if r["category"] == "ambiguous"]
        invalid = [r for r in results if r["category"] == "invalid"]

        def avg(group):
            if not group:
                return 0
            return sum(r["scores"]["total"] for r in group) / len(group)

        total = sum(r["scores"]["total"] for r in results)
        max_total = len(results) * 4

        lines.append(
            f"{model_name:<30} {avg(precise):>8.2f} {avg(ambiguous):>8.2f} "
            f"{avg(invalid):>8.2f} {total:>6.1f}/{max_total}"
        )

    # Detailed results per model
    for model_name, results in all_results.items():
        lines.append(f"\n{'=' * 70}")
        lines.append(f"MODEL: {model_name}")
        lines.append(f"{'=' * 70}")
        for r in results:
            s = r["scores"]
            status = "✓" if s["total"] >= 3.5 else "~" if s["total"] >= 2.0 else "✗"
            lines.append(
                f"  {status} [{r['test_id']}] {r['category'].upper():<10} "
                f"score={s['total']:.1f}/4 "
                f"time={r.get('elapsed_s', 0):.2f}s "
                f"(json={s['json_valid']} action={s['action_compliant']} "
                f"type={s['correct_type']} quality={s['quality']})"
            )
            if r["notes"]:
                lines.append(f"       Notes: {'; '.join(r['notes'])}")
            lines.append(f"       Input:    {r['input']}")
            lines.append(f"       Response: {r['response'][:120]}{'...' if len(r['response']) > 120 else ''}")

    report = "\n".join(lines)
    print(report)

    with open(REPORT_FILE, "w") as f:
        f.write(report)
    print(f"\nReport saved to {REPORT_FILE}")


# =============================================================================
# ENTRY POINT
# =============================================================================

def main():
    # Clean GPU memory and verify device availability at startup
    print("=== Initial GPU cleanup and verification ===")
    os.system("/home/talos/cleanup_gpu.sh > /dev/null 2>&1")
    time.sleep(10)

    def gpu_available():
        result = subprocess.run(
            ["/home/talos/github/llama.cpp/build/bin/llama-cli", "--list-devices"],
            capture_output=True, text=True
        )
        return "CUDA" in result.stdout

    if not gpu_available():
        print("WARNING: GPU not available after initial cleanup, retrying...")
        os.system("/home/talos/cleanup_gpu.sh > /dev/null 2>&1")
        time.sleep(5)
        if not gpu_available():
            print("ERROR: GPU unavailable — reboot required before running evaluation")
            sys.exit(1)

    print("GPU available. Proceeding with evaluation.")

    # Warm up sudo credentials before the long evaluation starts
    # This prevents the cleanup script from failing silently due to password timeout
    print("Requesting sudo credentials upfront (needed for GPU cleanup between models)...")
    ret = os.system("sudo -v")
    if ret != 0:
        print("ERROR: sudo credentials failed — cannot proceed")
        sys.exit(1)
    print("Sudo credentials confirmed.\n")

    # Keep sudo alive in background throughout the evaluation (refreshes every 5 min)
    def sudo_keepalive():
        while True:
            subprocess.run(
                ["sudo", "-v"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            time.sleep(300)

    import threading
    keepalive = threading.Thread(target=sudo_keepalive, daemon=True)
    keepalive.start()

    # Load test cases
    with open(TEST_CASES_FILE) as f:
        data = json.load(f)
    system_prompt = data["system_prompt"]
    test_cases = data["test_cases"]

    print(f"Loaded {len(test_cases)} test cases")
    print(f"Testing {len(MODELS)} models\n")

    all_results = {}

    for model in MODELS:
        print(f"\n{'=' * 50}")
        print(f"Testing: {model['name']}")
        print(f"{'=' * 50}")

        # Start server
        proc = start_server(model)
        if not wait_for_server(timeout=90):
            print(f"  ERROR: Server did not start within 90 seconds — skipping")
            stop_server(proc)
            continue

        print(f"  Server ready. Warming up model...")
        # Send a warmup query to initialize CUDA context — not scored
        _warmup, _wt = query_model(system_prompt, "move forward 1 meter")
        print(f"  Warmup done ({_wt:.1f}s). Running {len(test_cases)} test cases...")

        # Run evaluation
        results = evaluate_model(model, test_cases, system_prompt)
        all_results[model["name"]] = results

        # Stop server and clean GPU memory before next model
        print(f"  Stopping server...")
        stop_server(proc)
        print(f"  Cleaning GPU memory...")
        os.system("/home/talos/cleanup_gpu.sh > /dev/null 2>&1")
        time.sleep(10)
        # Diagnostic: show CMA state after cleanup
        with open("/proc/meminfo") as f:
            for line in f:
                if "Cma" in line:
                    print(f"    {line.strip()}")
        # Diagnostic: show orphan handles
        try:
            import subprocess as _sp
            _r = _sp.run(
                ["sudo", "cat", "/sys/kernel/debug/nvmap/iovmm/orphan_handles"],
                capture_output=True, text=True, timeout=5
            )
            _lines = _r.stdout.strip().split("\n")
            if len(_lines) > 1:
                print(f"    WARNING: {len(_lines)-1} orphan handle(s) detected")
            else:
                print(f"    Orphan handles: clean")
        except Exception as _e:
            print(f"    Orphan handles: could not read ({_e})")

        # Verify GPU is available after cleanup
        def gpu_available():
            result = subprocess.run(
                ["/home/talos/github/llama.cpp/build/bin/llama-cli", "--list-devices"],
                capture_output=True, text=True
            )
            return "CUDA" in result.stdout

        if gpu_available():
            print(f"  GPU available.")
        else:
            print(f"  WARNING: GPU check failed — continuing anyway (module stays loaded)")

        # Save intermediate results
        with open(RESULTS_FILE, "w") as f:
            json.dump(all_results, f, indent=2)
        print(f"  Done. Results saved.")

    # Print final report
    print("\n")
    print_report(all_results)


if __name__ == "__main__":
    main()
