"""Run native OCR/importer fixtures through the existing Robot JSON runner.

The image fixtures contain printed block letters. They do not establish arbitrary
handwriting recognition or robustness on real photographs.
"""
from pathlib import Path
import argparse
from datetime import datetime, timezone
import json
import os
import shutil
import subprocess

import numpy as np
from PIL import Image

from check_robot_output import expand_rectangles, verify_path


HERE = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--importer", required=True, type=Path)
    parser.add_argument("--runner", type=Path, default=HERE.parents[1] / "robot_json_solution/build/robot_benchmark.exe")
    parser.add_argument("--ocr", choices=["windows", "tesseract"], default="windows")
    parser.add_argument("--dll-dir", type=Path, default=Path(r"D:\vcpkg\installed\x64-windows\bin"))
    args = parser.parse_args()
    manifest = json.loads((HERE / "fixtures/manifest.json").read_text(encoding="utf-8"))
    run_dir = HERE / "test_runs" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    run_dir.mkdir(parents=True)
    environment = os.environ.copy()
    environment["PATH"] = str(args.dll_dir) + os.pathsep + environment.get("PATH", "")
    evidence = {"run_directory": str(run_dir), "fixture_scope": manifest["description"], "ocr_backend": args.ocr,
        "importer": str(args.importer.resolve()), "runner": str(args.runner.resolve()), "checks": [], "failures": []}
    checks = 0

    def check(condition, message):
        nonlocal checks
        checks += 1
        if not condition:
            raise AssertionError(message)

    def invoke(command, label):
        completed = subprocess.run([str(part) for part in command], capture_output=True, text=True,
            encoding="utf-8", errors="replace", env=environment, timeout=120)
        (run_dir / f"{label}.log").write_text(completed.stdout + completed.stderr, encoding="utf-8")
        return completed

    def importer_command(image, output, geometry=None):
        command = [args.importer.resolve(), "--image", image, "--output", output,
            "--grid-width", manifest["grid_width"], "--grid-height", manifest["grid_height"],
            "--cell-size", manifest["cell_size"], "--start-cell", *manifest["start_cell"],
            "--goal-cell", *manifest["goal_cell"], "--headless", "--ocr", args.ocr]
        command += geometry or ["--scan"]
        return command

    fixtures = [
        ("multicolour_scan", ["--scan"]),
        ("multicolour_shadow", ["--scan"]),
        ("multicolour_perspective", ["--corners", *sum(manifest["photo_corners_tl_tr_br_bl"], [])]),
    ]
    for name, geometry in fixtures:
        try:
            output = run_dir / f"{name}.json"
            completed = invoke(importer_command(HERE / "fixtures" / f"{name}.png", output, geometry), name)
            check(completed.returncode == 0, f"Importer failed for {name}: {completed.stdout} {completed.stderr}")
            document = json.loads(output.read_text(encoding="utf-8"))
            map_id, map_data, blocked = expand_rectangles(document)
            check(document["width"] == manifest["grid_width"] and document["height"] == manifest["grid_height"], "Grid dimensions changed")
            check(document["cell_size"] == manifest["cell_size"], "Cell size changed")
            for probe_name, probe in manifest["probes"].items():
                check((tuple(probe["cell"]) in blocked) == probe["blocked"], f"{name}: occupancy probe {probe_name} failed")
            review = output.with_name(output.stem + "_review")
            report = json.loads((review / "report.json").read_text(encoding="utf-8"))
            check(report["accepted_labels"] == 3, f"{name}: expected three OBSTACLE labels, got {report}")
            check(report["unmatched_labels"] == 0, f"{name}: unmatched OBSTACLE label")
            check(len(report["selected_shapes"]) == 3, f"{name}: selected shape count differs")
            check(report["exported"] is True, f"{name}: report not exported")
            grid = np.asarray(Image.open(review / "grid.png").convert("L"))
            reconstructed = {(int(x), document["height"] - 1 - int(row)) for row, x in zip(*np.where(grid != 0))}
            check(reconstructed == blocked, f"{name}: rectangle JSON differs from saved occupancy grid")
            case = map_data["cases"][0]
            check(case["Ct"] == [(x + 0.5)*manifest["cell_size"] for x in manifest["start_cell"]], "Ct world center differs")
            check(case["n"] == [(x + 0.5)*manifest["cell_size"] for x in manifest["goal_cell"]], "n world center differs")
            fixture_record = {"fixture": name, "accepted_labels": report["accepted_labels"], "blocked_cells": len(blocked), "runner": {}}
            for configuration in ["E1", "E2", "E3"]:
                runner_dir = run_dir / f"{name}_{configuration}"
                completed = invoke([args.runner.resolve(), "--input", output, "--configuration", configuration,
                    "--output", runner_dir], f"{name}_{configuration}")
                check(completed.returncode == 0, f"Robot runner failed: {completed.stdout} {completed.stderr}")
                outputs = list(runner_dir.rglob("benchmark_result_*.json"))
                check(len(outputs) == 1, f"Expected one robot result: {outputs}")
                result = json.loads(outputs[0].read_text(encoding="utf-8"))
                fixture_record["runner"][configuration] = verify_path(result, document["width"], document["height"],
                    document["cell_size"], blocked, manifest["start_cell"], manifest["goal_cell"], 4 if configuration == "E3" else 8)
            evidence["checks"].append(fixture_record)
        except Exception as error:
            evidence["failures"].append({"fixture": name, "error": str(error)})

    try:
        unicode_directory = run_dir / "thư mục kiểm thử"
        unicode_directory.mkdir()
        unicode_image = unicode_directory / "bản đồ màu.png"
        unicode_output = unicode_directory / "kết quả.json"
        shutil.copyfile(HERE / "fixtures/multicolour_scan.png", unicode_image)
        completed = invoke(importer_command(unicode_image, unicode_output), "unicode_and_space_paths")
        check(completed.returncode == 0, f"Unicode/space paths failed: {completed.stdout} {completed.stderr}")
        document = json.loads(unicode_output.read_text(encoding="utf-8"))
        reference = json.loads((run_dir / "multicolour_scan.json").read_text(encoding="utf-8"))
        check(document == reference, "Unicode/space paths changed generated map JSON")
        unicode_report = json.loads((unicode_output.with_name("kết quả_review") / "report.json").read_text(encoding="utf-8"))
        check(unicode_report["source"] == str(unicode_image.resolve()), "Diagnostic source path lost Unicode characters")
        check(unicode_report["accepted_labels"] == 3 and unicode_report["exported"] is True,
            "Unicode/space path import did not recognize/export three labelled objects")
        evidence["checks"].append({"unicode_and_space_paths": True,
            "input": str(unicode_image), "output": str(unicode_output)})
    except Exception as error:
        evidence["failures"].append({"case": "unicode_and_space_paths", "error": str(error)})

    try:
        protected_image = run_dir / "protected_source.png"
        shutil.copyfile(HERE / "fixtures/multicolour_scan.png", protected_image)
        original_bytes = protected_image.read_bytes()
        completed = invoke(importer_command(protected_image, protected_image), "reject_overwrite_source")
        check(completed.returncode != 0, "Source image was accepted as JSON output destination")
        check(protected_image.read_bytes() == original_bytes, "Source image bytes changed after rejected overwrite")
        check(not protected_image.with_name("protected_source_review").exists(),
            "Source-overwrite rejection created diagnostics before rejecting")
        evidence["checks"].append({"rejected": "overwrite_source_image", "source_bytes_preserved": True})

        source_alias = run_dir / "protected_hard_link_alias.json"
        os.link(protected_image, source_alias)
        check(os.path.samefile(protected_image, source_alias), "Test hard-link alias does not share source identity")
        completed = invoke(importer_command(protected_image, source_alias), "reject_hard_link_alias")
        check(completed.returncode != 0, "Hard-link alias of source image was accepted as JSON output destination")
        check(protected_image.read_bytes() == original_bytes, "Source image bytes changed through rejected hard-link output")
        check(source_alias.read_bytes() == original_bytes, "Hard-link alias bytes changed after rejection")
        check(not source_alias.with_name("protected_hard_link_alias_review").exists(),
            "Hard-link rejection created diagnostics before rejecting")
        evidence["checks"].append({"rejected": "hard_link_alias_of_source", "source_bytes_preserved": True,
            "alias_bytes_preserved": True})
    except Exception as error:
        evidence["failures"].append({"case": "source_image_preservation", "error": str(error)})

    image = HERE / "fixtures/multicolour_scan.png"
    failure_cases = [
        ("width_too_small", image, ["--grid-width", 1]),
        ("nonpositive_cell_size", image, ["--cell-size", 0]),
        ("nonfinite_cell_size", image, ["--cell-size", "nan"]),
        ("unsafe_map_id", image, ["--map-id", "CON"]),
        ("fractional_endpoint", image, ["--start-cell", "1.5", 1]),
        ("outside_endpoint", image, ["--start-cell", -1, 0]),
        ("blocked_endpoint", image, ["--start-cell", *manifest["probes"]["blue_outline_interior"]["cell"]]),
        ("bad_coverage", image, ["--coverage", 1.1]),
        ("manual_headless_conflict", image, ["--manual"]),
        ("no_obstacle_label", HERE / "fixtures/no_obstacle_label.png", []),
        ("unclosed_object", HERE / "fixtures/open_outline.png", []),
        ("missing_image", HERE / "fixtures/does_not_exist.png", []),
    ]
    for name, source, extra in failure_cases:
        try:
            output = run_dir / f"reject_{name}.json"
            completed = invoke(importer_command(source, output) + extra, f"reject_{name}")
            check(completed.returncode != 0, f"Invalid import {name} succeeded")
            check(not output.exists(), f"Invalid import {name} wrote Robot JSON")
            evidence["checks"].append({"rejected": name})
        except Exception as error:
            evidence["failures"].append({"case": name, "error": str(error)})

    try:
        output = run_dir / "allowed_empty.json"
        completed = invoke(importer_command(HERE / "fixtures/no_obstacle_label.png", output) + ["--allow-empty"], "allowed_empty")
        check(completed.returncode == 0, "Explicit --allow-empty failed")
        document = json.loads(output.read_text(encoding="utf-8"))
        _, _, blocked = expand_rectangles(document)
        check(not blocked, "Unlabelled colored PARK became obstacle in empty map")
        evidence["checks"].append({"explicit_empty_map": True})
    except Exception as error:
        evidence["failures"].append({"case": "allowed_empty", "error": str(error)})

    evidence["assertion_count"] = checks
    evidence["passed"] = not evidence["failures"]
    (HERE / "acceptance_results.json").write_text(json.dumps(evidence, indent=2), encoding="utf-8")
    print(json.dumps(evidence, indent=2))
    return 0 if evidence["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
