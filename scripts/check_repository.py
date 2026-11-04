#!/usr/bin/env python3
"""Check repository contracts without ROS, network access or robot I/O."""

import json
import re
import sys
from pathlib import Path
from urllib.parse import unquote, urlsplit
from xml.etree import ElementTree


ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    errors = []

    def require(condition, message):
        if not condition:
            errors.append(message)

    def read_json(relative):
        try:
            return json.loads((ROOT / relative).read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            errors.append(f"{relative}: {exc}")
            return {}

    required = (
        "README.md", "docs/architecture.md", "docs/development.md",
        "docs/github-setup.md",
        "docs/mapping-capture.md", "config/examples/mapping-capture.json",
        "scripts/a2_capture.py", "scripts/inspect_mapping_bag.py",
        "docs/mapping-readiness.md", "third_party/mapping-candidate.repos",
        "scripts/package_capture_tools.py",
        "docs/a2-connection.md",
        "docs/design/A2-PRO_Utility_Tunnel_Navigation_v0.5.md",
        ".gitignore", ".dockerignore", ".gitattributes", ".editorconfig",
        ".github/workflows/repository-check.yml",
        ".github/ISSUE_TEMPLATE/bug_report.yml",
        ".github/ISSUE_TEMPLATE/feature_request.yml",
        ".github/pull_request_template.md", "docker/development/Dockerfile",
        "third_party/README.md", "tests/hil/report-template.md",
    )
    for relative in required:
        require((ROOT / relative).is_file(), f"Missing file: {relative}")

    plan = read_json("src/package-plan.json")
    packages = plan.get("packages", [])
    require(isinstance(packages, list) and bool(packages), "Package plan is empty or invalid")
    names = set()
    for package in packages if isinstance(packages, list) else []:
        if not isinstance(package, dict):
            errors.append("Each package entry must be an object")
            continue
        name = package.get("name", "")
        require(bool(re.fullmatch(r"a2_[a-z0-9_]+", name)), f"Invalid package name: {name}")
        require(name not in names, f"Duplicate package: {name}")
        names.add(name)
        require(package.get("path") == f"src/{name}", f"Invalid package path: {name}")
        status = package.get("status")
        require(status in {"planned", "in_progress", "implemented", "verified"},
                f"Invalid package status: {name}")
        require((ROOT / "src" / name / "README.md").is_file(), f"Missing package README: {name}")
        if status in {"implemented", "verified"}:
            try:
                declared = ElementTree.parse(ROOT / "src" / name / "package.xml").findtext("name")
                require(declared == name, f"package.xml name mismatch: {name}")
            except (OSError, ElementTree.ParseError) as exc:
                errors.append(f"{name}/package.xml: {exc}")

    config = read_json("config/examples/profiles.json")
    profiles = config.get("profiles", {})
    require(config.get("default_profile") == "mock", "Default profile must be mock")
    require(isinstance(profiles, dict) and set(profiles) ==
            {"mock", "replay", "robot_readonly", "robot_motion"}, "Profile names disagree")
    if isinstance(profiles, dict):
        for name in ("mock", "replay"):
            profile = profiles.get(name, {})
            for flag in ("hardware_access", "unitree_sdk_loaded", "motion_output"):
                require(profile.get(flag) is False, f"{name}.{flag} must be false")
        require(profiles.get("robot_readonly", {}).get("motion_output") is False,
                "Readonly profile must disable motion output")
        motion = profiles.get("robot_motion", {})
        require(motion.get("requires_local_motion_authorization") is True and
                motion.get("startup_motion_authorized") is False,
                "Robot motion must require new local authorization")

    release = read_json("deploy/templates/release-manifest.example.json")
    require(release.get("status") == "template_not_deployable" and
            release.get("approved_for_motion") is False,
            "Release example must remain an unapproved template")
    dependencies = read_json("third_party/dependencies.repos")
    require(isinstance(dependencies.get("repositories"), dict), "Invalid dependency manifest")
    candidate = read_json("third_party/mapping-candidate.repos")
    candidate_repositories = candidate.get("repositories")
    require(isinstance(candidate_repositories, dict) and bool(candidate_repositories),
            "Mapping candidate manifest must contain repositories")
    if isinstance(candidate_repositories, dict):
        for name, dependency in candidate_repositories.items():
            require(isinstance(dependency, dict) and dependency.get("type") == "git" and
                    isinstance(dependency.get("url"), str) and dependency["url"].startswith("https://") and
                    isinstance(dependency.get("version"), str) and
                    bool(re.fullmatch(r"[0-9a-f]{40}", dependency["version"])),
                    f"Candidate dependency must pin a full Git SHA: {name}")

    markdown = [ROOT / "README.md", ROOT / "third_party" / "README.md"]
    for directory in ("docs", "src", "tests", ".github"):
        markdown.extend((ROOT / directory).rglob("*.md"))
    for path in sorted(set(markdown)):
        if not path.is_file():
            continue
        text = path.read_text(encoding="utf-8")
        label = path.relative_to(ROOT)
        require(len(re.findall(r"^```", text, re.M)) % 2 == 0,
                f"{label}: unbalanced code fences")
        for block in re.findall(r"```json\n(.*?)\n```", text, re.S):
            try:
                json.loads(block)
            except ValueError as exc:
                errors.append(f"{label}: invalid JSON example: {exc}")
        prose = re.sub(r"^```[^\n]*\n.*?^```[^\n]*$", "", text, flags=re.M | re.S)
        for target in re.findall(r"\[[^\]]*\]\(([^)\s]+)\)", prose):
            url = urlsplit(target)
            if url.scheme or url.netloc or not url.path:
                continue
            require((path.parent / unquote(url.path)).exists(),
                    f"{label}: broken local link: {target}")

    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print(f"PASS: repository contracts, {len(names)} package entries, profiles, JSON and local links.")
    print("This does not verify ROS builds, robot software behavior, or A2 hardware.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
