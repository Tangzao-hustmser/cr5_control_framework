"""Asset consistency checks across URDF/USD/config/kinematics."""

from dataclasses import dataclass, asdict
from typing import Any, Dict, List, Tuple
import json
import os
import re
import yaml


@dataclass
class AssetIssue:
    level: str
    code: str
    message: str

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)



def _load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8-sig") as fh:
        return yaml.safe_load(fh)



def _read_text(path: str) -> str:
    with open(path, "r", encoding="utf-8", errors="ignore") as fh:
        return fh.read()



def _extract_links(urdf_text: str) -> List[str]:
    return re.findall(r"<link\s+name=\"([^\"]+)\"", urdf_text)



def _extract_joint_limits(urdf_text: str) -> Dict[str, Tuple[float, float]]:
    results: Dict[str, Tuple[float, float]] = {}
    for match in re.finditer(r"<joint\s+name=\"([^\"]+)\"[^>]*>([\s\S]*?)</joint>", urdf_text):
        name = match.group(1)
        block = match.group(2)
        limit_match = re.search(r"<limit[^>]*lower=\"([^\"]+)\"[^>]*upper=\"([^\"]+)\"", block)
        if limit_match:
            try:
                lower = float(limit_match.group(1))
                upper = float(limit_match.group(2))
                results[name] = (lower, upper)
            except ValueError:
                continue
    return results



def _extract_mesh_scales(urdf_text: str) -> List[Tuple[str, List[float]]]:
    scales: List[Tuple[str, List[float]]] = []
    for mesh in re.finditer(r"<mesh[^>]*filename=\"([^\"]+)\"[^>]*scale=\"([^\"]+)\"", urdf_text):
        path = mesh.group(1)
        scale_text = mesh.group(2)
        try:
            vals = [float(v) for v in scale_text.split()]
        except ValueError:
            vals = []
        scales.append((path, vals))
    return scales



def run_asset_consistency_check(
    project_root: str,
    robot_config_path: str,
    kinematics_config_path: str,
    report_output_path: str,
) -> Dict[str, Any]:
    issues: List[AssetIssue] = []

    robot_cfg = _load_yaml(robot_config_path)
    kin_cfg = _load_yaml(kinematics_config_path)

    urdf_rel = robot_cfg.get("robot", {}).get("urdf_path", "")
    usd_rel = robot_cfg.get("robot", {}).get("usd_path", "")

    urdf_path = os.path.join(project_root, urdf_rel)
    usd_path = os.path.join(project_root, usd_rel)

    if not os.path.exists(urdf_path):
        issues.append(AssetIssue("critical", "R3-E1801", f"URDF file missing: {urdf_path}"))
        urdf_text = ""
    else:
        urdf_text = _read_text(urdf_path)

    if not os.path.exists(usd_path):
        issues.append(AssetIssue("critical", "R3-E1801", f"USD file missing: {usd_path}"))

    links = _extract_links(urdf_text)
    urdf_limits = _extract_joint_limits(urdf_text)

    arm_joints = list(robot_cfg.get("action_space", {}).get("arm", {}).get("joints", []))
    gripper_joints = list(robot_cfg.get("action_space", {}).get("gripper", {}).get("joints", []))

    for joint in arm_joints + gripper_joints:
        if joint not in urdf_limits:
            issues.append(AssetIssue("critical", "R3-E1801", f"Joint not found in URDF limits: {joint}"))

    arm_cfg_limits = robot_cfg.get("action_space", {}).get("arm", {}).get("joint_limits", {})
    for name in arm_joints:
        if name in urdf_limits and name in arm_cfg_limits:
            cfg_lo, cfg_hi = float(arm_cfg_limits[name][0]), float(arm_cfg_limits[name][1])
            urdf_lo, urdf_hi = urdf_limits[name]
            if abs(cfg_lo - urdf_lo) > 1e-2 or abs(cfg_hi - urdf_hi) > 1e-2:
                issues.append(
                    AssetIssue(
                        "critical",
                        "R3-E1801",
                        f"Joint limit mismatch for {name}: cfg=({cfg_lo},{cfg_hi}) urdf=({urdf_lo},{urdf_hi})",
                    )
                )

    gripper_cfg_limits = robot_cfg.get("action_space", {}).get("gripper", {}).get("joint_limits", {})
    for name in gripper_joints:
        if name in urdf_limits and name in gripper_cfg_limits:
            cfg_lo, cfg_hi = float(gripper_cfg_limits[name][0]), float(gripper_cfg_limits[name][1])
            urdf_lo, urdf_hi = urdf_limits[name]
            if abs(cfg_lo - urdf_lo) > 1e-2 or abs(cfg_hi - urdf_hi) > 1e-2:
                issues.append(
                    AssetIssue(
                        "warning",
                        "R3-E1801",
                        f"Gripper limit mismatch for {name}: cfg=({cfg_lo},{cfg_hi}) urdf=({urdf_lo},{urdf_hi})",
                    )
                )

    cspace = list(kin_cfg.get("cspace", []))
    if cspace != arm_joints:
        issues.append(
            AssetIssue(
                "critical",
                "R3-E1801",
                f"kinematics cspace mismatch with robot arm joints. cspace={cspace} arm={arm_joints}",
            )
        )

    ee_link = str(kin_cfg.get("ee_link", ""))
    if ee_link and ee_link not in links:
        issues.append(AssetIssue("critical", "R3-E1801", f"ee_link not found in URDF links: {ee_link}"))

    root_link = str(kin_cfg.get("root_link", ""))
    if root_link and root_link not in links:
        issues.append(AssetIssue("critical", "R3-E1801", f"root_link not found in URDF links: {root_link}"))

    for mesh_path, scales in _extract_mesh_scales(urdf_text):
        if scales and any(abs(v) > 100.0 for v in scales):
            issues.append(
                AssetIssue(
                    "warning",
                    "R3-E1801",
                    f"Large mesh scale detected ({scales}) for mesh {mesh_path}; verify units consistency.",
                )
            )

    critical_count = sum(1 for issue in issues if issue.level == "critical")
    warning_count = sum(1 for issue in issues if issue.level == "warning")

    report = {
        "summary": {
            "critical": critical_count,
            "warning": warning_count,
            "status": "BLOCKED" if critical_count > 0 else ("DEGRADED" if warning_count > 0 else "NORMAL"),
        },
        "inputs": {
            "project_root": project_root,
            "robot_config_path": robot_config_path,
            "kinematics_config_path": kinematics_config_path,
            "urdf_path": urdf_path,
            "usd_path": usd_path,
        },
        "issues": [issue.to_dict() for issue in issues],
        "facts": {
            "arm_joint_count": len(arm_joints),
            "gripper_joint_count": len(gripper_joints),
            "urdf_joint_limit_count": len(urdf_limits),
            "urdf_link_count": len(links),
            "ee_link": ee_link,
        },
    }

    os.makedirs(os.path.dirname(report_output_path), exist_ok=True)
    with open(report_output_path, "w", encoding="utf-8") as fh:
        json.dump(report, fh, indent=2, ensure_ascii=False)

    return report
