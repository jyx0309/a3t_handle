#!/usr/bin/env python3
"""Offline CAD/SDK comparison, never imports the SDK or commands hardware.

Assumes SDK joint1..6 coordinates equal URDF coordinates and base gravity is -Z.
FK disagreement invalidates that assumption; do not deploy these torques.
Requires numpy. Finger joints default to zero opening (symmetric mimic).
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


def vector(text):
    value = np.array([float(x) for x in text.split()])
    if value.shape != (3,) or not np.isfinite(value).all():
        raise ValueError("Expected finite xyz/rpy vector")
    return value


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * skew @ skew


def origin(element):
    result = np.eye(4)
    if element is not None:
        r, p, y = vector(element.get("rpy", "0 0 0"))
        result[:3, :3] = rotation([0, 0, 1], y) @ rotation([0, 1, 0], p) @ rotation([1, 0, 0], r)
        result[:3, 3] = vector(element.get("xyz", "0 0 0"))
    return result


class Model:
    def __init__(self, path):
        root = ET.parse(path).getroot()
        self.links = {}
        for link in root.findall("link"):
            inertial = link.find("inertial")
            if inertial is None:
                raise ValueError(f"Missing inertial: {link.get('name')}")
            mass = float(inertial.find("mass").get("value"))
            if not np.isfinite(mass) or mass < 0:
                raise ValueError("Invalid mass")
            self.links[link.get("name")] = (mass, origin(inertial.find("origin"))[:3, 3])
        self.joints = root.findall("joint")
        children = {j.find("child").get("link") for j in self.joints}
        roots = set(self.links) - children
        if len(roots) != 1:
            raise ValueError("Expected one rooted URDF tree")
        self.root = roots.pop()

    def evaluate(self, q, finger=0.0, gravity=(0, 0, -9.81)):
        q = np.asarray(q, dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all() or not 0 <= finger <= 0.037:
            raise ValueError("Expected six finite joint angles and finger in [0,.037]")
        values = {f"joint{i+1}": value for i, value in enumerate(q)}
        values["gripper_right_joint"] = finger
        transforms = {self.root: np.eye(4)}
        ancestors = {self.root: []}
        axes = {}
        pending = list(self.joints)
        while pending:
            progressed = False
            for joint in pending[:]:
                parent = joint.find("parent").get("link")
                if parent not in transforms:
                    continue
                name, kind = joint.get("name"), joint.get("type")
                child = joint.find("child").get("link")
                transform = transforms[parent] @ origin(joint.find("origin"))
                chain = list(ancestors[parent])
                if kind != "fixed":
                    axis = vector(joint.find("axis").get("xyz"))
                    if np.linalg.norm(axis) < 1e-12:
                        raise ValueError("Zero joint axis")
                    axis /= np.linalg.norm(axis)
                    mimic = joint.find("mimic")
                    value = (values[mimic.get("joint")] * float(mimic.get("multiplier", "1")) +
                             float(mimic.get("offset", "0"))) if mimic is not None else values[name]
                    if kind in ("revolute", "continuous"):
                        axes[name] = (transform[:3, 3].copy(), transform[:3, :3] @ axis)
                        chain.append(name)
                        transform[:3, :3] = transform[:3, :3] @ rotation(axis, value)
                    elif kind == "prismatic":
                        transform[:3, 3] += transform[:3, :3] @ (axis * value)
                    else:
                        raise ValueError(f"Unsupported joint {kind}")
                transforms[child], ancestors[child] = transform, chain
                pending.remove(joint)
                progressed = True
            if not progressed:
                raise ValueError("Invalid/disconnected joint tree")
        parts, potential = {}, 0.0
        for name, (mass, com) in self.links.items():
            t = transforms[name]
            point = t[:3, :3] @ com + t[:3, 3]
            force = mass * np.asarray(gravity)
            potential -= float(force @ point)
            torque = np.zeros(6)
            for i in range(6):
                joint_name = f"joint{i+1}"
                if joint_name in ancestors[name]:
                    pivot, axis = axes[joint_name]
                    torque[i] = -float(np.cross(axis, point - pivot) @ force)
            parts[name] = torque
        return sum(parts.values()), transforms, parts, potential


def compare(model, session, stride=20, finger=0.0):
    rows, state, pending, command = [], None, None, None
    count, malformed = 0, 0
    with (Path(session) / "diagnostics.jsonl").open() as stream:
        for line in stream:
            try:
                r = json.loads(line)
            except ValueError:
                malformed += 1
                continue
            if r.get("type") == "low_state":
                state = r if r.get("data_valid") and r.get("result") == 1 else None
            elif r.get("type") == "control_math":
                pending = (r, state)
                command = None
            elif r.get("type") == "mit_input":
                command = r
            elif r.get("type") == "mit_result" and pending:
                math, sample = pending
                pending = None
                if (r.get("result") != 1 or r.get("frame") != math.get("next_frame") or
                        not math.get("gravity_only_unlimited") or not sample or not command or
                        command.get("frame") != r.get("frame") or
                        len(command.get("kp", [])) != 6 or len(command.get("kd", [])) != 6 or
                        any(v != 0 for v in command["kp"] + command["kd"]) or
                        not np.allclose(command["tau_ff"], math["gravity"], rtol=0, atol=1e-6) or
                        not 0 <= math["monotonic_ms"] - sample["monotonic_ms"] < 50):
                    continue
                count += 1
                if (count - 1) % stride:
                    continue
                q = sample["state"]["q"]
                torque, frames, parts, _ = model.evaluate(q, finger)
                payload = sum(v for k, v in parts.items() if k.startswith("gripper_"))
                pose = np.array(math["pose"])
                x, y, z, w = pose[3:] / np.linalg.norm(pose[3:])
                sdk_r = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                                  [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                                  [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
                angle = np.arccos(np.clip((np.trace(frames["link6"][:3,:3].T @ sdk_r)-1)/2, -1, 1))
                rows.append(dict(timestamp=math["timestamp"], q=q, tool=math.get("tool"),
                    sdk=math.get("sdk_gravity", math["gravity"]), urdf=torque.tolist(), gripper=payload.tolist(),
                    bare=(torque-payload).tolist(),
                    fk_position_error_m=float(np.linalg.norm(frames["link6"][:3,3]-pose[:3])),
                    fk_orientation_error_rad=float(angle)))
    report = dict(session=str(session), eligible_frames=count, sampled_frames=len(rows), malformed_lines=malformed,
                  assumptions="q_URDF=q_SDK; base gravity=(0,0,-9.81); SDK pose compared to link6; no camera added",
                  finger_position_m=finger, link_masses_kg={k: v[0] for k,v in model.links.items()})
    if rows:
        error = np.array([x["urdf"] for x in rows])-np.array([x["sdk"] for x in rows])
        bare_error = np.array([x["bare"] for x in rows])-np.array([x["sdk"] for x in rows])
        report.update(mean_urdf_minus_sdk_nm=error.mean(axis=0).tolist(),
                      max_abs_urdf_minus_sdk_nm=np.abs(error).max(axis=0).tolist(),
                      rmse_urdf_minus_sdk_nm=np.sqrt((error**2).mean(axis=0)).tolist(),
                      rmse_bare_minus_sdk_nm=np.sqrt((bare_error**2).mean(axis=0)).tolist(),
                      max_fk_position_error_m=max(x["fk_position_error_m"] for x in rows),
                      max_fk_orientation_error_rad=max(x["fk_orientation_error_rad"] for x in rows),
                      joint_span_rad=np.ptp([x["q"] for x in rows], axis=0).tolist(),
                      tool_indices=sorted(set(x["tool"] for x in rows)), first=rows[0], last=rows[-1])
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf", type=Path)
    parser.add_argument("session", type=Path)
    parser.add_argument("--stride", type=int, default=20)
    parser.add_argument("--finger", type=float, default=0.0)
    args = parser.parse_args()
    if args.stride < 1:
        parser.error("stride must be positive")
    result = compare(Model(args.urdf), args.session, args.stride, args.finger)
    result["urdf_sha256"] = hashlib.sha256(args.urdf.read_bytes()).hexdigest()
    print(json.dumps(result, ensure_ascii=False, indent=2))
