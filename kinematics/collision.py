import numpy as np
from typing import Tuple
from kinematics.forward_kinematics import get_all_transforms
from kinematics.dh_params import DH6R

def check_capsule_collision(
    p1_start: np.ndarray, p1_end: np.ndarray, r1: float,
    p2_start: np.ndarray, p2_end: np.ndarray, r2: float
) -> bool:
    """Check if two capsules intersect. A capsule is a line segment padded by a radius."""
    u = p1_end - p1_start
    v = p2_end - p2_start
    w = p1_start - p2_start
    
    a = np.dot(u, u)
    b = np.dot(u, v)
    c = np.dot(v, v)
    d = np.dot(u, w)
    e = np.dot(v, w)
    
    D = a*c - b*b
    
    sc, sN, sD = D, D, D
    tc, tN, tD = D, D, D
    
    if D < 1e-8:
        sN = 0.0
        sD = 1.0
        tN = e
        tD = c
    else:
        sN = (b*e - c*d)
        tN = (a*e - b*d)
        if sN < 0.0:
            sN = 0.0
            tN = e
            tD = c
        elif sN > sD:
            sN = sD
            tN = e + b
            tD = c
            
    if tN < 0.0:
        tN = 0.0
        if -d < 0.0: sN = 0.0
        elif -d > a: sN = sD
        else:
            sN = -d
            sD = a
    elif tN > tD:
        tN = tD
        if (-d + b) < 0.0: sN = 0
        elif (-d + b) > a: sN = sD
        else:
            sN = (-d +  b)
            sD = a
            
    sc = 0.0 if abs(sN) < 1e-8 else sN / sD
    tc = 0.0 if abs(tN) < 1e-8 else tN / tD
    
    dP = w + (sc * u) - (tc * v)
    distance = np.linalg.norm(dP)
    
    return distance < (r1 + r2)

def check_trajectory_collision(
    trajectory: np.ndarray,
    dh_table: DH6R,
    link_radius: float = 0.04  # 40mm capsule radius — matches AR2 arm cross-section
) -> Tuple[bool, str]:
    """
    Check a full joint trajectory for physical collisions.
    Returns (is_valid, reason_string).
    """
    for step, q in enumerate(trajectory):
        q_6 = q[:6]
        transforms = get_all_transforms(q_6, dh_table)
        
        positions = [T[:3, 3] for T in transforms]
        
        # 1. Floor collision check (Z < 0)
        # Start checking after base link
        for i, pos in enumerate(positions[1:]):
            if pos[2] < (link_radius * 0.5):
                return False, f"Floor limit reached at step {step}, link {i+1} Z={pos[2]:.3f}"
                
        # 2. Self-collision check (skip adjacent links as they physically intersect)
        for i in range(len(positions) - 1):
            p1_start = positions[i]
            p1_end = positions[i+1]
            
            for j in range(i + 2, len(positions) - 1):
                p2_start = positions[j]
                p2_end = positions[j+1]
                
                if check_capsule_collision(p1_start, p1_end, link_radius, p2_start, p2_end, link_radius):
                    return False, f"Self-collision between link {i}-{i+1} and link {j}-{j+1} at step {step}"
                    
    return True, "Trajectory safely validated."
