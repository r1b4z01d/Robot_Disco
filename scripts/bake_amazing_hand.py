"""Bake the AmazingHand MuJoCo model (AH_Right) into a static URDF xacro at the as-exported CAD pose (keyframe 'zero').
usage: bake_hand.py <AH_Right/scene.xml> <out_xacro> <out_mesh_dir>
"""
import os, re, shutil, sys, numpy as np, mujoco
scene, out_xacro, out_dir = sys.argv[1:4]
m = mujoco.MjModel.from_xml_path(scene); d = mujoco.MjData(m)
# The MJCF is an open tree: the finger link chains (ball joints, rods, pins) have NO loop-closure
# constraints (additional.xml is empty), so stepping physics lets them dangle. Use the exported CAD
# assembly pose instead: the "zero" keyframe + forward kinematics, no simulation.
key = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "zero")
if key >= 0:
    mujoco.mj_resetDataKeyframe(m, d, key)
mujoco.mj_forward(m, d)
root = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "r_wrist_interface")
Rr, pr = d.xmat[root].reshape(3, 3), d.xpos[root]
def under_root(b):
    while b > 0:
        if b == root: return True
        b = m.body_parentid[b]
    return False
# mesh name -> file from the xml (robot.xml next to scene.xml)
xml = open(os.path.join(os.path.dirname(scene), "robot.xml")).read()
files = {}
for tag in re.findall(r"<mesh [^>]+>", xml):
    f = re.search(r'file="([^"]+)"', tag).group(1); n = re.search(r'name="([^"]+)"', tag)
    files[n.group(1) if n else os.path.splitext(os.path.basename(f))[0]] = f
os.makedirs(out_dir, exist_ok=True)
def rpy(R):
    sy = -R[2, 0]; cy = np.sqrt(max(0.0, 1 - sy * sy))
    if cy > 1e-8: return np.arctan2(R[2, 1], R[2, 2]), np.arcsin(np.clip(sy, -1, 1)), np.arctan2(R[1, 0], R[0, 0])
    return np.arctan2(-R[1, 2], R[1, 1]), np.arcsin(np.clip(sy, -1, 1)), 0.0
visuals, used = [], set()
for g in range(m.ngeom):
    if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH or not under_root(m.geom_bodyid[g]): continue
    if m.geom_contype[g] != 0 or m.geom_conaffinity[g] != 0:   # keep the visual class only
        if m.geom_group[g] != 2: continue
    mesh = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, m.geom_dataid[g])
    f = files.get(mesh) or (mesh + ".stl")
    src = os.path.join(os.path.dirname(scene), "assets", f)
    if not os.path.exists(src): continue
    dst = os.path.basename(f).lower(); shutil.copy(src, os.path.join(out_dir, dst)); used.add(dst)
    # MuJoCo re-centres each mesh at load (centroid + principal axes, recorded in mesh_pos/mesh_quat)
    # and folds that into the geom pose; the STL file Foxglove loads is in the ORIGINAL frame, so undo it:
    # authored pose = geom pose o inverse(centring)  ->  R_file = R_g R_mq^T, p_file = p_g - R_file p_m
    mid = m.geom_dataid[g]
    Rmq = np.zeros(9); mujoco.mju_quat2Mat(Rmq, m.mesh_quat[mid]); Rmq = Rmq.reshape(3, 3)
    Rg, pg = d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g]
    Rg = Rg @ Rmq.T; pg = pg - Rg @ m.mesh_pos[mid]
    R, p = Rr.T @ Rg, Rr.T @ (pg - pr); r, pch, y = rpy(R)
    rgba = m.geom_rgba[g]
    visuals.append(f'''      <visual>
        <origin xyz="{p[0]:.5f} {p[1]:.5f} {p[2]:.5f}" rpy="{r:.5f} {pch:.5f} {y:.5f}"/>
        <geometry><mesh filename="package://robot_disco/meshes/amazing_hand/{dst}"/></geometry>
        <material name="ah_{dst[:-4]}"><color rgba="{rgba[0]:.3f} {rgba[1]:.3f} {rgba[2]:.3f} 1"/></material>
      </visual>''')
with open(out_xacro, "w") as f:
    f.write(f'''<?xml version="1.0"?>
<!-- AmazingHand (right) as a STATIC visual at the as-exported CAD pose (keyframe 'zero'), baked from the MuJoCo model
     (VLA/assets/amazing_hand/AH_Right, servos driven to the sim's open offsets and settled) by
     rd_ws/provision/../bake_hand.py. {len(visuals)} mesh visuals on one link; fingers do not animate.
     The hand root (r_wrist_interface) sits at the UR flange frame, as ur5e_lerobot/sim/combined_model.py
     attaches it to the MuJoCo attachment_site (= tool0). -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="amazing_hand" params="parent *origin">
    <joint name="${{parent}}_to_amazing_hand" type="fixed">
      <parent link="${{parent}}"/>
      <child link="amazing_hand"/>
      <xacro:insert_block name="origin"/>
    </joint>
    <link name="amazing_hand">
{chr(10).join(visuals)}
    </link>
  </xacro:macro>
</robot>
''')
print(f"baked {len(visuals)} visuals, {len(used)} meshes ({sum(os.path.getsize(os.path.join(out_dir, u)) for u in used)//1024} KB) -> {out_xacro}")
