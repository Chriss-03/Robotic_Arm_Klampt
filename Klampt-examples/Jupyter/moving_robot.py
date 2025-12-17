import time
import math

from klampt import WorldModel, vis
from klampt.math import so3, se3
from klampt.model import ik
from klampt.model import geometry as geom


ROBOT_FILE = "../data/robots/tx90l.rob"

FPS = 30.0          
STEPS_PER_SEG = 15
SAFE_Z = 1.3 



def build_world():
    world = WorldModel()

    robot = world.loadRobot(ROBOT_FILE)
    if robot is None or robot.numLinks() == 0:
        raise RuntimeError(f"Couldn't load robot from {ROBOT_FILE}")

    for link in robot.links:
        link.appearance().setColor(0.3, 0.3, 0.3, 1.0)

    q_home = [0.0, -1.2, 2.0, 0.0, 1.3, 0.0]
    q_home = (q_home + [0.0] * robot.numLinks())[:robot.numLinks()]
    robot.setConfig(q_home)

    ground = geom.box(
        4.0, 4.0, 0.02,
        R=so3.identity(),
        t=[0.0, 0.0, -0.01],
        world=world,
        name="ground",
    )
    ground.appearance().setColor(0.7, 0.5, 0.3, 1.0)

    table_w = 0.6
    table_d = 0.6
    table_h = 0.7
    top_th = 0.05
    leg_th = 0.05

    left_xy  = (0.7,  0.35)
    right_xy = (0.7, -0.35)

    def make_table(name_prefix, cx, cy):
        top_center_z = table_h + 0.5 * top_th
        top = geom.box(
            table_w, table_d, top_th,
            R=so3.identity(),
            t=[cx, cy, top_center_z],
            world=world,
            name=f"{name_prefix}_top",
        )
        top.appearance().setColor(0.1, 0.5, 0.1, 1.0)

        dx = 0.5 * table_w - 0.5 * leg_th
        dy = 0.5 * table_d - 0.5 * leg_th
        leg_center_z = 0.5 * table_h
        for i, (ox, oy) in enumerate([(-dx,-dy), (-dx,dy), (dx,-dy), (dx,dy)]):
            leg = geom.box(
                leg_th, leg_th, table_h,
                R=so3.identity(),
                t=[cx + ox, cy + oy, leg_center_z],
                world=world,
                name=f"{name_prefix}_leg_{i}",
            )
            leg.appearance().setColor(0.15, 0.07, 0.02, 1.0)

    make_table("left_table",  *left_xy)
    make_table("right_table", *right_xy)

    table_z_surface = table_h + top_th

    mug_w = mug_d = 0.05
    mug_h = 0.10
    offset = 0.12
    mugs = []
    rx, ry = right_xy
    colors = [
        (0.0, 0.6, 0.0, 1.0),
        (0.7, 0.0, 0.0, 1.0),
        (0.0, 0.6, 0.0, 1.0),
    ]
    for i, dx in enumerate([-offset, 0.0, offset]):
        center = [rx + dx, ry, table_z_surface + 0.5 * mug_h]
        mug = geom.box(
            mug_w, mug_d, mug_h,
            R=so3.identity(),
            t=center,
            world=world,
            name=f"mug_{i}",
            mass=0.1,
        )
        mug.appearance().setColor(*colors[i])
        mugs.append(mug)

    lx, ly = left_xy
    left_slots = [
        (lx - offset, ly),
        (lx,          ly),
        (lx + offset, ly),
    ]

    ee_link_index = 6 

    scene = {
        "left_slots": left_slots,
        "table_z_surface": table_z_surface,
        "mug_h": mug_h,
        "mugs": mugs,
        "ee_link_index": ee_link_index,
        "q_home": q_home,
        "left_xy": left_xy,
        "right_xy": right_xy,
    }
    return world, scene



def solve_ik_position(robot, ee_index, target_pos, tol=1e-3):
    ee_link = robot.link(ee_index)
    obj = ik.objective(ee_link, local=[0, 0, 0], world=target_pos)

    solver = ik.solver(obj)
    solver.setMaxIters(200)
    solver.setTolerance(tol)
    ok = solver.solve()

    res_vec = solver.getResidual()
    res = math.sqrt(sum(r*r for r in res_vec))
    if not ok:
        print("   IK warn: residual", res, "for target", target_pos)

    return robot.getConfig(), res


def make_attachment(robot, ee_index, obj):
    ee_link = robot.link(ee_index)
    R_link, t_link = ee_link.getTransform()
    R_obj, t_obj = obj.getTransform()
    T_rel = se3.mul(se3.inv((R_link, t_link)), (R_obj, t_obj))
    return {"obj": obj, "link_index": ee_index, "local_offset": T_rel}


def update_attachment(robot, attach):
    ee_link = robot.link(attach["link_index"])
    R_link, t_link = ee_link.getTransform()
    R_obj, t_obj = se3.mul((R_link, t_link), attach["local_offset"])
    attach["obj"].setTransform(R_obj, t_obj)


def move_robot_smooth(robot, q_target, attach=None):
    q_start = robot.getConfig()
    dt = 1.0 / FPS

    for s in range(1, STEPS_PER_SEG + 1):
        u = s / float(STEPS_PER_SEG)
        q = [(1 - u) * qi + u * qj for qi, qj in zip(q_start, q_target)]
        robot.setConfig(q)
        if attach is not None:
            update_attachment(robot, attach)
        vis.update()
        time.sleep(dt)


def go_to_ee_point(world, scene, target_pos, attach=None):
    robot = world.robot(0)
    ee = scene["ee_link_index"]

    q_sol, res = solve_ik_position(robot, ee, target_pos)
    if q_sol is None:
        print("IK failed for", target_pos)
        return False

    move_robot_smooth(robot, q_sol, attach=attach)
    return True



def pick_and_place_mug(world, scene, mug, place_xy):
    robot = world.robot(0)
    ee   = scene["ee_link_index"]
    table_z = scene["table_z_surface"]
    mug_h = scene["mug_h"]

    move_robot_smooth(robot, scene["q_home"])

    R_mug, t_mug = mug.getTransform()
    px, py = place_xy

    high_base   = [0.4, 0.0, SAFE_Z]   
    high_mug    = [t_mug[0], t_mug[1], SAFE_Z] 
    high_place  = [px,       py,       SAFE_Z] 
    park_high   = [0.4, 0.0, SAFE_Z] 

    grasp_pos   = [t_mug[0], t_mug[1], t_mug[2] + 0.02]
    place_down  = [px, py, table_z + 0.5 * mug_h + 0.01]

    if not go_to_ee_point(world, scene, high_base):
        return

    if not go_to_ee_point(world, scene, high_mug):
        return

    if not go_to_ee_point(world, scene, grasp_pos):
        return

    attach = make_attachment(robot, ee, mug)

    if not go_to_ee_point(world, scene, high_mug, attach=attach):
        return

    if not go_to_ee_point(world, scene, high_place, attach=attach):
        return

    if not go_to_ee_point(world, scene, place_down, attach=attach):
        return

    attach = None
    if not go_to_ee_point(world, scene, high_place):
        return

    if not go_to_ee_point(world, scene, high_base):
        return

    go_to_ee_point(world, scene, park_high)




def demo_pick_and_place():
    world, scene = build_world()
    robot = world.robot(0)

    vis.init("IPython")
    vis.add("world", world)
    vis.show()

    robot.setConfig(scene["q_home"])
    vis.update()
    time.sleep(0.5)

    for mug, slot in zip(scene["mugs"], scene["left_slots"]):
        print("Moving", mug.getName(), "to slot", slot)
        pick_and_place_mug(world, scene, mug, slot)
        time.sleep(0.5)

    print("Done.")
    return world, scene


world, scene = demo_pick_and_place()
