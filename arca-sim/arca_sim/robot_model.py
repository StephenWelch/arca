import mujoco as mj
from dm_control import mjcf
from dm_control.mjcf import export_with_assets
from mujoco import MjModel

def get_mujoco_model(fixed:bool=False)->MjModel:
    urdf_model = MjModel.from_xml_path("models/urdf/robot.urdf")
    mj.mj_saveLastXML("models/urdf/robot.xml", urdf_model)

    robot = mjcf.from_path("models/urdf/robot.xml")
    scene = mjcf.from_path("models/mjcf/scene.xml")

    # all just hacks to be compatible w/ onshape-to-robot
    base = robot.worldbody.add("body", name="base")
    base_geom = robot.worldbody.geom[0]
    base.add(base_geom.tag, type=base_geom.type, rgba=base_geom.rgba, mesh=base_geom.mesh)
    del robot.worldbody.geom[0]
    # if fixed:
        # robot.equality.add("weld", body1="base", relpose="0 0 0 1 0 0 0")
    robot.contact.add("exclude", name="exclude_1_body_0", body1="base", body2="r_thigh")
    robot.contact.add("exclude", name="exclude_1_body_1", body1="base", body2="l_thigh")
    
    # add actuators for left joints, then right
    # sorted_joints = sorted(robot.find_all("joint"), key=lambda j: j.name)
    # for joint in sorted_joints:
    for joint in robot.find_all("joint"):
        if "passive" not in joint.name:
            robot.actuator.add("motor", joint=joint.name, ctrlrange="-1 1")
        if "ankle" in joint.name:
            joint.damping = 0.01
        if "l_" in joint.name:
            joint.axis = [0, 0, -1]

    scene.option.timestep = 0.001
    scene.default.geom.contype = "0"
    scene.default.geom.solref = "0.005 1"
    # scene.compiler.angle = "radian"
    # scene.compiler.fitaabb = True
    # scene.compiler.fusestatic = True
    # scene.compiler.discardvisual = False
    scene.compiler.balanceinertia = True
    scene.compiler.inertiafromgeom = True
    attach_frame = scene.attach(robot)
    attach_frame.add("freejoint")

    mjcf.export_with_assets(scene, "models/mjcf", "compiled.xml")

    return MjModel.from_xml_path("models/mjcf/compiled.xml")
