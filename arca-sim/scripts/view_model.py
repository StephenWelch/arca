import argparse
import mujoco
import mujoco.viewer


def main(args):
    # Load the MuJoCo model from the specified XML file
    model = mujoco.MjModel.from_xml_path(f"models/{args.model}/mjcf/scene.xml")
    data = mujoco.MjData(model)

    def key_cb(keycode):
        if chr(keycode) == 'R':
            gantry_eq_id = model.equality("gantry").id
            data.eq_active[gantry_eq_id] ^= 1
            print("Releasing gantry")

    # Launch the viewer in passive mode
    with mujoco.viewer.launch_passive(model, data, key_callback=key_cb) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="View a MuJoCo model.")
    parser.add_argument(
        "--model",
        type=str,
        default="reduced",
        help="Name of a model directory under models/",
    )
    args = parser.parse_args()

    main(args)