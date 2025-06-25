import argparse
import zenoh
from arca_sim.controller import Controller
from arca_sim.sim2 import MujocoSim
from arca_sim.state import LowLevelState



def main(args):
    z = zenoh.open(zenoh.Config())
    controller = Controller()
    sim = MujocoSim(args.model, args.viewer)

    ll_state = sim.reset()
    while sim.is_running():
        ll_cmd = controller.update(ll_state)
        ll_state = sim.step(ll_cmd)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=str, choices=["full", "reduced"], default="full")
    parser.add_argument("--type", type=str, choices=["sim", "real"], default="sim")
    parser.add_argument("--viewer", action="store_true")
    args = parser.parse_args()
    main(args)