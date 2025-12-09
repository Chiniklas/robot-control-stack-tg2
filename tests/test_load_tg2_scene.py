import argparse
import pathlib

import mujoco

try:
    import mujoco.viewer as mj_viewer
except ImportError:  # pragma: no cover - optional viewer dependency
    mj_viewer = None


def _scene_path() -> pathlib.Path:
    return (
        pathlib.Path(__file__).resolve().parent.parent
        / "assets"
        / "scenes"
        / "tg2_empty_world"
        / "scene.xml"
    )


def test_load_tg2_scene():
    """Load the TG2 scene to verify the split files and asset paths are valid."""
    model = mujoco.MjModel.from_xml_path(str(_scene_path()))
    data = mujoco.MjData(model)

    # A minimal rollout ensures model/data are consistent
    mujoco.mj_step(model, data)

    # Basic sanity: robot bodies should be present
    assert model.nbody > 1


def main():
    parser = argparse.ArgumentParser(description="Load TG2 scene; optionally launch viewer.")
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Skip opening the Mujoco viewer (default: viewer opens).",
    )
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(_scene_path()))
    data = mujoco.MjData(model)

    if args.headless:
        mujoco.mj_step(model, data)
        print("TG2 scene loaded and stepped headlessly.")
    else:
        if mj_viewer is None:
            raise RuntimeError("mujoco.viewer is unavailable; install mujoco's viewer extras or run with --headless.")
        with mj_viewer.launch_passive(model, data) as viewer:
            print("Viewer running. Close the window to exit.")
            while viewer.is_running():
                mujoco.mj_step(model, data)
                viewer.sync()


if __name__ == "__main__":
    main()
