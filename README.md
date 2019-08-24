# Da Vinci Code

Requires Python 2.7 and using our davinci0 machine.

Use the following core API:

- `dvrkArm.py`: an API wrapping around the ROS messages to get or set positions.

Other supporting files are `dvrkMotion.py` and `dvrkClothSim.py` for moving the
arms.

## Experimental Usage

Performing our experiments involves several steps, due to loading a separate
neural network (using a Python3 virtualenv). Roughly, the steps are:

1. Activate the robot via `roscore`, then (in a separate tab) run `./teleop` in
`~/catkin_ws`.

2. In another tab in `/catkin_ws/src/zivid-ros/zivid_samples/launch`, run the
camera capture script:

```
roslaunch zivid_samples sample_capture_py.launch 
```

3. In another tab in `dvrk_python/image_manip`, run `python capture_image.py`.

4. In yet another tab, *set up a new Python 3 virtualenv*, and run `python
load_net.py`.  See `image_manip/README.md` for details on Python3 integration.




## Quick Start

Here's a minimal working example. Put this set of code in `tests/test_01_positions.py`:

```python
import sys
sys.path.append('.')
import dvrkArm

if __name__ == "__main__":
    p = dvrkArm.dvrkArm('/PSM1')
    pose_frame = p.get_current_pose_frame()
    pose_rad = p.get_current_pose()
    pose_deg = p.get_current_pose(unit='deg')

    print('pose_frame:\n{}'.format(pose_frame))
    print('pose_rad: {}'.format(pose_rad))
    print('pose_deg: {}'.format(pose_deg))

    targ_pos = [0.04845971,  0.05464517, -0.12231114]
    targ_rot = [4.65831872, -0.69974499,  0.87412989]
    p.set_pose(pos=targ_pos, rot=targ_rot, unit='rad')
```

and run `python tests/test_01_positions.py`. This will print out pose
information about the dvrk arm, which can then be used to hard code some
actions. For example, above we show how to set the pose of the arm given
pre-computed `targ_pos` and `targ_rot`.


