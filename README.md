# EML 4842: Autonomous Vehicles 1/10th Vehicle Drivers
A repository for interfacing with the hardware onboard the AV 1/10th vehicle.

### Installation

Follow the [instructions for setup](https://av1tenth-docs.readthedocs.io/en/latest/gettingstarted/setup.html) before building this repository.

You will need to clone this repository into the ***src*** directory of a workspace. You can create the appropriate directories with:
```bash
mkdir -p ~/av1tenth_ws/src
```
***av1tenth_ws*** can be replaced with whatever name you want.

Then clone the github repository into the ***src*** directory of the workspace.
```bash
cd av1tenth_ws/src
```

```bash
git clone https://github.com/av-mae-uf/av1tenth.git
```

Build repository:

```bash
cd ~/av1tenth_ws
```

```bash
colcon build
```

Before trying to launch anything remember to source the workspace from the root directory (***~/av1tenth_ws*** in this case).
```bash
source install/setup.bash
```