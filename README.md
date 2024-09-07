# iLQR_SVR (iLQR with State Vector Reduction)
Russell et al. ~WAFR 2024: Online state vector reduction during model predictive control with gradient-based trajectory optimisation.

Model predictive control using trajectory optimisation struggles to extend to high-dimensional systems, such as manipulation in clutter or manipulation of soft body objects. Dimensionality reduction is one method of speeding up trajectory optimisation to make closed-loop MPC feasible on these high-dimensional systems. This repository proposes a method of **dynamically** changing the size of the state vector **online** during MPC. The advantage of this is that optimisation is not considering the full system state at all times, which can considerably reduce the time taken for optimisation.

How do we determine which Degrees of freedom (DoFs) to optimsie over and which to ignore? Our implementation is tailored to the popular trajectory optimisation algorithm [iLQR](https://homes.cs.washington.edu/~todorov/papers/TassaIROS12.pdf). In iLQR, state-feedback gain matrices $K$ are computed. These matrices tell us, depending on the deviation of a certain DoF from it's nominal value, how much of a control modification to make. Based on this information, we can determine what DoFs are more important than others.

To determine when a Dof should be re-added to the set of states we optimise over, we use a simple random resampling scheme. If a Dof is resampled, and it is deemed unimportant, it is removed after one MPC iteration.


## Video
TODO - Create a video summary and upload to youtube.

## Dependencies
### [MuJoCo 2.32](http://www.mujoco.org/) or newer
This repository uses a custom fork of MuJoCo (simply for the access to one private function - please 
see this [issue](https://github.com/google-deepmind/mujoco/issues/1453)).

As such you need to git clone my custom fork and then build from source.

```
git clone git@github.com:DMackRus/mujoco.git mujoco_temp
cd mujoco_temp
mkdir build
cd build
cmake ..
cmake --build .
cmake .. -DCMAKE_INSTALL_PREFIX="~/mujoco"
cmake --install .
echo export MJ_HOME='"'$(pwd)/mujoco'"' >> ~/.bashrc
```

These commands also set an environment variable "MJ_HOME" for CMake, if you are installing
MuJoCo differently, remember to set this variable.

### [Eigen 3](https://eigen.tuxfamily.org/index.php?title=Main_Page)
Eigen is used for matrix computations in trajectory optimisation. Download and install it
with the following command:

``` 
sudo apt install -y libeigen3-dev
```

### [YAML](https://github.com/jbeder/yaml-cpp)
This repository uses YAML for configuration files. Install YAML with the following command.
```
sudo apt install -y libyaml-cpp-dev
```

### [GLFW](https://www.glfw.org/)
GLFW is used for visualisation. Download with the following command.
```
sudo apt install -y libglfw3 libglfw3-dev
```

## Container
If you have singularity installed, you can use this 
[singularity container](https://github.com/DMackRus/Apptainer_TrajOptKP) which has all the dependancies 
installation and setup.

## Usage
-  Compile with CMake
-  run the executable
-  Setup the config file (defualt.yaml) depending on what you want to do. Changing task etc.
-  Setup the Task YAML (push_moderate_clutter.yaml) to change task sepcific settings (opt horizon, slowdown factor, parametrisation of iLQR_SVR)

## TODO
- Add K matrix feedback to MPC.
- Fix init controls for soft tasks. Make it automatic rather than manual.
- Fix MuJoCo build and plugins so people can automatically use latest MuJoCo release.
- Improve readme documentation
- Add media to Github
- Better residuals for push soft task

## Citing
TODO - Add citation infomation.
