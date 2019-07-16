# Rigid body dynamics from URDF


### Dependencies
* CMake
```
sudo apt-get install cmake
```
* Boost for unit-testing and some asserts
```
sudo apt-get install libboost-all-dev
```
* URDFDOM for URDF parser
```
sudo apt-get install liburdfdom-dev
```
* CASADI >= 3.4.4 (with pkg-config support)

* Pinocchio (with Casadi interface - needs pkg-config support) from branch 'devel' - commit 607bab825ae37749034f2da11d16e3b883670f0b
```
# Clone Pinocchio's repository (devel branch)
git clone -b devel https://github.com/stack-of-tasks/pinocchio.git
# Checkout the commit I have been using
cd pinocchio
git checkout 607bab825ae37749034f2da11d16e3b883670f0b
# Build from source
...
```
* C++11 (for randomConfiguration)

### TODO-list
* [ ] Add Recorder as submodule and generate MATLAB code for functions (aba, rnea, ...)
