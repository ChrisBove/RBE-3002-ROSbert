# RBE-3002-ROSbert
Team ROSbert repo for lab code

## Setup

```
cd ~/catkin_ws/src/
git clone https://github.com/ChrisBove/RBE-3002-ROSbert.git
cd ..
catkin_make
source devel/setup.bash
```

## Full setup
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
#bashrc stuff
echo source ~/catkin_ws/devel/setup.bash >> ~/.bashrc
echo alias rbhome="'cd ~/catkin_ws/src/RBE-3002-ROSbert/'" >> ~/.bashrc
echo alias rbmake="'catkin_make install -DCMAKE_INSTALL_PREFIX:PATH=~/catkin_ws/install -C ~/catkin_ws -DCMAKE_BUILD_TYPE=Release'" >> ~/.bashrc
echo alias rbeclipse="'catkin_make --force-cmake -G\"Eclipse CDT4 - Unix Makefiles\" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j8 -C ~/catkin_ws'" >> ~/.bashrc
#workspace setup
cd ~/catkin_ws/src
wstool init
wstool set RBE-3002-ROSbert https://github.com/ChrisBove/RBE-3002-ROSbert --git
. ~/.bashrc
wstool update
. ~/.bashrc
cd ~/catkin_ws/
catkin_make
. ~/.bashrc

```

##Aliases
```
echo source ~/catkin_ws/devel/setup.bash >> ~/.bashrc
echo alias rbhome="'cd ~/catkin_ws/src/RBE-3002-ROSbert/'" >> ~/.bashrc
echo alias rbmake="'catkin_make install -DCMAKE_INSTALL_PREFIX:PATH=~/catkin_ws/install -C ~/catkin_ws -DCMAKE_BUILD_TYPE=Release'" >> ~/.bashrc
echo alias rbeclipse="'catkin_make --force-cmake -G\"Eclipse CDT4 - Unix Makefiles\" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j8 -C ~/catkin_ws'" >> ~/.bashrc
```
