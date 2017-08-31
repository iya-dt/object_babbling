# Install object_babbling

## Dependencies

The following libraries :
- boost 
- eigen3
- libb64
- opencv2 and opencv_nonfree (2.4.8)
- pcl (1.7.2) compiled with c++11


ROS Indigo [http://wiki.ros.org/indigo/Installation](http://wiki.ros.org/indigo/Installation).


The image processing library (object-babbling).
```bash
git clone https://github.com/robotsthatdream/image_processing.git
cd image_processing
git checkout object-babbling
mkdir build
cd build
cmake ..
sudo make -j8 install
```


The IAGMM library (object-babbling).
```bash
git clone https://github.com/LeniLeGoff/IAGMM_Lib.git
cd IAGMM_Lib
git checkout object-babbling
mkdir build
cd build
cmake ..
sudo make -j8 install
```


The cafer framework.
```bash
git clone https://github.com/robotsthatdream/cafer.git
```


The rgbd_utils from the babbling_arm_exp repository.
```bash
env GIT_SSL_NO_VERIFY=true git clone https://project.isir.upmc.fr/git/babbling_arm_exp
git config http.sslVerify "false"
```

### For the Baxter

### For the PR2
