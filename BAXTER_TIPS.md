# Using the Baxter

## Calibration 

Launch kinect :
```bash
roslaunch freenect_launch freenect.launch rgb_frame_id:=camera_rgb_optical_frame depth_frame_id:=camera_depth_optical_frame
```

Launch calibration : 
```bash
roslaunch baxter_kinect_calibration baxter_bundle_calibrate_xtion.launch
```

Look at the results :

```bash
rosrun tf tf_echo /torso /camera_link
```

```
Results for camera infront:

- Translation: [1.4, 0.03, 0.58]
- Rotation: in Quaternion [-0.424, -0.026, 0.905, -0.010]
            in RPY (radian) [-0.061, 0.875, 3.136]
            in RPY (degree) [-3.481, 50.162, 179.682]
```

Once we have the matrix transformation from tf tf_echo, we copy the translation and rotation part and publish a static transformation :
```bash
rosrun tf static_transform_publisher 1.4 0.03 0.58 -0.424 -0.026 0.905 -0.010 /world /camera_link 100
```

You can see the feedbacks in rviz and ajust the translations.

Update the `camera_link_pose` in the controller launch file when the transformation is good enough.


## Running

On branch `master`



