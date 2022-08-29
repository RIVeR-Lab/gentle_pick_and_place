* installed the [Apriltags ROS package](http://wiki.ros.org/apriltag_ros)
  * Takes in a camera feed 
  * Publishes a TF location for an Apriltag, so can be shown on RViz
    * (need to research TF more to understand how to parse this/transform into IK)
  * The Apriltag must be listed in the `config/tags.yaml`
    * must list 'size' of the tag in real life, ID of tag
    * Currently loaded on machine is only the 36h11 family tag, ID 1
* Run the video Apriltag detection with `roslaunch apriltags2_ros continuous_detection.launch`
  * the camera & robot should be running first so the topics exist


Apriltag also publishes to `/tag_detections` which seems to contain a pose as an XYZ point/XYZW orientation. Possibly easer to use. Messages look like:
```
header: 
  seq: 2448
  stamp: 
    secs: 1661795935
    nsecs: 923418178
  frame_id: "rgb_camera_link"
detections: 
  - 
    id: [1]
    size: [0.1125]
    pose: 
      header: 
        seq: 8106
        stamp: 
          secs: 1661795935
          nsecs: 923418178
        frame_id: "rgb_camera_link"
      pose: 
        pose: 
          position: 
            x: -0.4604387282265957
            y: -0.34909099407950955
            z: 1.0812755767473274
          orientation: 
            x: -0.6874761962378958
            y: 0.7260056597955421
            z: 0.008088762109603964
            w: 0.015060991954323812
        covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```
Pose values seem to be constant to two decimal points.
