* installed the [Apriltags ROS package](http://wiki.ros.org/apriltag_ros)
  * Takes in a camera feed 
  * Returns a TF location for an Apriltag 
    * (need to research TF more to understand how to parse this/transform into IK)
  * The Apriltag must be listed in the `config/tags.yaml`
    * must list 'size' of the tag in real life, ID of tag
    * Currently loaded on machine is only the 36h11 family tag, ID 1
* Run the video Apriltag detection with `roslaunch apriltags2_ros continuous_detection.launch`
  * the camera & robot should be running first so the topics exist
