# Gazebo-Screen
**ROS Package to project a camera stream over a screen in Gazebo**

This ROS package creates a screen_link in Gazebo to introduce a live screen in the simulated environment. The screen is created in Gazebo using the Gazebo video plugin. The visual plugin displays a ROS image stream on an OGRE Texture inside gazebo. This plugin does not modify the texture of the existing link surfaces, but creates a new texture on top of it. The texture will be created on the XY plane, visible from the +Z side in the reference frame of the link. The plugin requires a pixel size while constructing the texture, and will resize incoming ROS image messages to match if they are a different size. The plugin would also rescale the image to match the dimensions of the projection surface of the link. 

To display a live stream, publish the image stream over the '/screen/image_topic' ROS topic in the form sensor_msgs/Image. 
