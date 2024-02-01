Hi this is simple demo of running a turtle sim to draw a circle and a square. They are far from perfect. Infact sometimes the square is quite trapzoidal and the circle is either too small or too big. But this is due to delays in the loops and the ROS communication and things take little longer to take effect than we would like.

To run this code:
    This assumes you aleady have ros installed. If not, please get ros installed before proceeding
    first go to the directory ~/ros
    
        - run catkin_make  

        - run source devel/setup.bash  

        - roslaunch assignment2 assignment.launch
        

    Output should be two turtlesim windows will open. One will draw circle and the other will draw a pseudo sqare(very slowly)

    See the attached video for expected results.

    [screenRecord.mkv](/video/screenRecord.mkv)

    <video src='screenRecord.mkv' width=180/>
