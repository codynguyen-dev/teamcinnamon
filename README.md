# teamCinnamon

This lab will put together everything you have learned so far into one assignment to be completed on a robot. In summary, you will:

Create a GitHub repo for your lab group

Modify the FSM code from your Lab 2.3 submission to work on the robot. You can start fresh or from one of your submissions.

You do NOT need to latch the ESTOP

You need a FAST way to change between modes. Consider a single keypress rather than full words.

You can choose to use the Turtlesim Teleop node we used before or another of the many, many ROS2 teleoperation nodes available. Some allow the use of game controllers (we do not provide these) and other input devices.
Similar to Objective 2, modify your Lab 3.1 and 3.2 code to work on the robot. The output from 3.2 will be the input to a controller node in Objective 4. Make sure to output the filtered image for debugging purposes.

Use what you learned in Lab 2.2 to implement TWO PID controllers, preferably in the same ROS2 node. One will control the direction and the other the forward speed of the robot. Consider how this appears in a Twist message. You can go back to Lab 1 to experiment. This will output to your FSM code in Objective 2, which will then output to the robot’s hardware interface node (when in Autonomous mode) to move the actual wheels. Your two PID controllers should be used to generate ONE message per image update.

Before Using the Robots
We have limited time this semester to work on the robots, so it is important to get started as soon as possible. Before showing up to class to use the robot, please complete the following:

Understand how the Twist message used on a cmd_vel topic moves a ground robot. Again, use Lab 1 and Turtlesim to get a feel for which number causes which output.

Write code that can nominally complete the above Objectives, once it is tuned and properly connected. You have the individual components, put them together and test them with the provided pingpong ball images BEFORE class. You should see a command telling the robot to go left when the object is on the left side of the screen, forward when the object is small, etc.

Use ROS2 params to make tuning easier. For example, use a param to adjust the high and low end of color thresholding and the k values for your PID controllers. You only need to use one color object. It is your choice what to use.

MAKE SURE YOUR CODE RESPECTS NAMESPACES! Do NOT use absolute topics (starting with a “/”) in your code.

Once you set up your robot (instructions below), you can then focus on the following steps to complete this assignment IN CLASS:

Determine which topic you should subscribe to in order to get an image from the camera and which topic your code should publish to in order to drive the motors.

Verify that your code receives images, that you can control the robot with teleop, and that your FSM can stop the robot quickly if something goes wrong.

Tune the image processing component until it finds the object and not the background.

Tune your turning control such that your robot always points at the object.

Finally, tune your velocity controller such that the robot stays about 10cm away from the object. The exact distance is not important as long as it is consistent and reacts to changes.
