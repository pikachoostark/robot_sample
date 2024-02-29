# robot_sample

Use smth like
```bash
$ colcon build --packages-select robot_sample
```
to make ROS2 pkg

Then use
```bash
$ . install/setup.bash
$ ros2 run robot_sample talker
to run project
```

You can uncomment marked lines in /src/main.cpp to add time delay (5 seconds there) if you want to listen topics manually.
You can use 
```bash
$ ros2 topic echo <topic_name> 
```

(<topic_name>: /ground_truth or /position)
