# Exercise 2

Exercise 2 is where we made our own custom ROS nodes for moving the robot around the lab's Duckietown.

## Running the demo

First start the LED emitter node by running this command in the terminal:

```bash
dts duckiebot demo --demo_name led_emitter_node --duckiebot_name $BOT --package_name led_emitter --image duckietown/dt-core:daffy-arm64v8
```

Where `$BOT` is the hostname of your robot.

Then you can run the program by running this at the repo's root:

```bash
dts devel build -f -H $BOT && dts devel run -H $BOT
```

If pulling, and building the images are taking too long you can build and run the container locally by running this instead:

```bash
dts devel build -f && dts devel run -R $BOT
```