Baxter Makes a Smoothie!

This code allows Baxter and Zumy to work together to make a smoothie.
See full documentation and our report on [our google site](sites.google.com/site/baxterblender)

To run this file you must first launch a few nodes:
Launch bax_left.launch in the project package to run ar_track_alvar on the left hand camera
Run joint_trajectory_action_server.py in the baxter_interface package
Launch move_group.launch in the baxter_moveit_config package

Baxter's code can then be run via
`python smoothie.py [fruits] [load]`

For example, if you want to make an apple banana orange smoothie in a new setting, you would run
`python smoothie.py apple banana orange`
You would then need to calibrate Baxter by using his left arm to learn the location of the blender and the desired neutral point, as well as the height of the table.
However, future runs could then load this state with
'python smoothie.py apple orange banana load'
for the same effect.

To use Zumy, launch run_all.launch with a microsoft webcam connected to the computer.
Then run the python files `zumy_driver.py` and `zumy_follow_ar_tag.py`

Zumy will now bring Baxter his missing ingredients!

Enjoy!
