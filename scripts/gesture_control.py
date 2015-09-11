import herbpy
import numpy

env, robot = herbpy.initialize(attach_viewer=True)

keep_going = True

while keep_going:
    print "Press W to wave\nPress Q to quit.\n"
    user_input = raw_input("Gesture? ")

    if user_input == 'Q':
        keep_going = False
    elif user_input == 'W':
        robot.Wave()
