#!/usr/bin/env python
import herbpy
import numpy

if __name__ == '__main__':

    env, robot = herbpy.initialize(sim=False)
    env.SetViewer('interactivemarker')
    keep_going = True

    while keep_going:
        print "\n\nWelcome to the pose control:\n \
               Press 0 to reset to relaxed home.\n \
               Press 1 to wave.\n \
               Press 2 to say something.\n \
               Press 3 for the Rock On pose.\n \
               Press 4 for the Right Arm Hug pose.\n \
               Press 5 for the Huzzah pose.\n \
               Press 6 for the Samba pose.\n \
               Press 9 to quit.\n"
        user_input = int(raw_input("Gesture? "))

        if user_input == 9:
            print 'Goodbye!\n'
            keep_going = False
        elif user_input == 1:
            print 'Waving!\n'
            robot.right_arm.SetActive()
            robot.Wave()
        elif user_input == 0:
            print 'Reset to relaxed home'
            robot.right_arm.SetStiffness(1)
            robot.left_arm.SetStiffness(1)
            robot.right_arm.PlanToNamedConfiguration('relaxed_home', execute=True)
            robot.left_arm.PlanToNamedConfiguration('relaxed_home', execute=True)
            robot.right_hand.OpenHand()
            robot.left_hand.OpenHand()
        elif user_input == 2:
            say_this = raw_input('What do you want HERB to say? ')
            robot.Say(str(say_this))
        elif user_input == 3:
            print 'Rock On'
            robot.right_arm.SetActive()
            robot.right_arm.SetStiffness(1)
            robot.HaltHand()
            robot.right_hand.MoveHand(f1=0, f2=0, f3=4, spread=3.14)

            robot.left_arm.SetActive()
            robot.left_arm.SetStiffness(1)
            robot.HaltHand()
            robot.left_hand.MoveHand(f1=0, f2=0, f3=4, spread=3.14)
        elif user_input == 4:
            print 'Right Arm Hug'
            robot.left_arm.SetStiffness(1)
            robot.left_arm.PlanToNamedConfiguration('relaxed_home', execute=True)
            robot.right_arm.SetActive()
            robot.right_arm.SetStiffness(1)
            hug_pose = numpy.array([4.63, -1.64, -0.06, 1.81, -1.50, -0.06, -0.00])
            robot.right_arm.PlanToConfiguration(hug_pose, execute=True)
            robot.right_hand.MoveHand(f1=3, f2=3, f3=3, spread=3.14)
        elif user_input == 5:
            print 'Huzzah'
            robot.right_arm.SetActive()
            robot.right_arm.SetStiffness(1)
            right_huzzah = numpy.array([4.18, -1.57, 1.44, 1.55, -2.21, -0.16, -1.20])
            robot.right_arm.PlanToConfiguration(right_huzzah, execute=True)
            robot.right_hand.MoveHand(f1=3, f2=3, f3=3, spread=3.14)

            robot.left_arm.SetActive()
            robot.left_arm.SetStiffness(1)
            left_huzzah = numpy.array([2.11, -1.57, -1.44, 1.55, -0.74, -0.16, 2.96])
            robot.left_arm.PlanToConfiguration(left_huzzah, execute=True)
            robot.left_hand.MoveHand(f1=3, f2=3, f3=3, spread=3.14)

        elif user_input == 6:
            print 'Samba!'
            robot.right_arm.SetActive()
            robot.right_arm.SetStiffness(1)
            right_samba = numpy.array([3.86, -1.51, 1.67, 1.95, -2.69, 0.04, 2.61])
            robot.right_arm.PlanToConfiguration(right_samba, execute=True)
            robot.right_hand.MoveHand(f1=0, f2=0, f3=0, spread=3.14)

            robot.left_arm.SetActive()
            robot.left_arm.SetStiffness(1)
            left_samba = numpy.array([1.05, -0.72, 0.59, 2.36, 1.19, -0.41, 2.96])
            robot.left_arm.PlanToConfiguration(left_samba, execute=True)
            robot.left_hand.MoveHand(f1=0, f2=0, f3=0, spread=3.14)

        else:
            print 'Input Not Recognized\n'
