#!/usr/bin/env python
import herbpy
import numpy

if __name__ == '__main__':

    env, robot = herbpy.initialize()
    keep_going = True

    while keep_going:
        print "\n\nWelcome to the gesture control:\n \
               Press W to wave.\n \
               Press S to say something.\n \
               Press Y to nod 'yes'.\n \
               Press N to nod 'no'.\n \
               Press H to high five.\n \
               Press P to point at an object.\n \
               Press S to present an object.\n \
               Press R to reset the robot.\n \
               Press Q to quit.\n"
        user_input = raw_input("Gesture? ")

        if user_input == 'Q':
            print 'Goodbye!\n'
            keep_going = False
        elif user_input == 'W':
            print 'Waving!\n'
            robot.Wave()
        elif user_input == 'Y':
            print 'Nodding yes!\n'
            robot.Nod(word='yes')
        elif user_input == 'N':
            print 'Nodding no\n!'
            robot.Nod(word='no')
        elif user_input == 'H':
            print 'High Fiving!\n'
            robot.HighFive()
        elif user_input == 'R':
            robot.right_arm.PlanToNamedConfiguration('relaxed_home', execute=True)
            robot.left_arm.PlanToNamedConfiguration('relaxed_home', execute=True)
            robot.right_hand.OpenHand()
            robot.left_hand.OpenHand()
        elif user_input == 'S':
            say_this = raw_input('What do you want HERB to say? ')
            robot.Say(str(say_this))
        elif user_input == 'P':
            items = [x.GetName() for x in env.GetBodies()]
            print "HERB can see these things:"
            print items[1:] #Don't show the robot as an option
            point_at = raw_input('I want to point at item number: ')
            robot.Point(env.GetBodies()[int(point_at)])
        elif user_input == 'S':
            items = [x.GetName() for x in env.GetBodies()]
            print "HERB can see these things:"
            print items[1:]
            point_at = raw_input('I want to point at item number: ')
            robot.Point(env.GetBodies()[int(point_at)])
        else:
            print 'Input Not Recognized\n'
