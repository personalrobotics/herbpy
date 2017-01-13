#!/usr/bin/env python
import argparse, copy, numpy, random, yaml
import IPython


def generate_primitives_list():
    primitives = {}

    # Define a primitive as x, y, theta change in grid cells

    ##############################
    # Heading 0
    ##############################
    primitives[0] = []

    # Generate primitives for moving forward and backward
    primitives[0].append((1, 0, 0, 1.))
    primitives[0].append((8, 0, 0, 1.))
    primitives[0].append((-1, 0, 0, 100.))

    # 1/16 theta change
    primitives[0].append((8, 1, 1, 1.))
    primitives[0].append((8, -1, -1, 1.))

    # Turn in place
    primitives[0].append((0, 0, 1, 3.))
    primitives[0].append((0, 0, -1, 3.))

    ##############################
    # Heading 45
    ##############################
    primitives[45] = []

    # Forward and reverse
    primitives[45].append((1, 1, 0, 1.))
    primitives[45].append((6, 6, 0, 1.))
    primitives[45].append((-1, -1, 0, 100.))

    # 1/16 theta change
    primitives[45].append((5, 7, 1, 1.))
    primitives[45].append((7, 5, -1, 1.))

    # turn in place
    primitives[45].append((0, 0, 1, 3.))
    primitives[45].append((0, 0, -1, 3.))

    ##############################
    # Heading 22.5
    ##############################
    primitives[22.5] = []

    # Straight
    primitives[22.5].append((2, 1, 0, 1.))
    primitives[22.5].append((6, 3, 0, 1.))
    primitives[22.5].append((-2, -1, 0, 100.))

    # 1/16 theta change
    primitives[22.5].append((5, 4, 1, 1.))
    primitives[22.5].append((7, 2, -1, 1.))

    # turn in place
    primitives[22.5].append((0, 0, 1, 3.))
    primitives[22.5].append((0, 0, -1, 3.))

    ##############################
    # Heading 67.5 - same as 22.5 but with x and y reversed and
    # heading flipped sign
    ##############################
    primitives[67.5] = []

    # Straight
    primitives[67.5].append((1, 2, 0, 1.))
    primitives[67.5].append((3, 6, 0, 1.))
    primitives[67.5].append((-1, -2, 0, 100.))

    # 1/16 theta change
    primitives[67.5].append((4, 5, -1, 1.))
    primitives[67.5].append((2, 7, 1, 1.))

    # turn in place
    primitives[67.5].append((0, 0, -1, 3.))
    primitives[67.5].append((0, 0, 1, 3.))

    return primitives


if __name__ == '__main__':

    parser = argparse.ArgumentParser(
        description="Generate a primitives for the herb robot")
    parser.add_argument(
        '--resolution', type=float, default=0.1, help="The resolution in xy")
    parser.add_argument(
        '--angles',
        type=int,
        default=16,
        help="The number of angles for the planner to consider")
    parser.add_argument(
        '--outfile',
        type=str,
        default='base_planner_parameters.yaml',
        help="The name of the yaml file to generate")
    parser.add_argument(
        '--actions',
        type=int,
        default=7,
        help="The number of actions to select")
    parser.add_argument(
        '--lweight',
        type=float,
        default=100.0,
        help='The weight to apply to the linear translation during cost computation'
    )
    parser.add_argument(
        '--tweight',
        type=float,
        default=10.0,
        help='The weight to apply to the orientation change during cost computation'
    )
    parser.add_argument(
        '--linear_collision_resolution',
        type=float,
        default=0.05,
        help='The linear resolution for collision checking (meters)')
    parser.add_argument(
        '--angular_collision_resolution',
        type=float,
        default=0.1,
        help='The angular resolution for collision checking')
    parser.add_argument(
        '--debug', action='store_true', help='Print debug info')
    args = parser.parse_args()

    params = {}
    params['cellsize'] = args.resolution
    params['numangles'] = args.angles
    params['linear_weight'] = args.lweight
    params['theta_weight'] = args.tweight
    params['actions'] = []

    num_primitives = args.actions
    if num_primitives > 7:
        print 'Warning: Max of 7 primitives per angle'
    primitives = generate_primitives_list()

    num_angles = args.angles
    angular_resolution = 2. * numpy.pi / num_angles
    resolution = args.resolution
    for angleind in range(num_angles):

        # First pull the appropriate list of primitives for this particular angle
        start_angle = angleind * angular_resolution
        start_degrees = round(start_angle * 180. * 100 / numpy.pi)

        if (start_degrees % 9000) < 0.001:
            prims = primitives[0]
            angle = start_angle
        elif (start_degrees % 4500) < 0.001:
            prims = primitives[45]
            angle = start_angle - numpy.pi / 4.
        elif ((start_degrees - 6750) % 9000) < 0.001:
            prims = primitives[67.5]
            angle = start_angle - 3. * numpy.pi / 8.
        elif ((start_degrees - 2250) % 9000) < 0.001:
            prims = primitives[22.5]
            angle = start_angle - numpy.pi / 8.
        else:
            print 'Unrecognized angle resolution: %0.3f (%0.1f degrees)' % (
                start_angle, start_angle * 180. / numpy.pi)
            IPython.embed()
            exit(0)

        # Now add this orientation to thie list of actions
        action = {}
        action['angle'] = angleind
        action['primitives'] = []

        for primind in range(num_primitives):

            # Grab the current primitive
            prim = prims[primind]

            # Figure out the final coordinate by applying the rotation of the angle to the 
            #  offset defined in the primitive
            final_x = round(prim[0] * numpy.cos(angle) - prim[1] * numpy.sin(
                angle))
            final_y = round(prim[0] * numpy.sin(angle) + prim[1] * numpy.cos(
                angle))
            final_theta = (angleind + prim[2]) % num_angles

            # This will be the final pose, in grid coordinates, of the action
            endcoord = numpy.array([final_x, final_y, final_theta])

            # Set the start and end points of the action - in world coordinates
            startpt = numpy.array([0., 0., start_angle])
            endpt = numpy.array([
                endcoord[0] * resolution, endcoord[1] * resolution, endcoord[2]
                * angular_resolution
            ])

            # A list of intermediate poses visited during the action
            intermediates = []

            if (final_x == 0. and final_y == 0.) or prim[2] == 0.:
                # Figure out how many samples based on the collision resolution 
                #  and the type of action
                if (final_x == 0. and final_y == 0.):  # Turn in place
                    num_samples = int(
                        numpy.ceil(
                            abs(prim[2] * angular_resolution /
                                args.angular_collision_resolution)) + 0.5)
                else:
                    num_samples = int(
                        numpy.ceil(
                            numpy.linalg.norm(endpt[:2]) /
                            args.linear_collision_resolution) + 0.5)

                # Turn in place or move forward or backward
                for sampleidx in range(num_samples + 1):

                    # Perform linear interpolation
                    t = sampleidx / float(num_samples)
                    pt = startpt + (endpt - startpt) * t
                    r = prim[2] * angular_resolution
                    pt[2] = (startpt[2] + r * t) % (2. * numpy.pi)

                    # Add the interpolated pose to the list
                    intermediates.append(pt)
            else:
                # Invert the vehicle model
                R = numpy.array([[
                    numpy.cos(startpt[2]), numpy.sin(endpt[2]) - numpy.sin(
                        startpt[2])
                ], [
                    numpy.sin(startpt[2]), -numpy.cos(endpt[2]) + numpy.cos(
                        startpt[2])
                ]])
                S = numpy.dot(numpy.linalg.pinv(R), (endpt[:2] - startpt[:2]))

                rv = prim[2] * angular_resolution + S[0] / S[1]
                tv = S[1] * rv

                if S[0] < 0.:
                    print 'Warning: l = %d < 0 -> bad action start/end points' % S[
                        0]
#                    IPython.embed()

# Figure out how many samples based on the collision resolution 
#  and the type of action
                num_samples = max(
                    int(0.5 + numpy.ceil(
                        abs(prim[2] * angular_resolution /
                            args.angular_collision_resolution))),
                    int(0.5 + numpy.ceil(
                        numpy.linalg.norm(endpt[:2]) /
                        args.linear_collision_resolution)))

                for sampleidx in range(num_samples + 1):
                    dt = sampleidx / float(num_samples)

                    if dt * tv < S[0]:
                        pt = numpy.array([
                            startpt[0] + dt * tv * numpy.cos(startpt[2]),
                            startpt[1] + dt * tv * numpy.sin(startpt[2]),
                            startpt[2]
                        ])
                    else:
                        dtheta = rv * (dt - S[0] / tv) + startpt[2]
                        pt = numpy.array([
                            startpt[0] + S[0] * numpy.cos(startpt[2]) + S[1] *
                            (numpy.sin(dtheta) - numpy.sin(startpt[2])),
                            startpt[1] + S[0] * numpy.sin(startpt[2]) - S[1] *
                            (numpy.cos(dtheta) - numpy.cos(startpt[2])), dtheta
                        ])
                    intermediates.append(pt)

                # Correct for error
                error = endpt[:2] - intermediates[-1][:2]
                if numpy.linalg.norm(error) > 0.0001:
                    for idx in range(len(intermediates)):
                        intermediates[idx][:2] += error * idx * (1. /
                                                                 num_samples)

            pose_list = []
            for intermediate in intermediates:
                pose_list.append([
                    float(intermediate[0]), float(intermediate[1]), float(
                        intermediate[2])
                ])

            # Create the yaml-ized primitive and add it to the action
            primitive = {}
            primitive['poses'] = pose_list
            primitive['weight'] = prim[3]
            action['primitives'].append(primitive)
        params['actions'].append(action)

    with open(args.outfile, 'w') as f:
        f.write(yaml.dump(params, default_flow_style=False))

    print 'Wrote primitives to file %s' % args.outfile
