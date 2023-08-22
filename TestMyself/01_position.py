from copy import copy
from pathlib import Path
from sys import path

# Path to the build directory including a file similar to 'ruckig.cpython-37m-x86_64-linux-gnu'.
build_path = Path(__file__).parent.absolute().parent / 'build'
path.insert(0, str(build_path))

from ruckig import InputParameter, OutputParameter, Result, Ruckig, Synchronization
from plotter import Plotter

if __name__ == '__main__':
    # Create instances: the Ruckig OTG as well as input and output parameters
    otg = Ruckig(3, 0.01)  # DoFs, control cycle
    inp = InputParameter(3)
    out = OutputParameter(3)

    # Set input parameters
    inp.current_position = [0.0, 0.0, 0.5]
    inp.current_velocity = [1.0, -2.2, -0.5]
    inp.current_acceleration = [0.0, 2.5, -0.5]

    inp.target_position = [5.0, -2.0, 3.5]
    inp.target_velocity = [0.0, 1, 5.0]
    inp.target_acceleration = [0.0, 0.0, 0.5]

    inp.max_velocity = [2.0, 1.0, 10.0]
    inp.max_acceleration = [3.0, 2.0, 10.0]
    inp.max_jerk = [4.0, 3.0, 10.0]

    inp.synchronization = Synchronization.Time
    inp.enabled = [True, True, False]

    print('\t'.join(['t'] + [str(i) for i in range(otg.degrees_of_freedom)]))

    # Generate the trajectory within the control loop
    first_output, out_list = None, []
    res = Result.Working

    updated = False

    while res == Result.Working:
        res = otg.update(inp, out)

        # if res != Result.Working:
        #     print(f"Error: {res}")
        #
        # try:
        #     result = otg.validate_input(inp, check_target_state_within_limits=True,
        #                                 check_current_state_within_limits=True)
        # except RuntimeError as e:
        #     print(f"{e}")

        print('\t'.join([f'{out.time:0.3f}'] + [f'{p:0.3f}' for p in out.new_position]))
        out_list.append(copy(out))

        out.pass_to_input(inp)

        # if out.new_position[0] > 4 and not updated:
        #     inp.max_velocity = [5, 1.0, 10.0]
        #     inp.target_position = [7.0, -2.0, 3.5]
        #     inp.target_velocity = [-2.0, 1, 5.0]
        #     inp.max_acceleration = [10.0, 2.0, 10.0]
        #     inp.max_jerk = [20.0, 3.0, 10.0]
        #     out_list.clear()
        #     updated = True

        if not first_output:
            first_output = copy(out)

    print(f'Calculation duration: {first_output.calculation_duration:0.1f} [µs]')
    print(f'Trajectory duration: {first_output.trajectory.duration:0.4f} [s]')

    # Plot the trajectory
    Plotter.plot_trajectory(Path(__file__).parent.absolute() / '1_trajectory.pdf', otg, inp, out_list,
                            show=True,
                            plot_jerk=True)
