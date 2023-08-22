#include <iostream>

#include <ruckig/ruckig.hpp>


using namespace ruckig;

int main() {
    double cycle = 0.004;
    // Create instances: the Ruckig OTG as well as input and output parameters
    Ruckig<3> otg {cycle};  // control cycle
    InputParameter<3> input;
    OutputParameter<3> output;

    Trajectory<3> traj;

    // Set input parameters
    input.current_position = {0.0, 0.0, 0.5};
    input.current_velocity = {1.0, -2.2, -0.5};
    input.current_acceleration = {0.0, 2.5, -0.5};

    input.target_position = {5.0, -2.0, 3.5};
    input.target_velocity = {0.0, 1, 5.0};
    input.target_acceleration = {0.0, 0.0, 0.5};

    input.max_velocity = {2.0, 1.0, 10.0};
    input.max_acceleration = {3.0, 2.0, 10.0};
    input.max_jerk = {4.0, 3.0, 10.0};

    input.enabled = {true, true, false};

    auto result = otg.calculate(input, output.trajectory);

    double duration = output.trajectory.get_duration();

    int steps = static_cast<int>(duration / cycle);

    OutputParameter<3> cur;

    output.trajectory.at_time((steps - 1)*cycle, cur.new_position, cur.new_velocity, cur.new_acceleration);

    double time;
    output.trajectory.get_first_time_at_position(0, 4.9, time);
    std::cout << time << std::endl;

    auto extrema = output.trajectory.get_position_extrema();

    auto profile_0 = output.trajectory.get_profiles()[0][0];
    auto profile_1 = output.trajectory.get_profiles()[0][1];



    int count = 1;

    // Generate the trajectory within the control loop
    std::cout << "t | p1 | p2 | p3" << std::endl;
    while (otg.update(input, output) == Result::Working) {
        auto& p = output.new_position;
        std::cout << count << " " << output.time << " " << p[0] << " " << p[1] << " " << p[2] << " " << std::endl;

        output.pass_to_input((input));

//        std::cout << output.calculation_duration << std::endl;

        count++;
    }

    std::cout << "Trajectory duration: " << output.trajectory.get_duration() << " [s]." << std::endl;
}
