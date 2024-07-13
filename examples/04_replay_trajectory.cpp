#include <iostream>
#include <cmath>
#include <functional>
#include <Eigen/Dense>
#include <franka/exception.h>
#include <franka/duration.h>
#include <franka/model.h>
#include <franka/robot.h>
#include "examples_common.h"
#include <vector>
#include <string>
#include <tuple>
#include <array>
#include <chrono>
#include <queue>
#include "json_utils.h"

using PosTuple = std::tuple<std::array<double, 16>, std::array<double, 7>>;
enum State {INIT, END};

constexpr std:string input_file = "output.json";
constexpr std::string out_file = "output-6.json";

int main(int argc, char** argv) {
    try {
        // replay trajectory json string
        std::string json = readFileToString(input_file);
        try {
            // trajectory data structures
            std::vector<PosTuple> trajectoryVec = parseJson(json);
            std::queue<PosTuple> trajectoryQ;

            // Compliance parameters, set up impedance controller
            const double translational_stiffness{150.0};
            const double rotational_stiffness{10.0};
            Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
            stiffness.setZero();
            stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
            stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
            damping.setZero();
            damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                                Eigen::MatrixXd::Identity(3, 3);
            damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                                    Eigen::MatrixXd::Identity(3, 3);

            // transform vector to the queue (to pop from top to bottom movements)
            for (auto v : trajectoryVec) {
                trajectoryQ.push(v);
            }

            // initialize robot
            franka::Robot robot(argv[1]);
            setDefaultBehavior(robot);

            // Set high collision thresholds to enable hand guiding
            std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
            MotionGenerator motion_generator(0.5, q_goal);
            std::cout << "WARNING: This example will move the robot! "
                    << "Please make sure to have the user stop button at hand!" << std::endl
                    << "Press Enter to continue..." << std::endl;
            std::cin.ignore();
            robot.control(motion_generator);
            std::cout << "Finished moving to initial joint configuration." << std::endl;

            // Set additional parameters always before the control loop, NEVER in the control loop!
            // Set collision behavior.
            robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

            std::vector<PosTuple> trajectory; 
            bool hasEntered = false;

            // load the kinematics and dynamics model
            franka::Model model = robot.loadModel();

            franka::RobotState initial_state = robot.readOnce();

            // equilibrium point is the initial position
            Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(
                initial_state.O_T_EE.data()
            ));


            // define callback for the torque control loop, modified impedance control loop
            std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
                impedance_control_callback = [&](
                                                 const franka::RobotState& robot_state,
                                                 franka::Duration) -> franka::Torques {
                // record new position for the current generated trajectory
                PosTuple posTuple = std::make_tuple(robot_state.O_T_EE, robot_state.q);
                trajectory.push_back(posTuple);

                // use positions recorded from the json file and record them into desired input
                Eigen::Map<Eigen::Matrix4d> desired_input(std::get<0>(trajectoryQ.front()).data());
                Eigen::Vector3d position_d(desired_input.block<3, 1>(0, 3));
                Eigen::Quaterniond orientation_d(desired_input.block<3, 3>(0, 0));

                if (!trajectoryQ.empty()) {
                    // release destined positions from the queue
                    trajectoryQ.pop();
                } else if (!hasEntered) {
                    // write data into json file once the trajecotory finished its execution
                    std::string json = toJson(trajectory);
                    writeToFile(out_file, json);
                    hasEntered = true;
                }

                // go back to the stale state of balance (no torgues applied) once recorded trajectory has finished
                if (hasEntered) {
                     return std::array<double, 7>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                }
                
                // get state variables
                std::array<double, 7> coriolis_array = model.coriolis(robot_state);
                std::array<double, 42> jacobian_array =
                    model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

                // convert to Eigen
                Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
                Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
                Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
                Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

                Eigen::Affine3d transform;
                transform = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
                Eigen::Vector3d position(transform.translation());
                Eigen::Quaterniond orientation(transform.rotation());

                // compute error to desired equilibrium pose
                // position error
                Eigen::Matrix<double, 6, 1> error;
                error.head(3) << position - position_d;

                // orientation error
                // "difference" quaternion
                if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
                    orientation.coeffs() << -orientation.coeffs();
                }
                // "difference" quaternion
                Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
                error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
                // Transform to base frame
                error.tail(3) << transform.rotation() * error.tail(3);

                // compute control
                Eigen::VectorXd tau_task(7), tau_d(7);

                // Spring damper system with damping ratio=1
                tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
                tau_d << tau_task + coriolis;

                std::array<double, 7> tau_d_array{};
                Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
                return tau_d_array;
            };
            

            // start real-time control loop
            std::cout << "WARNING: Collision thresholds are set to high values. "
                    << "Make sure you have the user stop at hand!" << std::endl
                    << "After starting try to push the robot and see how it reacts." << std::endl
                    << "Press Enter to continue..." << std::endl;
            std::cin.ignore();
            robot.control(impedance_control_callback);
        } catch (const std::exception& e) {
            std::cerr << "Error parsing JSON: " << e.what() << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    } 
  

  return 0;
}
