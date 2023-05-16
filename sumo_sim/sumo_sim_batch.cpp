#include <iostream>
#include <string>
#include <vector>

#include "argparse/argparse.hpp"
#include "sumo_simulator.h"
#include "parameters.h"
#include "sumo_batch_utility.h"
#include "colormod.h"

using namespace intersection_management;

int main(int argc, char *argv[]) {
    argparse::ArgumentParser program("sumo_batch");
    program.add_argument("-v", "--verbose");  // parameter packing

    program.add_argument("--num_nodes")
        .default_value(10)
        .help("the number of vehicles in sumo simulation")
        .scan<'i', int>();
    program.add_argument("--test_count")
        .default_value(-1)
        .help("total test to do in the batch test")
        .scan<'i', int>();
    program.add_argument("--schedule_methods")
        .default_value<std::vector<std::string>>({"dynamic_lane", "dfs", "bfs", "mdbfs", "fifo"})
        .help("schedule methods list that will be in the batch test")
        .append();
    program.add_argument("--arrival_interval_avg")
        .default_value(2.0)
        .help("the average arrival interval in seconds between vehicles")
        .scan<'f', double>();
    program.add_argument("--geometryID")
        .default_value(0)
        .help("index of the intersection scenarios")
        .scan<'i', int>();
    program.add_argument("--print_interval")
        .default_value(1)
        .help("interval between two summary prints")
        .scan<'i', int>();
    program.add_argument("--starting_seed")
        .default_value(-1)
        .help("starting random seed for the batch test")
        .scan<'i', int>();
    program.add_argument("--log_dir")
        .default_value(std::string{"/log/sumosim"})
        .help("log file directory");
    program.add_argument("--sumo_step_length")
        .default_value(0.01)
        .help("the step length used in sumo simulation")
        .scan<'f', double>();

    try {
        program.parse_args(argc, argv);
    }
    catch (const std::runtime_error &err) {
        std::cout << err.what() << std::endl;
        std::cout << program;
        exit(0);
    }
    int num_nodes = program.get<int>("--num_nodes");
    int test_count = program.get<int>("--test_count");
    std::vector<std::string> schedule_methods = program.get<std::vector<std::string>>("--schedule_methods");
    double arrival_interval_avg = program.get<double>("--arrival_interval_avg");
    int geometryID = program.get<int>("--geometryID");
    int print_interval = program.get<int>("--print_interval");
    int starting_seed = program.get<int>("--starting_seed");
    std::string log_dir = program.get<std::string>("--log_dir");
    double sumo_step_length = program.get<double>("--sumo_step_length");

    std::cout << Color::green;
    std::cout << "######### Batch sumo sim configs: #########\n"
        << "the number of vehicles in sumo simulation, --num_nodes: " << num_nodes << std::endl
        << "total test to do in the batch test, --test_count: " << test_count << std::endl
        << "schedule methods list that will be in the batch test --schedule_methods:";
    for (auto method : schedule_methods) std::cout << " " << method << ",";
    std::cout << std::endl
        << "the average arrival interval in seconds between vehicles, --arrival_interval_avg: " << arrival_interval_avg << std::endl
        << "index of the intersection scenarios, --geometryID: " << geometryID << std::endl
        << "interval between two summary prints, --print_interval: " << print_interval << std::endl
        << "starting random seed for the batch test, --starting_seed: " << starting_seed << std::endl
        << "log file directory, --log_dir: " << log_dir << std::endl
        << "the step length used in sumo simulation, --sumo_step_length: " << sumo_step_length << std::endl
        << "############# End of configs: #############\n\n";
    std::cout << Color::def;

    sumoBatchTest(num_nodes, test_count, schedule_methods, arrival_interval_avg, geometryID, print_interval,
                  starting_seed, sumo_step_length, log_dir);
    return 0;
}