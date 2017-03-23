#include <boost/program_options.hpp>
#include <iostream>
#include "../../drivers/atsp/atsp.hpp"
#include "../robot/robot.hpp"

namespace po = boost::program_options;

int main(int argc, char* argv[]) {
  std::string uri;
  std::string mqtt_entity;
  std::string is_entity;

  po::options_description description("Allowed options");
  auto&& options = description.add_options();
  options("help,", "show available options");
  options("uri,u", po::value<std::string>(&uri)->default_value("amqp://guest:guest@localhost"), "broker uri");
  options("mqtt_entity,m", po::value<std::string>(&mqtt_entity), "mqtt entity name");
  options("is_entity,e", po::value<std::string>(&is_entity), "is entity name");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, description), vm);
  po::notify(vm);

  if (vm.count("help") || !vm.count("is_entity") || !vm.count("mqtt_entity")) {
    std::cout << description << std::endl;
    return 1;
  }

  is::driver::ATSP robot(uri, mqtt_entity);

  is::gw::Robot<is::driver::ATSP> gw(is_entity, uri, robot);

  return 0;
}