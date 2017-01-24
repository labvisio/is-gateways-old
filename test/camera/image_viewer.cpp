#include <boost/program_options.hpp>
#include <chrono>
#include <iostream>
#include <is/is.hpp>
#include <is/msgs/camera.hpp>
#include <is/msgs/common.hpp>
#include <opencv2/highgui.hpp>

namespace po = boost::program_options;
using namespace is::msg::camera;
using namespace std::chrono_literals;

int main(int argc, char* argv[]) {
  std::string uri;
  std::string entity;

  double fps;
  std::string img_type;
  is::msg::camera::Resolution resolution;
  is::msg::common::SamplingRate sample_rate;

  po::options_description description("Allowed options");
  auto&& options = description.add_options();
  options("help,", "show available options");
  options("uri,u", po::value<std::string>(&uri)->default_value("amqp://localhost"), "broker uri");
  options("entity,e", po::value<std::string>(&entity), "entity name");
  options("width,w", po::value<unsigned int>(&resolution.width)->default_value(1288), "image width");
  options("height,h", po::value<unsigned int>(&resolution.height)->default_value(728), "image height");
  options("fps,f", po::value<double>(&fps)->default_value(10.0), "frames per second");
  options("type,t", po::value<std::string>(&img_type)->default_value("gray"), "image type");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, description), vm);
  po::notify(vm);

  if (vm.count("help") || !vm.count("entity")) {
    std::cout << description << std::endl;
    return 1;
  }

  auto is = is::connect(uri);
  auto client = is::make_client(is);
  sample_rate.rate = fps;

  client.request(entity + ".set_sample_rate", is::msgpack(sample_rate));
  client.request(entity + ".set_resolution", is::msgpack(resolution));
  client.request(entity + ".set_image_type", is::msgpack(ImageType{img_type}));

  while (client.receive_for(1s) != nullptr) {
  }

  auto frames = is.subscribe({entity + ".frame"});

  while (1) {
    auto message = is.consume(frames);
    auto image = is::msgpack<is::msg::camera::CompressedImage>(message);
    cv::Mat frame = cv::imdecode(image.data, CV_LOAD_IMAGE_COLOR);

    cv::imshow("Webcam stream", frame);
    cv::waitKey(1);
  }

  return 0;
}