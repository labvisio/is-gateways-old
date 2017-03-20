#ifndef __IS_GW_CAMERA_HPP__
#define __IS_GW_CAMERA_HPP__

#include <is/is.hpp>
#include <is/msgs/camera.hpp>
#include <is/msgs/common.hpp>
#include <is/msgs/cv.hpp>

namespace is {
namespace gw {

using namespace is::msg::common;
using namespace is::msg::camera;

template <typename ThreadSafeCameraDriver>
struct Camera {
  is::Connection is;

  // clang-format off
  Camera(std::string const& name, std::string const& uri, ThreadSafeCameraDriver & camera) : is(is::connect(uri)) {
    auto thread = is::advertise(uri, name, {
      {
        "set_sample_rate", [&](is::Request request) -> is::Reply {
          camera.set_sample_rate(is::msgpack<SamplingRate>(request));
          return is::msgpack(status::ok);
        }
      },
      {
        "set_resolution", [&](is::Request request) -> is::Reply {
          camera.set_resolution(is::msgpack<Resolution>(request));
          return is::msgpack(status::ok);
        }
      },
      {
        "set_image_type", [&](is::Request request) -> is::Reply {
          camera.set_image_type(is::msgpack<ImageType>(request));
          return is::msgpack(status::ok);
        }
      },
      {
        "set_delay", [&](is::Request request) -> is::Reply {
          camera.set_delay(is::msgpack<Delay>(request));
          return is::msgpack(status::ok);
        }
      },
      {
        "get_sample_rate", [&](is::Request) -> is::Reply {
          return is::msgpack(camera.get_sample_rate());
        }
      },
      {
        "get_resolution", [&](is::Request) -> is::Reply {
          return is::msgpack(camera.get_resolution());
        }
      },
      {
        "get_image_type", [&](is::Request) -> is::Reply {
          return is::msgpack(camera.get_image_type());
        }
      }
    });

    // clang-format on

    while (1) {
      cv::Mat frame = camera.get_frame();
      Timestamp timestamp = camera.get_last_timestamp();
      CompressedImage image;
      image.format = ".png";
      cv::imencode(image.format, frame, image.data);
      if (!is.publish(name + ".frame", is::msgpack(image), "data", true)) {
        camera.stop_capture();
        is.wait_event("binding.created",
                      [&](auto headers) { return name + ".frame" == headers["routing_key"].GetString(); });
        camera.start_capture();
      }
      is.publish(name + ".timestamp", is::msgpack(timestamp));
    }

    thread.join();
  }
};

}  // ::gw
}  // ::is

#endif // __IS_GW_CAMERA_HPP__