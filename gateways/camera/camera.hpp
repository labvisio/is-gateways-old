#ifndef __IS_GW_CAMERA_HPP__
#define __IS_GW_CAMERA_HPP__

#include <is/is.hpp>
#include <is/msgs/camera.hpp>
#include <is/msgs/common.hpp>
#include <is/msgs/cv.hpp>
#include <is/theora-encoder.hpp>

namespace is {
namespace gw {

using namespace is::msg::common;
using namespace is::msg::camera;

template <typename ThreadSafeCameraDriver>
struct Camera {
  Connection is;
  DataPublisher publisher;
  ServiceProvider service;

  TheoraEncoder encoder;

  Camera(std::string const& name, std::string const& uri, std::string const& format, ThreadSafeCameraDriver& camera)
      : is(is::connect(uri)), publisher(is), service(name, make_channel(uri)) {
    service.expose("set_sample_rate", [&camera](is::Request request) -> is::Reply {
      camera.set_sample_rate(is::msgpack<SamplingRate>(request));
      return is::msgpack(status::ok);
    });

    service.expose("set_resolution", [&camera](is::Request request) -> is::Reply {
      camera.set_resolution(is::msgpack<Resolution>(request));
      return is::msgpack(status::ok);
    });

    service.expose("set_image_type", [&camera](is::Request request) -> is::Reply {
      camera.set_image_type(is::msgpack<ImageType>(request));
      return is::msgpack(status::ok);
    });

    service.expose("set_delay", [&camera](is::Request request) -> is::Reply {
      camera.set_delay(is::msgpack<Delay>(request));
      return is::msgpack(status::ok);
    });

    service.expose("get_headers", [this](is::Request) {
      auto headers = encoder.get_headers();
      return is::msgpack(headers);  // get_headers is thread safe
    });

    service.expose("get_sample_rate", [&camera](is::Request) -> is::Reply {
      return is::msgpack(camera.get_sample_rate());
    });

    service.expose("get_resolution", [&camera](is::Request) -> is::Reply {
      return is::msgpack(camera.get_resolution());
    });

    service.expose("get_image_type", [&camera](is::Request) -> is::Reply {
      return is::msgpack(camera.get_image_type());
    });

    publisher.add(name + ".frame", [&camera, &format]() {
      auto frame = camera.get_last_frame();
      CompressedImage image;
      image.format = "." + format;
      cv::imencode(image.format, frame, image.data);
      auto msg = is::msgpack(image);
      msg->Timestamp(camera.get_last_timestamp().nanoseconds);
      return msg;
    });

    publisher.add(name + ".theora", [this, &camera]() {
      auto frame = camera.get_last_frame().clone();
      auto packets = encoder.encode(frame);
      return is::msgpack(packets);
    });

    publisher.add(name + ".timestamp",
                  [this, &camera]() { return is::msgpack(camera.get_last_timestamp()); });

    std::thread thread([this, &camera]() { service.listen(); });

    for (;;) {
      try {
        camera.update();
        auto n = publisher.publish();
        if (n == 0) {
          camera.stop_capture();
        } else {
          camera.start_capture();
        }
      } catch (...) {
        is::log::warn(":(");
      }
    }

    thread.join();
  }

};  // ::Camera

}  // ::gw
}  // ::is

#endif  // __IS_GW_CAMERA_HPP__