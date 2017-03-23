#ifndef __IS_DRIVER_ATSP_HPP__
#define __IS_DRIVER_ATSP_HPP__

#include <is/is.hpp>
#include <is/logger.hpp>
#include <is/msgs/common.hpp>
#include <is/msgs/geometry.hpp>
#include <is/msgs/robot.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

namespace is {
namespace driver {

using namespace std::chrono;
using namespace is::msg::robot;
using namespace is::msg::common;
using namespace is::msg::geometry;

constexpr double pi() {
  return std::atan(1) * 4;
}
auto deg2rad = [](double deg) { return deg * (pi() / 180.0); };
auto rad2deg = [](double rad) { return rad * (180.0 / pi()); };

struct ATSP {
  is::Connection is;
  std::string broker;
  std::string mqtt_entity;

  Pose pose;
  Speed speed;
  Timestamp last_timestamp;
  std::atomic<int64_t> period_ms;
  std::atomic<int64_t> delay_ms;

  bool ready;
  std::mutex mutex;
  std::condition_variable condition;
  std::thread thread;
  std::atomic<bool> running;

  ATSP(std::string const& broker, std::string const& mqtt_entity) : is(broker) {
    this->broker = broker;
    this->mqtt_entity = mqtt_entity;
    this->period_ms = 500;
    initialize();
  }

  virtual ~ATSP() {
    is::log::info("Stopping publish");
    is.publish(mqtt_entity + ".cmd", BasicMessage::Create("#stopData;"));
    running = false;
    thread.join();
  }

  void set_speed(Speed const& speed) {
    std::unique_lock<std::mutex> lock(mutex);
    is.publish(mqtt_entity + ".cmd", BasicMessage::Create("#driveRobot(" + std::to_string(speed.linear) + "," +
                                                          std::to_string(rad2deg(speed.angular)) + "/0)" + ";"));
  }

  Speed get_speed() {
    std::unique_lock<std::mutex> lock(mutex);
    return this->speed;
  }

  Pose get_pose() {
    std::unique_lock<std::mutex> lock(mutex);
    return this->pose;
  }

  void set_pose(Pose const& pose) {
    std::unique_lock<std::mutex> lock(mutex);
    is.publish(mqtt_entity + ".cmd", BasicMessage::Create("#setPose(" + std::to_string(pose.position.x) + "," +
                                                          std::to_string(pose.position.y) + "/" +
                                                          std::to_string(rad2deg(pose.heading)) + ")" + ";"));
  }

  void set_sample_rate(SamplingRate const& rate) {
    std::unique_lock<std::mutex> lock(mutex);
    if (rate.period) {
      period_ms = std::max<int64_t>(500, *rate.period);
    } else if (rate.rate) {
      period_ms = std::max<int64_t>(500, 1000.0 / *rate.rate);
    }
  }

  SamplingRate get_sample_rate() {
    std::unique_lock<std::mutex> lock(mutex);
    SamplingRate rate;
    rate.period = period_ms;
    rate.rate = static_cast<double>(1000.0 / period_ms);
    return rate;
  }

  void set_delay(Delay const& delay) {
    std::unique_lock<std::mutex> lock(mutex);
    if (delay.milliseconds <= period_ms) {
      delay_ms = delay.milliseconds;
    }
  }

  Timestamp get_last_timestamp() {
    std::unique_lock<std::mutex> lock(mutex);
    return last_timestamp;
  }

  void wait() {
    std::unique_lock<std::mutex> lock(mutex);
    condition.wait(lock, [this] { return ready; });
    ready = false;
  }

 private:
  void initialize() {
    is.publish(mqtt_entity + ".cmd", BasicMessage::Create("#stopAll;"));
    is.publish(mqtt_entity + ".cmd", BasicMessage::Create("#sendData;"));

    auto loop = [this]() {
      is::Connection is_loop(broker);
      auto mqtt_topics = is_loop.subscribe(mqtt_entity + ".Pose");
      running = true;
      while (running) {
        auto mqtt_messages = is_loop.consume_for(mqtt_topics, std::chrono::milliseconds(5 * period_ms));

        if (mqtt_messages == nullptr) {
          is::log::info("Waiting subscription...");
          auto talk_topic = is_loop.subscribe(mqtt_entity + ".talk");
          do {
            is_loop.publish(mqtt_entity + ".cmd.Lua",
                            BasicMessage::Create("mqttClient:publish('" + mqtt_entity + "/talk','Connected!',0,0)"));
          } while (is_loop.consume_for(talk_topic, 1s) == nullptr);

          is::log::info("Connected!");
          is_loop.publish(mqtt_entity + ".cmd", BasicMessage::Create("#stopAll;"));
          is_loop.publish(mqtt_entity + ".cmd", BasicMessage::Create("#sendData;"));
        } else {
          auto timestamp = Timestamp();
          Pose pose;
          Speed speed;
          std::vector<std::string> fields;
          std::string message = mqtt_messages->Message()->Body();
          boost::split(fields, message, boost::is_any_of("/"), boost::token_compress_on);
          if (fields.size() == 5) {
            pose = {.position = {stod(fields[0]), stod(fields[1])}, .heading = deg2rad(stod(fields[2]))};
            speed = {.linear = stod(fields[3]), .angular = stod(fields[4])};
          }
          {
            std::unique_lock<std::mutex> lock(mutex);
            this->pose = pose;
            this->speed = speed;
            last_timestamp = timestamp;
            ready = true;
          }
          condition.notify_one();
        }
      }
    };

    thread = std::thread(loop);
  }
};

}  // ::driver
}  // ::is

#endif  // __IS_DRIVER_PIONEER_HPP__