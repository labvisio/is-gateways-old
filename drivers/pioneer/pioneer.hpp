#ifndef __IS_DRIVER_PIONEER_HPP__
#define __IS_DRIVER_PIONEER_HPP__

#include <is/logger.hpp>
#include <is/msgs/common.hpp>
#include <is/msgs/geometry.hpp>
#include <is/msgs/robot.hpp>

#include <cmath>
#include <chrono>
#include <memory>
#include <string>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

#ifndef isnan
#if __cplusplus <= 199711L  // c++98 or older
  #define isnan(x) ::isnan(x)
#else
  #define isnan(x) std::isnan(x)
#endif
#endif

#include <Aria/Aria.h>

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

struct Pioneer {
  ArRobot robot;
  ArDeviceConnection* connection;

  Timestamp last_timestamp;
  std::atomic<int64_t> period_ms;
  std::atomic<int64_t> delay_ms;

  bool ready;
  std::mutex mutex;
  std::condition_variable condition;
  std::thread thread;
  std::atomic<bool> running;

  Pioneer() : connection(nullptr) {}
  Pioneer(std::string const& port) { connect(port); }
  Pioneer(std::string const& hostname, int port) { connect(hostname, port); }

  virtual ~Pioneer() {
    running = false;
    thread.join();
    if (connection != nullptr) {
      delete connection;
    }
  }

  void connect(std::string const& port) {
    Aria::init();
    auto serial = new ArSerialConnection;
    connection = serial;
    serial->open(port.c_str());
    robot.setDeviceConnection(serial);
    initialize();
  }

  void connect(std::string const& hostname, int port) {
    Aria::init();
    auto tcp = new ArTcpConnection;
    connection = tcp;
    tcp->open(hostname.c_str(), port);
    robot.setDeviceConnection(tcp);
    initialize();
  }

  void set_speed(Speed const& speed) {
    robot.lock();
    robot.setVel(speed.linear);
    robot.setRotVel(rad2deg(speed.angular));
    robot.unlock();
  }

  Speed get_speed() {
    robot.lock();
    double linear = robot.getVel();
    double angular = robot.getRotVel();
    robot.unlock();
    return {linear, deg2rad(angular)};
  }

  Pose get_pose() {
    robot.lock();
    ArPose arpose = robot.getPose();
    robot.unlock();
    Pose pose;
    pose.position.x = arpose.getX();
    pose.position.y = arpose.getY();
    pose.heading = deg2rad(arpose.getTh());
    return pose;
  }

  void set_pose(Pose const& pose) {
    robot.lock();
    robot.moveTo(ArPose(pose.position.x, pose.position.y, rad2deg(pose.heading)));
    robot.unlock();
  }

  void set_sample_rate(SamplingRate const& rate) {
    if (rate.period) {
      period_ms = std::max<int64_t>(100, *rate.period);
    } else if (rate.rate) {
      period_ms = std::max<int64_t>(100, 1000.0 / *rate.rate);
    }
  }

  SamplingRate get_sample_rate() {
    SamplingRate rate;
    rate.period = period_ms;
    rate.rate = static_cast<double>(1000.0 / period_ms);
    return rate;
  }

  void set_delay(Delay const& delay) {
    if (delay.milliseconds <= period_ms) {
      delay_ms = delay.milliseconds;
    }
  }

  Timestamp get_last_timestamp() { return last_timestamp; }

  void wait() {
    std::unique_lock<std::mutex> lock(mutex);
    condition.wait(lock, [this] { return ready; });
    ready = false;
  }

 private:
  void initialize() {
    if (!robot.blockingConnect()) {
      throw std::runtime_error("Could not connect to robot.");
    }

    robot.runAsync(true);
    robot.lock();
    robot.moveTo(ArPose(0.0, 0.0, 0.0));
    robot.enableMotors();
    robot.setVel(0.0);
    robot.setRotVel(0.0);
    robot.unlock();

    period_ms = 100;
    delay_ms = 0;

    auto loop = [this]() {
      running = true;
      while (running) {
        auto now = std::chrono::system_clock::now();
        last_timestamp = Timestamp();
        std::this_thread::sleep_until(now + milliseconds(period_ms + delay_ms));
        {
          std::unique_lock<std::mutex> lock(mutex);
          ready = true;
        }
        condition.notify_one();
      }
    };

    thread = std::thread(loop);
  }
};

}  // ::driver
}  // ::is

#endif  // __IS_DRIVER_PIONEER_HPP__