#include <uWS/uWS.h>
#include <iostream>
#include <chrono>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid;
  // Initialize the pid variable.
  pid.Init(0.10, 0.00004, 4.0);

  clock_t begin_time = clock();
  bool first_twiddle_iteration = true;
  int param_to_modify = 0;
  double this_twiddle_iteration_time = -1.0;
  auto twiddle_start_time = std::chrono::high_resolution_clock::now();
  double tolerance = 0.001;
  std::cout << "Begin time: " << float(begin_time) << "\n";
  std::cout << "Clocks per sec: " << float(CLOCKS_PER_SEC) << "\n";
  h.onMessage([&pid, &this_twiddle_iteration_time, &twiddle_start_time,
                  &tolerance, &first_twiddle_iteration,
                  &param_to_modify](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {

        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {

          /*
           ********************************************************************************
           * Some time and twiddle variable bookkeeping / tweaking. Very crude at this point
           ********************************************************************************
           */

          if(this_twiddle_iteration_time < -0.5){
            // Indication
            twiddle_start_time = std::chrono::high_resolution_clock::now();
            this_twiddle_iteration_time = 0.0;
            if(first_twiddle_iteration)
              // Do nothing, just set the flag
              first_twiddle_iteration = false;
            else {
              // Nullify the PID controller total error
              // Modify the PID controller parameters
            }

          } else {
            auto t_now = std::chrono::high_resolution_clock::now();
            this_twiddle_iteration_time = std::chrono::duration<double, std::milli>(
                t_now - twiddle_start_time).count() / 1000.0;
          }

          /*
           ********************************************************************************
           * Adtual PID processing
           ********************************************************************************
           */

          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());

          if(this_twiddle_iteration_time >= 30 || pid.TotalD() >= tolerance){
          }
          double steer_value = pid.GetAngle(cte, speed);

          std::cout << "Time from twiddle begin: " << this_twiddle_iteration_time << "\n";
          std::cout << "CTE: " << cte << " Speed: " << speed << " Steering Value: " << steer_value << std::endl;
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.4;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
