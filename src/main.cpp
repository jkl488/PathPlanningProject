#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
//new includes
#include "HighwayPlanner.hpp"
#include "HelperFunctions.hpp"
#include <algorithm>
#include "Map.hpp"

//find the issue, i think we should use get spline xy
//cold start: low speed trajectory

int main() {
  uWS::Hub h;

  //Create HighwayPlanner object;
  HighwayPlanner highway_planner;
    
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
    Map highway_map;

  // Waypoint map to read from
  std::string map_file = "../../data/highway_map.csv";
    highway_map.ReadFile(map_file);



  highway_planner.SetMap(highway_map);
    
  highway_planner.Init();

  h.onMessage([&highway_planner,&highway_map]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = helpers::HasData(data);

      if (s != "") {
        auto j = nlohmann::json::parse(s);
        
          std::string event = j[0].get<std::string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          
          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
            
            OutputPath previous_path;
            previous_path.end_path_d = end_path_d;
            previous_path.end_path_s = end_path_s;
            for(int i = 0; i < previous_path_x.size();++i)
            {
                previous_path.planned_coordinates_cartesian.next_x_vals.push_back(previous_path_x[i]);
                previous_path.planned_coordinates_cartesian.next_y_vals.push_back(previous_path_y[i]);
            }
            highway_planner.SetPreviousPath(previous_path);
            // Main car's localization Data
            std::vector<double> f_vehicle = highway_map.GetFrenet(j[1]["x"], j[1]["y"], j[1]["yaw"]);
            std::cout << "calculated: s: " << f_vehicle.at(0) << ",d: " << f_vehicle.at(1) << '\n';
            std::cout << "perception: s: " << j[1]["s"] << ",d: " << j[1]["d"]  << '\n';
                                                                                             
                                                                                             
              highway_planner.SetVehiclePose(j[1]["x"], j[1]["y"], f_vehicle.at(0) , f_vehicle.at(1) , j[1]["yaw"], j[1]["speed"],previous_path.planned_coordinates_cartesian.next_x_vals.size());


          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
            //auto& sensor_fusion = j[1]["sensor_fusion"];
            std::vector<std::vector<double>> sensor_fusion = j[1]["sensor_fusion"].get<std::vector<std::vector<double>>>();
            highway_planner.InsertObjects(sensor_fusion);

          nlohmann::json msgJson;
          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
           highway_planner.Step();


            
          msgJson["next_x"] = highway_planner.GetOutputPath().next_x_vals;
          msgJson["next_y"] = highway_planner.GetOutputPath().next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
