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
#include "spline.h"
#include <algorithm>


int main() {
  uWS::Hub h;

  //Create HighwayPlanner object;
  HighwayPlanner highway_planner;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;

  // Waypoint map to read from
  std::string map_file_ = "../../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  std::string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
    //std::sort( map_waypoints_x.begin(), map_waypoints_x.end() );
    //std::sort( map_waypoints_y.begin(), map_waypoints_y.end() );
    
    //go through the list and remove duplicates in both

    //pobably when pushing back check if > then the last in array, else do neithe rpush x nor y
    /*std::sort( map_waypoints_x.begin(), map_waypoints_x.end() );
    map_waypoints_x.erase( std::unique( map_waypoints_x.begin(), map_waypoints_x.end() ), map_waypoints_x.end() );
    std::sort( map_waypoints_y.begin(), map_waypoints_y.end() );
    map_waypoints_y.erase( std::unique( map_waypoints_y.begin(), map_waypoints_y.end() ), map_waypoints_y.end() );*/
    
    //tk::spline s;
    //s.set_points(map_waypoints_x, map_waypoints_y);
    
    highway_planner.setMap(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy);
    
  highway_planner.init();

  h.onMessage([&highway_planner,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = helpers::hasData(data);

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
            highway_planner.setPreviousPath(previous_path);
            // Main car's localization Data
              highway_planner.setVehiclePose(j[1]["x"], j[1]["y"], j[1]["s"], j[1]["d"], j[1]["yaw"], j[1]["speed"],previous_path.planned_coordinates_cartesian.next_x_vals.size());


          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.

            //vector<vector<double>>
            //vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"].get<vector<vector<double>>>();
            
            
            auto& sensor_fusion = j[1]["sensor_fusion"];
            for(int i = 0; i < sensor_fusion.size(); ++i)
            {
                if(i<12)
                {
                highway_planner.setObjectToIndex(i, sensor_fusion[i][0], sensor_fusion[i][1], sensor_fusion[i][2], sensor_fusion[i][3], sensor_fusion[i][4],
                                                 sensor_fusion[i][5], sensor_fusion[i][6]);
                }
                else{std::cout << "ERROR: Objects array out of range..."<<'\n';}
            }

          nlohmann::json msgJson;
          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
           highway_planner.step();


            
          msgJson["next_x"] = highway_planner.getOutputPath().next_x_vals;
          msgJson["next_y"] = highway_planner.getOutputPath().next_y_vals;

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
