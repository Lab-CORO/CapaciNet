//
// Created by will on 09/04/24.
//

#include "../include/reachability_visualisation.h"


void ReachabilityVisualisation::show_map(std::string filename) {
    // load the json file
    std::ifstream i(filename);
    json j = json::parse(i);

    // create a rviz maker array points
    visualization_msgs::MarkerArray marker_array;
    // for each sphere in the json
    int id = 0;
    for (auto& sphere_json : j["spheres"]) {
        // create a marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time();
        marker.ns = "reachability" + std::to_string(id);
        marker.id = id ++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.pose.position.x = sphere_json["x"];
        marker.pose.position.y = sphere_json["y"];
        marker.pose.position.z = sphere_json["z"];
        marker.scale.x = double(j["resolution"])/2;
        marker.scale.y = double(j["resolution"])/2;
        marker.scale.z = double(j["resolution"])/2;


       // for each pose with a joint in the sphere increae the color of the sphere
        double rm_score = 0;
        for (auto& pose_json : sphere_json["poses"]) {
            if (pose_json["joints"].size() > 0) {
                rm_score ++;
            }
        }
        this->setColorFromScore(rm_score, marker.color);
        marker_array.markers.push_back(marker);
    }
    // send the msg to the rviz

    this->marker_pub.publish(marker_array);
    // wait to send the msg
    ros::Duration(1).sleep();

}


void ReachabilityVisualisation::setColorFromScore(double score,  std_msgs::ColorRGBA &color) {
    // Vérifie que le score est dans l'intervalle [0, 50]
    if (score < 0) score = 0;
    if (score > 50) score = 50;

    // Calcul de la fraction du score par rapport au maximum
    double fraction = score / 50.0;

    // Interpolation linéaire
    // Rouge diminue alors que Vert augmente en fonction du score
    color.a = 0.5;            // Opacité maximale
    color.r = 1.0 - fraction; // Rouge diminue
    color.g = fraction;       // Vert augmente
    color.b = 0.0;            // Pas de composante bleue pour le gradient rouge-vert
}


