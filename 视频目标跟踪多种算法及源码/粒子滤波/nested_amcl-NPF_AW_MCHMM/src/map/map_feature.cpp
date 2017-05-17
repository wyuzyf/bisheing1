/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/**************************************************************************
 * Desc: Global map storage functions
 * Author: Andrew Howard
 * Date: 6 Feb 2003
 * CVS: $Id: map_store.c 2951 2005-08-19 00:48:20Z gerkey $
**************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include "ros/assert.h"
#include "map.h"

using namespace std;

int doesFileExist(const char* filename);

// SINA: Computes the distance from the two neasrest walls around the robot pose
double* map_side_walls(map_t *map, pf_vector_t pose, double max_range){
    double heading = pose.v[2];

    if (std::isnan(heading)){
        ROS_DEBUG("Heading is NaN!");
        return NULL;
    }

    int index = -1;

    if (heading < M_PI && heading >= (M_PI / 2.0)){
        if (abs(heading - M_PI) < abs(heading - (M_PI / 2.0))){
            index = 0;
        }else{
            index = 1;
        }
    }else if (heading < (M_PI / 2.0) && heading >= 0.0){
        if (abs(heading - (M_PI / 2.0)) < abs(heading - 0.0)){
            index = 1;
        }else{
            index = 2;
        }
    }else if (heading < 0.0 && heading >= -(M_PI / 2.0)){
        if (abs(heading - 0) < abs(heading - (-(M_PI / 2.0)))){
            index = 2;
        }else{
            index = 3;
        }
    }else if (heading < -(M_PI / 2.0) && heading >= -M_PI){
        if (abs(heading - (-(M_PI / 2.0))) < abs (heading - (-M_PI))){
            index = 3;
        }else{
            index = 0;
        }
    }

    double* results = new double[2];
    switch (index) {
    case 0:
        results[0] = map_calc_range(map, pose.v[0], pose.v[1], (-M_PI / 2.0), max_range);
        results[1] = map_calc_range(map, pose.v[0], pose.v[1], (M_PI / 2.0), max_range);
        break;
    case 1:
        results[0] = map_calc_range(map, pose.v[0], pose.v[1], (M_PI), max_range);
        results[1] = map_calc_range(map, pose.v[0], pose.v[1], (0.0), max_range);
        break;
    case 2:
        results[0] = map_calc_range(map, pose.v[0], pose.v[1], (M_PI / 2.0), max_range);
        results[1] = map_calc_range(map, pose.v[0], pose.v[1], (-M_PI / 2.0), max_range);
        break;
    case 3:
        results[0] = map_calc_range(map, pose.v[0], pose.v[1], (0.0), max_range);
        results[1] = map_calc_range(map, pose.v[0], pose.v[1], (M_PI), max_range);
        break;
    default:
        results[0] = max_range;
        results[1] = max_range;
        break;
    }

    return results;
}

// SINA: Return the value for crosswalk feature based on the current pose
int map_see_crosswalk(map_t *map, pf_vector_t pose, double delta_dist, double max_dist){
    double dist = -1;
    for (int i = 0; i < map->cross_walks_count; i++){
        double temp_dist = pow((pose.v[0] - map->cross_walks[i].x), 2) + pow((pose.v[1] - map->cross_walks[i].y), 2);
        temp_dist = sqrt(temp_dist);

        if (dist == -1 || temp_dist < dist){
            pf_vector_t test_delta_pose;

            test_delta_pose.v[0] = cos(pose.v[2]) * delta_dist;
            test_delta_pose.v[1] = sin(pose.v[2]) * delta_dist;
            test_delta_pose.v[2] = 0.0;

            test_delta_pose = pf_vector_add(pose, test_delta_pose);

            double test_dist = pow((test_delta_pose.v[0] - map->cross_walks[i].x), 2) +
                    pow((test_delta_pose.v[1] - map->cross_walks[i].y), 2);

            test_dist = sqrt(test_dist);

            if (test_dist < temp_dist){
                dist = temp_dist;
            }
        }
    }

    if (dist != -1){
        if (dist <= max_dist){
            return 1;
        }
        return 0;
    }else{
        return 0;
    }
}

// SINA: Return the value for turnpoint feature based on the current pose
int map_see_turnpoint(map_t *map, pf_vector_t pose, double delta_dist, double max_dist){
    double dist = -1;
    int turn_point_index = -1;
    for (int i = 0; i < map->turn_points_count; i++){
        double temp_dist = pow((pose.v[0] - map->turn_points[i].x), 2) + pow((pose.v[1] - map->turn_points[i].y), 2);
        temp_dist = sqrt(temp_dist);

        if (dist == -1 || temp_dist < dist){
            pf_vector_t test_delta_pose;

            test_delta_pose.v[0] = cos(pose.v[2]) * delta_dist;
            test_delta_pose.v[1] = sin(pose.v[2]) * delta_dist;
            test_delta_pose.v[2] = 0.0;

            test_delta_pose = pf_vector_add(pose, test_delta_pose);

            double test_dist = pow((test_delta_pose.v[0] - map->turn_points[i].x), 2) +
                    pow((test_delta_pose.v[1] - map->turn_points[i].y), 2);

            test_dist = sqrt(test_dist);

            if (test_dist < temp_dist){
                dist = temp_dist;
                turn_point_index = i;
            }
        }
    }

    if (dist == -1){
        return 0;
    }else{
        if (dist > max_dist){
            return 0;
        }

        return map->turn_points[turn_point_index].orientation;
    }
}

// SINA: Return the value for junction feature based on the current pose
int map_see_junction(map_t *map, pf_vector_t pose, double delta_dist, double max_dist){
    double dist = -1;
    int junction_index = -1;
    for (int i = 0; i < map->junctions_count; i++){
        double temp_dist = pow((pose.v[0] - map->junctions[i].x), 2) + pow((pose.v[1] - map->junctions[i].y), 2);
        temp_dist = sqrt(temp_dist);

        if (dist == -1 || temp_dist < dist){
            pf_vector_t test_delta_pose;

            test_delta_pose.v[0] = cos(pose.v[2]) * delta_dist;
            test_delta_pose.v[1] = sin(pose.v[2]) * delta_dist;
            test_delta_pose.v[2] = 0.0;

            test_delta_pose = pf_vector_add(pose, test_delta_pose);

            double test_dist = pow((test_delta_pose.v[0] - map->junctions[i].x), 2) +
                    pow((test_delta_pose.v[1] - map->junctions[i].y), 2);

            test_dist = sqrt(test_dist);

            if (test_dist < temp_dist){
                dist = temp_dist;
                junction_index = i;
            }
        }
    }

    if (dist == -1){
        return 0;
    }else{
        if (dist > max_dist){
            return 0;
        }

        return map->junctions[junction_index].type;
    }
}

////////////////////////////////////////////////////////////////////////////
// SINA: Load feature properties from the file
void map_feature_load(map_t *map, const char *filename){

    printf("Loading features from ");
    printf(filename);
    printf("\n");

    if (!doesFileExist(filename)){
        printf("The feature cannot be loaded from the provided file!\n");
        return;
    }

    // File exists! Now let's load the features...

    map->cross_walks_count = 0;
    map->turn_points_count = 0;
    map->junctions_count = 0;

    FILE* fptr = fopen(filename, "r");
    char * line = NULL;
    size_t len = 0;
    ssize_t read;
    bool read_data = false;
    while ((read = getline(&line, &len, fptr)) != -1) {

        std::string line_string(line);
        vector<string> splitted;
        stringstream ssin(line_string);
        while (ssin.good()){
            string temp;
            ssin >> temp;
            splitted.push_back(temp);
        }
        splitted.pop_back();

        if (!read_data && splitted.size() != 0 && splitted[0] == "Features:"){
            read_data = true;
            continue;
        }
        if (!read_data){
            continue;
        }

        if (splitted[0] == "CrossWalk"){
            cross_walk_t cross_walk;
            cross_walk.x = atof(splitted[1].c_str());
            cross_walk.y = atof(splitted[2].c_str());

            map->cross_walks[map->cross_walks_count] = cross_walk;
            map->cross_walks_count++;
        }else if (splitted[0] == "TurnPoint"){
            turn_point_t turn_point;
            turn_point.x = atof(splitted[1].c_str());
            turn_point.y = atof(splitted[2].c_str());

            if (splitted[3] == "LEFT")
                turn_point.orientation = 1;
            else if (splitted[3] == "RIGHT")
                turn_point.orientation = 2;

            map->turn_points[map->turn_points_count] = turn_point;
            map->turn_points_count++;
        }else if (splitted[0] == "Junction"){
            junction_t junction;
            junction.x = atof(splitted[1].c_str());
            junction.y = atof(splitted[2].c_str());

            if (splitted[3] == "THREE")
                junction.type = 1;
            else if (splitted[3] == "FOUR")
                junction.type = 2;

            map->junctions[map->junctions_count] = junction;
            map->junctions_count++;
        }
    }

//    printf("\nMap Cross Walks: \n");
//    for (size_t i = 0; i < map->cross_walks_count; i++){
//        printf("Crosswalk %d:  %f\t%f\n", (int)i, map->cross_walks[i].x, map->cross_walks[i].y);
//    }

//    printf("\nTurn Points Walks: \n");
//    for (size_t i = 0; i < map->turn_points_count; i++){
//        printf("TurnPoint %d:  %f\t%f\t%d\n", (int)i, map->turn_points[i].x, map->turn_points[i].y,
//               map->turn_points[i].orientation);
//    }

//    printf("\nJunctions: \n");
//    for (size_t i = 0; i < map->junctions_count; i++){
//        printf("Junction %d:  %f\t%f\t%d\n", (int)i, map->junctions[i].x, map->junctions[i].y,
//               map->junctions[i].type);
//    }

    fclose(fptr);
    if (line) free(line);
}

int doesFileExist(const char* filename)
{
    FILE* fptr = fopen(filename, "r");
    if (fptr != NULL)
    {
        fclose(fptr);
        return 1;
    }
    return 0;
}

