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
///////////////////////////////////////////////////////////////////////////
//
// Desc: AMCL odometry routines
// Author: Andrew Howard
// Date: 6 Feb 2003
// CVS: $Id: amcl_odom.cc 7057 2008-10-02 00:44:06Z gbiggs $
//
///////////////////////////////////////////////////////////////////////////

#include <algorithm>

#include <sys/types.h> // required by Darwin
#include <math.h>
#include "Sampler.h"
#include "amcl_odom.h"

using namespace amcl;

// Added by KPM for nested particle motion
static double last_delta_trans = 0.0;

static double
normalize(double z)
{
    return atan2(sin(z),cos(z));
}
static double
angle_diff(double a, double b)
{
    double d1, d2;
    a = normalize(a);
    b = normalize(b);
    d1 = a-b;
    d2 = 2*M_PI - fabs(d1);
    if(d1 > 0)
        d2 *= -1.0;
    if(fabs(d1) < fabs(d2))
        return(d1);
    else
        return(d2);
}

////////////////////////////////////////////////////////////////////////////////
// Default constructor
AMCLOdom::AMCLOdom() : AMCLSensor()
{
    this->time = 0.0;
}

// Constructor with map parameter (Created by KPM for NPF)
AMCLOdom::AMCLOdom(map_t *map, MCHMM * hmm) : AMCLSensor()
{
    this->time = 0.0;
    this->map = map;
    this->hmm = hmm;
}

void
AMCLOdom::SetModelDiff(double alpha1,
                       double alpha2,
                       double alpha3,
                       double alpha4)
{
    this->model_type = ODOM_MODEL_DIFF;
    this->alpha1 = alpha1;
    this->alpha2 = alpha2;
    this->alpha3 = alpha3;
    this->alpha4 = alpha4;
}

void
AMCLOdom::SetModelOmni(double alpha1,
                       double alpha2,
                       double alpha3,
                       double alpha4,
                       double alpha5)
{
    this->model_type = ODOM_MODEL_OMNI;
    this->alpha1 = alpha1;
    this->alpha2 = alpha2;
    this->alpha3 = alpha3;
    this->alpha4 = alpha4;
    this->alpha5 = alpha5;
}


////////////////////////////////////////////////////////////////////////////////
// Apply the action model
bool AMCLOdom::UpdateAction(pf_t *pf, AMCLSensorData *data)
{
    AMCLOdomData *ndata;
    ndata = (AMCLOdomData*) data;

    // Compute the new sample poses
    pf_sample_set_t *set;

    set = pf->sets + pf->current_set;
    pf_vector_t old_pose = pf_vector_sub(ndata->pose, ndata->delta);

    double delta_trans = 0.0;

    if(this->model_type == ODOM_MODEL_OMNI)
    {
        double delta_rot, delta_bearing;
        double delta_trans_hat, delta_rot_hat, delta_strafe_hat;

        delta_trans = sqrt(ndata->delta.v[0]*ndata->delta.v[0] +
                ndata->delta.v[1]*ndata->delta.v[1]);
        delta_rot = ndata->delta.v[2];

        // Precompute a couple of things
        double trans_hat_stddev = (alpha3 * (delta_trans*delta_trans) +
                                   alpha1 * (delta_rot*delta_rot));
        double rot_hat_stddev = (alpha4 * (delta_rot*delta_rot) +
                                 alpha2 * (delta_trans*delta_trans));
        double strafe_hat_stddev = (alpha1 * (delta_rot*delta_rot) +
                                    alpha5 * (delta_trans*delta_trans));

        for (int i = 0; i < set->sample_count; i++)
        {
            pf_sample_t* sample = set->samples + i;

            delta_bearing = angle_diff(atan2(ndata->delta.v[1], ndata->delta.v[0]),
                    old_pose.v[2]) + sample->pose.v[2];
            double cs_bearing = cos(delta_bearing);
            double sn_bearing = sin(delta_bearing);

            // Sample pose differences
            delta_trans_hat = delta_trans + pf_ran_gaussian(trans_hat_stddev);
            delta_rot_hat = delta_rot + pf_ran_gaussian(rot_hat_stddev);
            delta_strafe_hat = 0 + pf_ran_gaussian(strafe_hat_stddev);
            // Apply sampled update to particle pose
            sample->pose.v[0] += (delta_trans_hat * cs_bearing +
                                  delta_strafe_hat * sn_bearing);
            sample->pose.v[1] += (delta_trans_hat * sn_bearing -
                                  delta_strafe_hat * cs_bearing);
            sample->pose.v[2] += delta_rot_hat ;
            sample->weight = 1.0 / set->sample_count;
        }
    }
    else //(this->model_type == ODOM_MODEL_DIFF)
    {
        // Implement sample_motion_odometry (Prob Rob p 136)
        double delta_rot1, delta_rot2;
        double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
        double delta_rot1_noise, delta_rot2_noise;

        // Avoid computing a bearing from two poses that are extremely near each
        // other (happens on in-place rotation).
        if(sqrt(ndata->delta.v[1]*ndata->delta.v[1] +
                ndata->delta.v[0]*ndata->delta.v[0]) < 0.01)
            delta_rot1 = 0.0;
        else
            delta_rot1 = angle_diff(atan2(ndata->delta.v[1], ndata->delta.v[0]),
                    old_pose.v[2]);
        delta_trans = sqrt(ndata->delta.v[0]*ndata->delta.v[0] +
                ndata->delta.v[1]*ndata->delta.v[1]);
        delta_rot2 = angle_diff(ndata->delta.v[2], delta_rot1);

        // We want to treat backward and forward motion symmetrically for the
        // noise model to be applied below.  The standard model seems to assume
        // forward motion.
        delta_rot1_noise = std::min(fabs(angle_diff(delta_rot1,0.0)),
                                    fabs(angle_diff(delta_rot1,M_PI)));
        delta_rot2_noise = std::min(fabs(angle_diff(delta_rot2,0.0)),
                                    fabs(angle_diff(delta_rot2,M_PI)));

        for (int i = 0; i < set->sample_count; i++)
        {
            pf_sample_t* sample = set->samples + i;

            // Sample pose differences
            delta_rot1_hat = angle_diff(delta_rot1,
                                        pf_ran_gaussian(this->alpha1*delta_rot1_noise*delta_rot1_noise +
                                                        this->alpha2*delta_trans*delta_trans));
            delta_trans_hat = delta_trans -
                    pf_ran_gaussian(this->alpha3*delta_trans*delta_trans +
                                    this->alpha4*delta_rot1_noise*delta_rot1_noise +
                                    this->alpha4*delta_rot2_noise*delta_rot2_noise);
            delta_rot2_hat = angle_diff(delta_rot2,
                                        pf_ran_gaussian(this->alpha1*delta_rot2_noise*delta_rot2_noise +
                                                        this->alpha2*delta_trans*delta_trans));

            // Apply sampled update to particle pose
            sample->pose.v[0] += delta_trans_hat *
                    cos(sample->pose.v[2] + delta_rot1_hat);
            sample->pose.v[1] += delta_trans_hat *
                    sin(sample->pose.v[2] + delta_rot1_hat);
            sample->pose.v[2] += delta_rot1_hat + delta_rot2_hat;
            sample->weight = 1.0 / set->sample_count;
        }
    }

    if(pf->nesting_lvl > 0){
        //AMCLOdomData *nested_odomData;
        pf_t *nested_pf_set, *nested_pf_sample;

        nested_pf_set = pf_get_this_nested_set(pf, pf->current_set);

        //nested_odomData->pose = pf->fake_nested_odomPose;
        //nested_odomData->delta = pf->fake_nested_odomDelta;

        //getNestedParticlePose(&nested_odomData->pose, &nested_odomData->delta);

        for(int i=0; i< set->sample_count; i++){
            nested_pf_sample = nested_pf_set + i;

            this->UpdateNestedAction(nested_pf_sample,
                                     ndata->nested_velocity,
                                     ndata->velocity_angle_diff,
                                     data->time
                                     );
        }
    }
    return true;
}


bool AMCLOdom::UpdateNestedAction(pf_t *pf, pf_vector_t vel, double correction_angle, double time){
    // AMCLOdomData *ndata =
    //ndata = (AMCLOdomData*) data;

    pf_vector_t delta = pf_vector_zero();

    // Compute the new sample poses
    pf_sample_set_t *set;

    set = pf->sets + pf->current_set;
    // pf_vector_t old_pose = pf_vector_sub(ndata->pose, ndata->delta);

    // This is the linear transition estimation
    double delta_trans = -1;
    // This is the rotation estiamtion
    double delta_rot = -1;

    for (int i = 0; i < set->sample_count; i++)
    {
        pf_sample_t* sample = set->samples + i;
        pf_vector_t old_pose = sample->pose;

        // get the new pose and delta in these passed arguments
        getNestedParticlePose(&sample->pose, &delta, vel, correction_angle, time);

        delta_trans = sqrt(delta.v[0] * delta.v[0] + delta.v[1] * delta.v[1]);
        delta_rot   = delta.v[2];

        pf->fake_nested_odomPose = sample->pose;
        pf->fake_nested_odomDelta = delta;

        if(this->model_type == ODOM_MODEL_OMNI)
        {
            double delta_bearing;
            double delta_trans_hat, delta_rot_hat, delta_strafe_hat;

            // Precompute a couple of things
            double trans_hat_stddev = (alpha3 * (delta_trans * delta_trans) +
                                       alpha1 * (delta_rot * delta_rot));
            double rot_hat_stddev = (alpha4 * (delta_rot * delta_rot) +
                                     alpha2 * (delta_trans * delta_trans));
            double strafe_hat_stddev = (alpha1 * (delta_rot * delta_rot) +
                                        alpha5 * (delta_trans * delta_trans));

            { // calculations specific to the OMNI model
                delta_bearing = angle_diff(atan2(delta.v[1], delta.v[0]),
                        old_pose.v[2]) + sample->pose.v[2];
                double cs_bearing = cos(delta_bearing);
                double sn_bearing = sin(delta_bearing);

                // Sample pose differences
                delta_trans_hat = delta_trans + pf_ran_gaussian(trans_hat_stddev);
                delta_rot_hat = delta_rot + pf_ran_gaussian(rot_hat_stddev);
                delta_strafe_hat = 0 + pf_ran_gaussian(strafe_hat_stddev);
                // Apply sampled update to particle pose
                sample->pose.v[0] += (delta_trans_hat * cs_bearing +
                                      delta_strafe_hat * sn_bearing);
                sample->pose.v[1] += (delta_trans_hat * sn_bearing -
                                      delta_strafe_hat * cs_bearing);
                sample->pose.v[2] += delta_rot_hat ;
                sample->weight = 1.0 / set->sample_count;
            }
        }
        else //(this->model_type == ODOM_MODEL_DIFF)
        {
            // Implement sample_motion_odometry (Prob Rob p 136)
            double delta_rot1, delta_rot2;
            double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
            double delta_rot1_noise, delta_rot2_noise;

            // Avoid computing a bearing from two poses that are extremely near each
            // other (happens on in-place rotation).
            if(sqrt(delta.v[1]*delta.v[1] + delta.v[0]*delta.v[0]) < 0.01)
                delta_rot1 = 0.0;
            else
                delta_rot1 = angle_diff(atan2(delta.v[1], delta.v[0]), old_pose.v[2]);
            delta_trans = sqrt(delta.v[0]*delta.v[0] + delta.v[1]*delta.v[1]);
            delta_rot2 = angle_diff(delta.v[2], delta_rot1);

            // We want to treat backward and forward motion symmetrically for the
            // noise model to be applied below.  The standard model seems to assume
            // forward motion.
            delta_rot1_noise = std::min(fabs(angle_diff(delta_rot1,0.0)),
                                        fabs(angle_diff(delta_rot1,M_PI)));
            delta_rot2_noise = std::min(fabs(angle_diff(delta_rot2,0.0)),
                                        fabs(angle_diff(delta_rot2,M_PI)));

            { // calculations specific to the DIFF model

                // Sample pose differences
                delta_rot1_hat = angle_diff(delta_rot1,
                                            pf_ran_gaussian(this->alpha1*delta_rot1_noise*delta_rot1_noise +
                                                            this->alpha2*delta_trans*delta_trans));
                delta_trans_hat = delta_trans -
                        pf_ran_gaussian(this->alpha3*delta_trans*delta_trans +
                                        this->alpha4*delta_rot1_noise*delta_rot1_noise +
                                        this->alpha4*delta_rot2_noise*delta_rot2_noise);
                delta_rot2_hat = angle_diff(delta_rot2,
                                            pf_ran_gaussian(this->alpha1*delta_rot2_noise*delta_rot2_noise +
                                                            this->alpha2*delta_trans*delta_trans));

                // Apply sampled update to particle pose
                sample->pose.v[0] += delta_trans_hat *
                        cos(sample->pose.v[2] + delta_rot1_hat);
                sample->pose.v[1] += delta_trans_hat *
                        sin(sample->pose.v[2] + delta_rot1_hat);
                sample->pose.v[2] += delta_rot1_hat + delta_rot2_hat;
                sample->weight = 1.0 / set->sample_count;

                //                sample->pose.v[0] += delta.v[0];
                //                sample->pose.v[1] += delta.v[1];
                //                sample->pose.v[2] += delta.v[2];
                //                sample->weight = 1.0 / set->sample_count;
            }
        }
    } // end for

    if(pf->nesting_lvl > 0){
        pf_t *nested_pf_set, *nested_pf_sample;

        nested_pf_set = pf_get_this_nested_set(pf, pf->current_set);

        for(int i=0; i< set->sample_count; i++){
            nested_pf_sample = nested_pf_set + i;
            this->UpdateNestedAction(nested_pf_sample, vel, correction_angle, time);
        }
    }
    return true;
}


// SINA: This method will propagate the odom
void AMCLOdom::getNestedParticlePose(pf_vector_t *odom_pose, pf_vector_t *delta, pf_vector_t vel, double correction_angle,
                                     double time){
    map_cell_t * map_cell = map_get_cell(this->map, odom_pose->v[0], odom_pose->v[1], odom_pose->v[2]);

    double delta_ = sqrt(vel.v[0] * vel.v[0] + vel.v[1] * vel.v[1]);
    double correction = pf_vector_angle(vel);

    delta_ *= time;

    double dice = drand48() * 100;
    double map_range = map_calc_range(this->map, odom_pose->v[0], odom_pose->v[1], odom_pose->v[2], 10);
    if(map_range < .5){
        if(dice <= 50){
            delta->v[0] = 0.00;
            delta->v[1] = 0.00;
            delta->v[2] = (M_PI / 5);
        }

        else{
            delta->v[0] = 0.00;
            delta->v[1] = 0.00;
            delta->v[2] = -(M_PI / 5);
        }

        dice = drand48() * 100;
        if (dice < 50){
            // To get out of deadlocks that might happen in small rooms!
            delta->v[0] = std::cos(delta->v[2]) * delta_;
            delta->v[1] = std::sin(delta->v[2]) * delta_;
        }
    }else if (map_cell->occ_state >= 0){ // If the particle is on the occupied or unknown cells of the map
        double* walls = map_side_walls(this->map, *odom_pose, 3.0);

        int direction = 0;
        if (walls[0] < walls[1]){  // The near wall is on the left side
            direction = -1;
        }else if (walls[1] < walls[0]){
            direction = 1;
        }else{
            double dice = drand48() * 100;
            if (dice < 50){
                direction = 1;
            }else{
                direction = -1;
            }
        }

        double recovery_turn = correction_angle * 2 * direction;
        delta->v[0] = std::cos(odom_pose->v[2] + recovery_turn) * delta_;
        delta->v[1] = std::sin(odom_pose->v[2] + recovery_turn) * delta_;
        delta->v[2] = recovery_turn;
    } else {
        double offset = (drand48() * M_PI / 2) - (M_PI / 4);
        delta->v[0] = std::cos(correction) * delta_;
        delta->v[1] = std::sin(correction) * delta_;
        delta->v[2] = correction - odom_pose->v[2] + offset;
    }
}
