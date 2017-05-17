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
 * Desc: Simple particle filter for localization.
 * Author: Andrew Howard
 * Date: 10 Dec 2002
 * CVS: $Id: pf.c 6345 2008-04-17 01:36:39Z gerkey $
 *************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

#include "pf.h"
#include "pf_pdf.h"
#include "pf_kdtree.h"

//#include "ros/ros.h"

#define DUAL_MCL 0
#define NESTED_DUAL 1

// Compute the required number of samples, given that there are k bins
// with samples in them.
static int pf_resample_limit(pf_t *pf, int k);

// Re-compute the cluster statistics for a sample set
static void pf_cluster_stats(pf_sample_set_t *set);

// Copy one filter into another
static void pf_copy(pf_t *pf_source, pf_t *pf_dest);

// Normalize all the weights inside given pf using the total provided
static void normalize_weights(double total, pf_t* pf);

// Create a new filter
pf_t *pf_alloc(int min_samples, int max_samples,
               double alpha_slow, double alpha_fast,
               pf_init_model_fn_t random_pose_fn,
               pf_dual_model_fn_t dual_pose_fn, //Added by KPM
               void *random_pose_data,

               //Added by KPM
               int nesting_level,
               int min_nested_samples, int max_nested_samples
               )
{
    int i, j;
    pf_t *pf;
    pf_sample_set_t *set;
    pf_sample_t *sample;
    pf_t *nested_pf_set;
    pf_t *nested_pf;

    srand48(time(NULL));

    pf = calloc(1, sizeof(pf_t));

    pf->random_pose_fn = random_pose_fn;
    pf->random_pose_data = random_pose_data;

    //Added by KPM
    pf->dual_pose_fn = dual_pose_fn;
    pf->nesting_lvl = nesting_level;
    pf->isNested = 0;


    pf->min_samples = min_samples;
    pf->max_samples = max_samples;

    pf->min_nested_samples = 0;
    pf->max_nested_samples = 0;

    // Control parameters for the population size calculation.  [err] is
    // the max error between the true distribution and the estimated
    // distribution.  [z] is the upper standard normal quantile for (1 -
    // p), where p is the probability that the error on the estimated
    // distrubition will be less than [err].
    pf->pop_err = 0.01;
    pf->pop_z = 3;

    pf->current_set = 0;

    // Set initial fake odom pose and delta for the nested particles
    if(nesting_level > 0){
        pf->fake_nested_odomPose = pf_vector_zero();
        pf->fake_nested_odomDelta = pf_vector_zero();
    }

    for (j = 0; j < 2; j++)
    {
        set = pf->sets + j;

        set->sample_count = max_samples;
        set->samples = calloc(max_samples, sizeof(pf_sample_t));

        // Allocate memory to the fake nested pf sets if present.
        if(nesting_level > 0){
            if(j == 0){
                pf->nested_pf_set_0 = calloc(pf->max_samples, sizeof(pf_t));
            }
            else{
                pf->nested_pf_set_1 = calloc(pf->max_samples, sizeof(pf_t));
            }
            nested_pf_set = pf_get_this_nested_set(pf,j);
            //nested_pf_set = calloc(max_samples, sizeof(pf_t));
        }

        for (i = 0; i < set->sample_count; i++)
        {
            sample = set->samples + i;
            sample->pose.v[0] = 0.0;
            sample->pose.v[1] = 0.0;
            sample->pose.v[2] = 0.0;
            sample->weight = 1.0 / max_samples;


            // Also initialize the nested particle filters if present
            if(nesting_level > 0){

                nested_pf = nested_pf_set + i;

                nested_pf->max_samples = pf->max_nested_samples; // This is initialized to 0 for now

                //Removing this in order to implement Adaptive NPF
                /*
                pf_nested_alloc(nested_pf, min_nested_samples, max_nested_samples,
                                alpha_slow, alpha_fast,
                                random_pose_fn,
                                dual_pose_fn, //Added by KPM
                                random_pose_data,
                                //Added by KPM
                                (nesting_level-1),
                                0, 0
                                );
                */

            }

        }

        // HACK: is 3 times max_samples enough?
        set->kdtree = pf_kdtree_alloc(3 * max_samples);

        set->cluster_count = 0;
        set->cluster_max_count = max_samples;
        set->clusters = calloc(set->cluster_max_count, sizeof(pf_cluster_t));

        set->mean = pf_vector_zero();
        set->cov = pf_matrix_zero();
    }

    pf->w_slow = 0.0;
    pf->w_fast = 0.0;

    pf->alpha_slow = alpha_slow;
    pf->alpha_fast = alpha_fast;

    return pf;
}

// Create a new filter
void pf_nested_alloc(pf_t *pf, int min_samples, int max_samples,
                     double alpha_slow, double alpha_fast,
                     pf_init_model_fn_t random_pose_fn,
                     pf_dual_model_fn_t dual_pose_fn, //Added by KPM
                     void *random_pose_data,

                     //Added by KPM
                     int nesting_level,
                     int min_nested_samples, int max_nested_samples
                     )
{
    int i, j;
    //pf_t *pf;
    pf_sample_set_t *set;
    pf_sample_t *sample;

    srand48(time(NULL));

    //pf = calloc(1, sizeof(pf_t));

    pf->random_pose_fn = random_pose_fn;
    pf->random_pose_data = random_pose_data;

    //Added by KPM
    pf->dual_pose_fn = dual_pose_fn;
    pf->nesting_lvl = nesting_level;
    pf->isNested = 1;


    pf->min_samples = min_samples;
    pf->max_samples = max_samples;

    // Control parameters for the population size calculation.  [err] is
    // the max error between the true distribution and the estimated
    // distribution.  [z] is the upper standard normal quantile for (1 -
    // p), where p is the probability that the error on the estimated
    // distrubition will be less than [err].
    pf->pop_err = 0.01;
    pf->pop_z = 3;

    pf->current_set = 0;

    // Set initial fake odom pose and delta for the nested particles
    if(nesting_level > 0){
        pf->fake_nested_odomPose = pf_vector_zero();
        pf->fake_nested_odomDelta = pf_vector_zero();
    }

    for (j = 0; j < 2; j++)
    {
        set = pf->sets + j;

        set->sample_count = max_samples;
        set->samples = calloc(max_samples, sizeof(pf_sample_t));

        for (i = 0; i < set->sample_count; i++)
        {
            sample = set->samples + i;
            sample->pose.v[0] = 0.0;
            sample->pose.v[1] = 0.0;
            sample->pose.v[2] = 0.0;
            sample->weight = 1.0 / max_samples;
        }

        // HACK: is 3 times max_samples enough?
        set->kdtree = pf_kdtree_alloc(3 * max_samples);

        set->cluster_count = 0;
        set->cluster_max_count = max_samples;
        set->clusters = calloc(set->cluster_max_count, sizeof(pf_cluster_t));

        set->mean = pf_vector_zero();
        set->cov = pf_matrix_zero();
    }

    pf->w_slow = 0.0;
    pf->w_fast = 0.0;

    pf->alpha_slow = alpha_slow;
    pf->alpha_fast = alpha_fast;

    return;
}


// Free an existing filter, and any lower level nested filters within it
void pf_free(pf_t *pf)
{

    if(pf != NULL){
        pf_t *nested_pf_set;
        pf_t *nested_pf;
        int i;

        for (i = 0; i < 2; i++)
        {
            if(pf->sets[i].clusters != NULL){
                free(pf->sets[i].clusters);
            }
            if(pf->sets[i].kdtree != NULL){
                pf_kdtree_free(pf->sets[i].kdtree);
            }
            if(pf->sets[i].samples != NULL){
                free(pf->sets[i].samples);
            }

            // Frees up all lower level filters too
            if(pf->nesting_lvl > 0 && pf->max_nested_samples > 0){

                nested_pf_set = pf_get_this_nested_set(pf,i);
                int j;
                for(j = 0; j < pf->max_samples; j++){
                    nested_pf = nested_pf_set + j;
                    pf_free(nested_pf);
                }

                free(nested_pf_set);
            }

        }
        free(pf);
    }

    return;
}


// Initialize the filter using a guassian
void pf_init(pf_t *pf, pf_vector_t mean, pf_matrix_t cov, map_t* map)
{
    int i;
    pf_sample_set_t *set;
    pf_sample_t *sample;
    pf_pdf_gaussian_t *pdf;

    set = pf->sets + pf->current_set;

    // Create the kd tree for adaptive sampling
    pf_kdtree_clear(set->kdtree);

    set->sample_count = pf->max_samples;

    pdf = pf_pdf_gaussian_alloc(mean, cov);

    // Compute the new sample poses
    for (i = 0; i < set->sample_count; i++)
    {
        sample = set->samples + i;
        sample->weight = 1.0 / pf->max_samples;

        // Switching back to original gaussian initial distribution for non-nested particles
        if(mean.v[0] == 0 && mean.v[1] == 0 && mean.v[2] == 0 && pf->isNested != 0){
            //KPM: using uniform particle generation for initialization
            //SINA: After the first time, add the particles in the freespace, not around the leader initial pose estimation
            sample->pose =(pf->random_pose_fn)(map);
        }else{
            //This was original gaussian sampling for initialization

            int sample_x = 0, sample_y = 0;

            do{ // keep getting new poses until we get a valid pose in a free cell
                sample->pose = pf_pdf_gaussian_sample(pdf);

                sample_x = MAP_GXWX(map, sample->pose.v[0]);
                sample_y = MAP_GYWY(map, sample->pose.v[1]);

            }while( ( !MAP_VALID(map, sample_x, sample_y)
                      || (map->cells[MAP_INDEX(map, sample_x, sample_y)].occ_state > -1) ));
        }

        // no covariance ...directly initialize all particles to the exact pose specified by user (or initial pose)
        //sample->pose = mean;

        // Add sample to histogram
        pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);
    }

    pf->w_slow = pf->w_fast = 0.0;

    pf_pdf_gaussian_free(pdf);

    // Re-compute cluster statistics
    pf_cluster_stats(set);


    return;
}


// Initialize the filter using some model.
// (The only time this is being used right now is to initialize using the uniform pose generator)
void pf_init_model(pf_t *pf, pf_init_model_fn_t init_fn, void *init_data)
{
    int i;
    pf_sample_set_t *set;
    pf_sample_t *sample;

    set = pf->sets + pf->current_set;

    // Create the kd tree for adaptive sampling
    pf_kdtree_clear(set->kdtree);

    set->sample_count = pf->max_samples;

    // Compute the new sample poses
    for (i = 0; i < set->sample_count; i++)
    {
        sample = set->samples + i;
        sample->weight = 1.0 / pf->max_samples;
        sample->pose = (*init_fn) (init_data);

        // Add sample to histogram
        pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);
    }

    pf->w_slow = pf->w_fast = 0.0;

    // Re-compute cluster statistics
    pf_cluster_stats(set);

    return;
}


// Update the filter with some new action
void pf_update_action(pf_t *pf, pf_action_model_fn_t action_fn, void *action_data)
{
    pf_sample_set_t *set;

    set = pf->sets + pf->current_set;

    (*action_fn) (action_data, set);

    return;
}


#include <float.h>
// Update the filter with some new sensor observation
void pf_update_sensor(pf_t *pf, pf_sensor_model_fn_t sensor_fn, void *sensor_data)
{
    pf_sample_set_t *set;
    //pf_sample_t *sample;
    double total;

    set = pf->sets + pf->current_set;

    // Compute the sample weights
    total = (*sensor_fn) (sensor_data, set);

    normalize_weights(total, pf);

    return;
}


// Update the filter with some new sensor observation
void pf_update_nested_sensor(pf_t *pf,
                             pf_sensor_AW_model_fn_t sensor_fn,
                             pf_nested_sensor_model_fn_t nested_sensor_fn,
                             void *sensor_data)
{
    pf_sample_set_t *set;
    pf_sample_t *sample;
    pf_t *nested_pf_set;
    double total;

    set = pf->sets + pf->current_set;
    nested_pf_set = pf_get_this_nested_set(pf, pf->current_set);

    // Compute sample weights for Nested particles
    if(pf->nesting_lvl > 0 && pf->max_nested_samples > 0){
        int i = 0;
        double nested_total = 0;
        pf_t *nested_pf;
        pf_sample_set_t nested_set;

        //nested_pf_set = pf_get_this_nested_set(pf, pf->current_set);

        for(i=0; i<set->sample_count; i++){
            // This sample is used as a reference for the nested particles!
            sample = set->samples + i;

            // This is the nested particle set that is assigned to the above sample
            nested_pf = nested_pf_set + i;

            if(nested_pf->max_samples > 0){
                nested_set = nested_pf->sets[nested_pf->current_set]; //TODO KPM: this should be [nested_pf->current_set]
                // Compute the weights for the nested set
                nested_total = (*nested_sensor_fn) (sample, sensor_data, &nested_set);
                // Normalize the weights for the nested set
                normalize_weights(nested_total, nested_pf);
            }
        }
    }

    // Compute the sample weights for !NORMAL! particles
    total = (*sensor_fn) (sensor_data, set, nested_pf_set);

    normalize_weights(total, pf);
    
    return;
}


static void normalize_weights(double total, pf_t* pf){

    int i;
    pf_sample_set_t *set;
    pf_sample_t *sample;

    set = pf->sets + pf->current_set;
    if (total > 0.0)
    {
        // Normalize weights
        double w_avg=0.0;
        for (i = 0; i < set->sample_count; i++)
        {
            sample = set->samples + i;
            w_avg += sample->weight;
            sample->non_normalized_weight = sample->weight;
            sample->weight /= total;
        }
        // Update running averages of likelihood of samples (Prob Rob p258)
        w_avg /= set->sample_count;
        set->avg_weight = w_avg;

        if(pf->w_slow == 0.0)
            pf->w_slow = w_avg;
        else
            pf->w_slow += pf->alpha_slow * (w_avg - pf->w_slow);
        if(pf->w_fast == 0.0)
            pf->w_fast = w_avg;
        else
            pf->w_fast += pf->alpha_fast * (w_avg - pf->w_fast);
        //printf("w_avg: %e slow: %e fast: %e\n",
        //w_avg, pf->w_slow, pf->w_fast);
    }
    else
    {
        // Handle zero total
        for (i = 0; i < set->sample_count; i++)
        {
            sample = set->samples + i;
            sample->weight = 1.0 / set->sample_count;
        }
    }
}

// Resample the distribution
void pf_update_resample(pf_t *pf, double landmark_r, double landmark_phi, double landmark_x, double landmark_y,
                        pf_vector_t leader_mean_, pf_matrix_t leader_cov_, pf_vector_t leader_pose,
                        pf_vector_t leader_vel)
{
    int i;
    double total;
    pf_sample_set_t *set_a, *set_b;
    pf_sample_t *sample_a, *sample_b;

    pf_t *pf_nested_set_a,*pf_nested_set_b;
    pf_t *pf_sample_a, *pf_sample_b;

    double r,c,U;
    int m;
    double count_inv;

    double w_diff;

    set_a = pf->sets + pf->current_set;
    set_b = pf->sets + (pf->current_set + 1) % 2;

    if(pf->nesting_lvl > 0){
        pf_nested_set_a = pf_get_this_nested_set(pf, pf->current_set);
        pf_nested_set_b = pf_get_this_nested_set(pf, (pf->current_set +1) % 2);
    }

    // Create the kd tree for adaptive sampling
    // pf_kdtree_clear(set_b->kdtree);

    // Draw samples from set a to create set b.
    total = 0;
    set_b->sample_count = 0;

    w_diff = 1.0 - pf->w_fast / pf->w_slow;
    if(w_diff < 0.0)
        w_diff = 0.0;
    //printf("w_diff: %9.6f\n", w_diff);

    int M = pf_resample_limit(pf, set_a->kdtree->leaf_count);
    //    printf("M: %i\n", M);


    // ********
    // Re-Initializing set_b related stuff since the memory allocations are changing due to a new 'M'

    // Free all pf related memory from any earlier iteration (set_b might have been used
    // before...especially in pf_init)
    if(set_b->clusters != NULL){
        free(set_b->clusters);
        set_b->clusters = NULL;
    }
    if(set_b->kdtree != NULL){
        pf_kdtree_free(set_b->kdtree);
        set_b->kdtree = NULL;
    }
    if(set_b->samples != NULL){
        free(set_b->samples);
        set_b->samples = NULL;
    }

    // Allocate new kdtree for the set_b particle set: "set_b" (the next set, not current set)
    // HACK: is 3 times max_samples enough?
    set_b->kdtree = pf_kdtree_alloc(3 * M);

    // new clusters
    set_b->cluster_count = 0;
    set_b->cluster_max_count = M;
    set_b->clusters = calloc(set_b->cluster_max_count, sizeof(pf_cluster_t));

    set_b->mean = set_a->mean;
    set_b->cov = set_a->cov;

    // new samples (with new "M" count...could be either higher or lower than before, doesn't matter)
    set_b->samples = calloc(M, sizeof(pf_sample_t));


    // Create the kd tree for adaptive sampling
    pf_kdtree_clear(set_b->kdtree);

    // ********

    // Low-variance resampler, taken from Probabilistic Robotics, p110
    count_inv = 1.0 / M;
    r = drand48() * count_inv;
    c = set_a->samples[0].weight;
    i = 0;
    m = 0;

    while(set_b->sample_count < M)
    {
        if(pf->nesting_lvl > 0){
            pf_sample_b = pf_nested_set_b + set_b->sample_count;
        }
        sample_b = set_b->samples + set_b->sample_count++;

        //    if(drand48() < w_diff){

        if((pf->isNested > 0) && drand48() < 0.05){
#if NESTED_DUAL
            if(landmark_r > 0){
                //KPM..this is the original random sampling function
                //sample_b->pose = (pf->random_pose_fn)(pf->random_pose_data);

                //KPM ...trying dual sampling instead of random sampling
                sample_b->pose = (pf->dual_pose_fn)(pf->random_pose_data,
                                                    landmark_r, landmark_phi,
                                                    landmark_x, landmark_y);
            }
            else{
                sample_b->pose = (pf->random_pose_fn)(pf->random_pose_data);
            }
#else
            sample_b->pose = (pf->random_pose_fn)(pf->random_pose_data);
#endif
        }else{
            if(drand48() < w_diff){ // Recovery for normal particles
                sample_b->pose = (pf->random_pose_fn)(pf->random_pose_data);
            }

            else
            {
                // Low-variance resampler, taken from Probabilistic Robotics, p110
                U = r + m * count_inv;
                while(U>c)
                {
                    i++;
                    // Handle wrap-around by resetting counters and picking a new random
                    // number
                    if(i >= set_a->sample_count)
                    {
                        r = drand48() * count_inv;
                        c = set_a->samples[0].weight;
                        i = 0;
                        m = 0;
                        U = r + m * count_inv;
                        continue;
                    }
                    c += set_a->samples[i].weight;
                }
                m++;

                assert(i<set_a->sample_count);

                sample_a = set_a->samples + i;
                if(pf->nesting_lvl > 0){
                    pf_sample_a = pf_nested_set_a + i;
                }

                assert(sample_a->weight > 0);

                // Add sample to list
                sample_b->pose = sample_a->pose;

                // Copy nested pf if nested particles more than 0
                if(pf->nesting_lvl > 0 && pf->max_nested_samples > 0){
                    pf_copy(pf_sample_a, pf_sample_b);
                    //pf_update_nested_resample(pf_sample_b, landmark_r, landmark_phi, sample_b->pose);
                }

            }
        }

        sample_b->weight = 1.0;
        total += sample_b->weight;

        // Add sample to histogram
        pf_kdtree_insert(set_b->kdtree, sample_b->pose, sample_b->weight);

    } // End looping through for resampling

    int old_max_nested_samples = pf->max_nested_samples;
    pf->max_nested_samples = (int)((pf->max_samples - set_b->sample_count) / set_b->sample_count);

    //Resample all nested particles (if particles exist)

    if(pf->nesting_lvl > 0){

        if(pf->max_nested_samples > 0){

            int sample_counter = 0;

            while(sample_counter < set_b->sample_count){

                sample_b = set_b->samples + sample_counter;
                pf_sample_b = pf_nested_set_b + sample_counter;

                pf_sample_b->max_samples = pf->max_nested_samples;

                // allocate and initiate from scratch if no nested particles before this
                if(old_max_nested_samples <= 0 ){
                    pf_nested_alloc(pf_sample_b,
                                    0,
                                    pf->max_nested_samples,
                                    pf->alpha_slow,
                                    pf->alpha_fast,
                                    pf->random_pose_fn,
                                    pf->dual_pose_fn,
                                    pf->random_pose_data,
                                    pf->nesting_lvl-1,
                                    0,0);
                    pf_init(pf_sample_b,
                            leader_mean_,
                            leader_cov_,
                            pf->random_pose_data
                            );
                } else { // When Nested particles exist before this
                    pf_update_nested_adaptive_resample(pf_sample_b, landmark_r, landmark_phi, sample_b->pose, leader_pose
                                                       , leader_vel);
                }

                sample_counter++;
            }
        } else { // when current max_nested_particles is 0 or less than 0

            int counter=0;

            // free all memory related to this set of pf samples since we do not need it right now.
            // It is going to e reallocated and re-initiated if and when we need it later anyways.
            for(counter = 0; counter< set_b->sample_count; counter++){

                pf_sample_b = pf_nested_set_b + counter;

                int j;
                for (j = 0; j < 2; j++)
                {
                    if(pf_sample_b->sets[j].clusters != NULL){
                        free(pf_sample_b->sets[j].clusters);
                        pf_sample_b->sets[j].clusters = NULL;
                    }
                    if(pf_sample_b->sets[j].kdtree != NULL){
                        pf_kdtree_free(pf_sample_b->sets[j].kdtree);
                        pf_sample_b->sets[j].kdtree = NULL;
                    }
                    if(pf_sample_b->sets[j].samples != NULL){
                        free(pf_sample_b->sets[j].samples);
                        pf_sample_b->sets[j].samples = NULL;
                    }
                }
            }
        }

    }// End Resampling of Nested Particles

    //Free all memory related to pf_nested_set_a
    if(pf->nesting_lvl>0){
        int counter=0;

        for(counter = 0; counter< set_a->sample_count; counter++){

            pf_sample_a = pf_nested_set_a + counter;
            /*
            if(pf_sample_a->sets[0].samples != NULL)
                free(pf_sample_a->sets[0].samples);

            if(pf_sample_a->sets[1].samples != NULL)
                free(pf_sample_a->sets[1].samples);

            */

            int j;
            for (j = 0; j < 2; j++)
            {
                if(pf_sample_a->sets[j].clusters != NULL){
                    free(pf_sample_a->sets[j].clusters);
                }
                if(pf_sample_a->sets[j].kdtree != NULL){
                    pf_kdtree_free(pf_sample_a->sets[j].kdtree);
                }
                if(pf_sample_a->sets[j].samples != NULL){
                    free(pf_sample_a->sets[j].samples);
                }
            }
        }
    }


    // **************
    // Memory deallocation for normal particles in set_a
    // Free all memory for set_a from current iteration, since it will get reallocated later
    if(set_a->clusters != NULL){
        free(set_a->clusters);
        set_a->clusters = NULL;
    }
    if(set_a->kdtree != NULL){
        pf_kdtree_free(set_a->kdtree);
        set_a->kdtree = NULL;
    }
    if(set_a->samples != NULL){
        free(set_a->samples);
        set_a->samples = NULL;
    }

    // **************




    // Reset averages, to avoid spiraling off into complete randomness.
    if(w_diff > 0.0)
        pf->w_slow = pf->w_fast = 0.0;

    //fprintf(stderr, "\n\n");

    // Normalize weights
    for (i = 0; i < set_b->sample_count; i++)
    {
        sample_b = set_b->samples + i;
        sample_b->weight /= total;
    }

    // Re-compute cluster statistics
    pf_cluster_stats(set_b);

    // Use the newly created sample set
    pf->current_set = (pf->current_set + 1) % 2;

    return;
}

// Resample the nested distribution
// SINA: NOT USED!!!
void pf_update_nested_resample(pf_t *pf, double landmark_r, double landmark_phi, pf_vector_t upper_particle_pose
                               , pf_vector_t leader_pose, pf_vector_t leader_vel)
{
    int i;
    double total;
    pf_sample_set_t *set_a, *set_b;
    pf_sample_t *sample_a, *sample_b;

    pf_t *pf_nested_set_a,*pf_nested_set_b;
    pf_t *pf_sample_a, *pf_sample_b;

    double r,c,U;
    int m;
    double count_inv;

    double w_diff;

    set_a = pf->sets + pf->current_set;
    set_b = pf->sets + (pf->current_set + 1) % 2;

    /*
      Removing any more levels of nesting to keep things simple for now
    if(pf->nesting_lvl > 0){
        pf_nested_set_a = pf_get_this_nested_set(pf, pf->current_set);
        pf_nested_set_b = pf_get_this_nested_set(pf, (pf->current_set +1) % 2);
    }
    */

    // Create the kd tree for adaptive sampling
    pf_kdtree_clear(set_b->kdtree);

    // Draw samples from set a to create set b.
    total = 0;
    set_b->sample_count = 0;


    w_diff = 1.0 - pf->w_fast / pf->w_slow;
    if(w_diff < 0.0)
        w_diff = 0.0;
    //printf("w_diff: %9.6f\n", w_diff);

    int M = pf_resample_limit(pf, set_a->kdtree->leaf_count);
    //  printf("M: %i\n", M);
    // Low-variance resampler, taken from Probabilistic Robotics, p110
    count_inv = 1.0/M;
    r = drand48() * count_inv;
    c = set_a->samples[0].weight;
    i = 0;
    m = 0;

    while(set_b->sample_count < M)
    {

        /*
          Removing any more levels of nesting to keep things simple for now
        if(pf->nesting_lvl > 0){
            pf_sample_b = pf_nested_set_b + set_b->sample_count;
        }
        */

        sample_b = set_b->samples + set_b->sample_count++;

        //    if(drand48() < w_diff){

#if NESTED_DUAL
        if( (pf->isNested != 0)
                && (landmark_r > 0)
                && drand48() < 0.05){

            //KPM ...trying dual sampling instead of random sampling
            sample_b->pose = nested_dual_fn(pf->random_pose_data, landmark_r, landmark_phi, upper_particle_pose, leader_pose
                                            , leader_vel);

        }else{

            if(drand48() < w_diff){ // Recovery for normal particles


                /** Turning off DUAL sampling for non-nested particles for the moment. Re-enable this when it is ready
                if(DUAL_MCL == 1){ //KPM ...trying dual sampling instead of random sampling
                    sample_b->pose = (pf->dual_pose_fn)(pf->random_pose_data, landmark_r, landmark_phi, landmark_x, landmark_y);
                }

                */

                //else{ //Added by KPM to take random samples only when DUAL is turned off. Originally this condition was absent

                sample_b->pose = (pf->random_pose_fn)(pf->random_pose_data);

                //}

            }

            else
            {

                // Low-variance resampler, taken from Probabilistic Robotics, p110
                U = r + m * count_inv;
                while(U>c)
                {
                    i++;
                    // Handle wrap-around by resetting counters and picking a new random
                    // number
                    if(i >= set_a->sample_count)
                    {
                        r = drand48() * count_inv;
                        c = set_a->samples[0].weight;
                        i = 0;
                        m = 0;
                        U = r + m * count_inv;
                        continue;
                    }
                    c += set_a->samples[i].weight;
                }
                m++;

                assert(i<set_a->sample_count);

                sample_a = set_a->samples + i;

                /*
                  Removing any more levels of nesting to keep things simple for now
                if(pf->nesting_lvl > 0){
                    pf_sample_a = pf_nested_set_a + i;
                }
                */

                assert(sample_a->weight > 0);

                // Add sample to list
                sample_b->pose = sample_a->pose;

                /*
                  Removing any more levels of nesting to keep things simple for now
                if(pf->nesting_lvl > 0){

                    pf_copy(pf_sample_a, pf_sample_b);

                    pf_update_nested_resample(pf_sample_b, landmark_r, landmark_phi, sample_b->pose);
                }
                */

            }
        }

#else
        if(drand48() < w_diff){ // Recovery for normal particles


            /** Turning off DUAL sampling for non-nested particles for the moment. Re-enable this when it is ready
                        if(DUAL_MCL == 1){ //KPM ...trying dual sampling instead of random sampling
                            sample_b->pose = (pf->dual_pose_fn)(pf->random_pose_data, landmark_r, landmark_phi, landmark_x, landmark_y);
                        }

                        */

            //else{ //Added by KPM to take random samples only when DUAL is turned off. Originally this condition was absent

            sample_b->pose = (pf->random_pose_fn)(pf->random_pose_data);

            //}

        }

        else
        {

            // Low-variance resampler, taken from Probabilistic Robotics, p110
            U = r + m * count_inv;
            while(U>c)
            {
                i++;
                // Handle wrap-around by resetting counters and picking a new random
                // number
                if(i >= set_a->sample_count)
                {
                    r = drand48() * count_inv;
                    c = set_a->samples[0].weight;
                    i = 0;
                    m = 0;
                    U = r + m * count_inv;
                    continue;
                }
                c += set_a->samples[i].weight;
            }
            m++;

            assert(i<set_a->sample_count);

            sample_a = set_a->samples + i;

            /*
                          Removing any more levels of nesting to keep things simple for now
                        if(pf->nesting_lvl > 0){
                            pf_sample_a = pf_nested_set_a + i;
                        }
                        */

            assert(sample_a->weight > 0);

            // Add sample to list
            sample_b->pose = sample_a->pose;

            /*
                          Removing any more levels of nesting to keep things simple for now
                        if(pf->nesting_lvl > 0){

                            pf_copy(pf_sample_a, pf_sample_b);

                            pf_update_nested_resample(pf_sample_b, landmark_r, landmark_phi, sample_b->pose);
                        }
                        */

        }

#endif


        sample_b->weight = 1.0;
        total += sample_b->weight;

        // Add sample to histogram
        pf_kdtree_insert(set_b->kdtree, sample_b->pose, sample_b->weight);

    }


    /*
      Removing any more levels of nesting to keep things simple for now

    //Free all memory related to pf_nested_set_a
    if(pf->nesting_lvl>0){
        int counter=0;
        for(counter = 0; counter< set_a->sample_count; counter++){
            pf_sample_a = pf_nested_set_a + counter;

            free(pf_sample_a->sets[0].samples);
            free(pf_sample_a->sets[1].samples);
            //            free(pf_nested_set_a);
            //            free( pf_get_this_nested_set(pf, pf->current_set));
        }
    }
    */


    // Reset averages, to avoid spiraling off into complete randomness.
    if(w_diff > 0.0)
        pf->w_slow = pf->w_fast = 0.0;

    //fprintf(stderr, "\n\n");

    // Normalize weights
    for (i = 0; i < set_b->sample_count; i++)
    {
        sample_b = set_b->samples + i;
        sample_b->weight /= total;
    }

    // Re-compute cluster statistics
    pf_cluster_stats(set_b);

    // Use the newly created sample set
    pf->current_set = (pf->current_set + 1) % 2;

    return;
}





/** Adaptive Resamplers **/

// Resample the nested distribution
void pf_update_nested_adaptive_resample(pf_t *pf, double landmark_r, double landmark_phi, pf_vector_t upper_particle_pose,
                                        pf_vector_t leader_pose, pf_vector_t leader_vel)
{
    int i;
    double total;
    pf_sample_set_t *set_a, *set_b;
    pf_sample_t *sample_a, *sample_b;

    pf_t *pf_nested_set_a,*pf_nested_set_b;
    pf_t *pf_sample_a, *pf_sample_b;

    double r,c,U;
    int m;
    double count_inv;

    double w_diff;

    set_a = pf->sets + pf->current_set;
    set_b = pf->sets + (pf->current_set + 1) % 2;

    // ********
    // Re-Initializing set_b related stuff since the memory allocations are changing due to a new max_sample count

    // Free all pf related memory from any earlier iteration (set_b might have been used before)
    if(set_b->clusters != NULL){
        free(set_b->clusters);
        set_b->clusters = NULL;
    }
    if(set_b->kdtree != NULL){
        pf_kdtree_free(set_b->kdtree);
        set_b->kdtree = NULL;
    }
    if(set_b->samples != NULL){
        free(set_b->samples);
        set_b->samples = NULL;
    }

    // Allocate new kdtree for the nested particle set: "set_b" (the next set, not current set)
    // HACK: is 3 times max_samples enough?
    set_b->kdtree = pf_kdtree_alloc(3 * pf->max_samples);

    // new clusters
    set_b->cluster_count = 0;
    set_b->cluster_max_count = pf->max_samples;
    set_b->clusters = calloc(set_b->cluster_max_count, sizeof(pf_cluster_t));

    set_b->mean = set_a->mean;
    set_b->cov = set_a->cov;

    // new samples (with new "max_samples" count...could be either higher or lower than before, doesn't matter)
    set_b->samples = calloc(pf->max_samples, sizeof(pf_sample_t));


    // Create the kd tree for adaptive sampling
    pf_kdtree_clear(set_b->kdtree);


    // Draw samples from set a to create set b.
    total = 0;
    set_b->sample_count = 0;


    w_diff = 1.0 - pf->w_fast / pf->w_slow;
    if(w_diff < 0.0)
        w_diff = 0.0;
    //printf("w_diff: %9.6f\n", w_diff);

    int M = pf_resample_limit(pf, set_a->kdtree->leaf_count);
    //  printf("M: %i\n", M);


    // Low-variance resampler, taken from Probabilistic Robotics, p110
    count_inv = 1.0 / M;
    r = drand48() * count_inv;
    c = set_a->samples[0].weight;
    i = 0;
    m = 0;

    i = 0;
    while(set_b->sample_count < M)
    {
        sample_b = set_b->samples + set_b->sample_count++;

        if( (pf->isNested != 0) && (landmark_r > 0) && drand48() < 0.05 ){
            //KPM ...trying dual sampling instead of random sampling
            sample_b->pose = nested_dual_fn(pf->random_pose_data, landmark_r, landmark_phi, upper_particle_pose, leader_pose
                                            , leader_vel);
        } else {
            if(drand48() < w_diff){ // Recovery for normal particles
                sample_b->pose = (pf->random_pose_fn)(pf->random_pose_data);
            } else {

                // Low-variance resampler, taken from Probabilistic Robotics, p110

                U = r + m * count_inv;
                while(U > c)
                {
                    i++;
                    // Handle wrap-around by resetting counters and picking a new random
                    // number
                    if(i >= set_a->sample_count)
                    {
                        r = drand48() * count_inv;
                        c = set_a->samples[0].weight;
                        i = 0;
                        m = 0;
                        U = r + m * count_inv;
                        continue;
                    }
                    c += set_a->samples[i].weight;
                }
                m++;

                assert(i < set_a->sample_count);

                sample_a = set_a->samples + i;

                assert(sample_a->weight > 0);

                // Add sample to list
                sample_b->pose = sample_a->pose;
            }
        }

        sample_b->weight = 1.0;
        total += sample_b->weight;

        // Add sample to histogram
        pf_kdtree_insert(set_b->kdtree, sample_b->pose, sample_b->weight);

    }

    // Reset averages, to avoid spiraling off into complete randomness.
    if(w_diff > 0.0)
        pf->w_slow = pf->w_fast = 0.0;

    // Normalize weights
    for (i = 0; i < set_b->sample_count; i++)
    {
        sample_b = set_b->samples + i;
        sample_b->weight /= total;
    }

    // Re-compute cluster statistics
    pf_cluster_stats(set_b);

    // Use the newly created sample set
    pf->current_set = (pf->current_set + 1) % 2;

    return;
}

// Compute the required number of samples, given that there are k bins
// with samples in them.  This is taken directly from Fox et al.
int pf_resample_limit(pf_t *pf, int k)
{
    double a, b, c, x;
    int n;

    int sample_count = pf->sets[pf->current_set].sample_count;

    if (k <= 1){
        if(k==1 && sample_count>0 && sample_count<pf->max_samples && pf->isNested==0){
            printf("\n\n\t ****** k: %d ****** \n",k );
            return sample_count;
        }
        else{
            return pf->max_samples;
        }
    }

    a = 1;
    b = 2 / (9 * ((double) k - 1));
    c = sqrt(2 / (9 * ((double) k - 1))) * pf->pop_z;
    x = a - b + c;

    n = (int) ceil((k - 1) / (2 * pf->pop_err) * x * x * x);

    if (n < pf->min_samples)
        return pf->min_samples;
    if (n > pf->max_samples)
        return pf->max_samples;

    return n;
}


// Re-compute the cluster statistics for a sample set
void pf_cluster_stats(pf_sample_set_t *set)
{
    int i, j, k, cidx;
    pf_sample_t *sample;
    pf_cluster_t *cluster;

    // Workspace
    double m[4], c[2][2];
    size_t count;
    double weight;

    // Cluster the samples
    pf_kdtree_cluster(set->kdtree);

    // Initialize cluster stats
    set->cluster_count = 0;

    for (i = 0; i < set->cluster_max_count; i++)
    {
        cluster = set->clusters + i;
        cluster->count = 0;
        cluster->weight = 0;
        cluster->mean = pf_vector_zero();
        cluster->cov = pf_matrix_zero();

        for (j = 0; j < 4; j++)
            cluster->m[j] = 0.0;
        for (j = 0; j < 2; j++)
            for (k = 0; k < 2; k++)
                cluster->c[j][k] = 0.0;
    }

    // Initialize overall filter stats
    count = 0;
    weight = 0.0;
    set->mean = pf_vector_zero();
    set->cov = pf_matrix_zero();
    for (j = 0; j < 4; j++)
        m[j] = 0.0;
    for (j = 0; j < 2; j++)
        for (k = 0; k < 2; k++)
            c[j][k] = 0.0;

    // Compute cluster stats
    for (i = 0; i < set->sample_count; i++)
    {
        sample = set->samples + i;

        //printf("%d %f %f %f\n", i, sample->pose.v[0], sample->pose.v[1], sample->pose.v[2]);

        // Get the cluster label for this sample
        cidx = pf_kdtree_get_cluster(set->kdtree, sample->pose);
        assert(cidx >= 0);
        if (cidx >= set->cluster_max_count)
            continue;
        if (cidx + 1 > set->cluster_count)
            set->cluster_count = cidx + 1;

        cluster = set->clusters + cidx;

        cluster->count += 1;
        cluster->weight += sample->weight;

        count += 1;
        weight += sample->weight;

        // Compute mean
        cluster->m[0] += sample->weight * sample->pose.v[0];
        cluster->m[1] += sample->weight * sample->pose.v[1];
        cluster->m[2] += sample->weight * cos(sample->pose.v[2]);
        cluster->m[3] += sample->weight * sin(sample->pose.v[2]);

        m[0] += sample->weight * sample->pose.v[0];
        m[1] += sample->weight * sample->pose.v[1];
        m[2] += sample->weight * cos(sample->pose.v[2]);
        m[3] += sample->weight * sin(sample->pose.v[2]);

        // Compute covariance in linear components
        for (j = 0; j < 2; j++)
            for (k = 0; k < 2; k++)
            {
                cluster->c[j][k] += sample->weight * sample->pose.v[j] * sample->pose.v[k];
                c[j][k] += sample->weight * sample->pose.v[j] * sample->pose.v[k];
            }
    }

    // Normalize
    for (i = 0; i < set->cluster_count; i++)
    {
        cluster = set->clusters + i;

        cluster->mean.v[0] = cluster->m[0] / cluster->weight;
        cluster->mean.v[1] = cluster->m[1] / cluster->weight;
        cluster->mean.v[2] = atan2(cluster->m[3], cluster->m[2]);

        cluster->cov = pf_matrix_zero();

        // Covariance in linear components
        for (j = 0; j < 2; j++)
            for (k = 0; k < 2; k++)
                cluster->cov.m[j][k] = cluster->c[j][k] / cluster->weight -
                        cluster->mean.v[j] * cluster->mean.v[k];

        // Covariance in angular components; I think this is the correct
        // formula for circular statistics.
        cluster->cov.m[2][2] = -2 * log(sqrt(cluster->m[2] * cluster->m[2] +
                cluster->m[3] * cluster->m[3]));

        //printf("cluster %d %d %f (%f %f %f)\n", i, cluster->count, cluster->weight,
        //cluster->mean.v[0], cluster->mean.v[1], cluster->mean.v[2]);
        //pf_matrix_fprintf(cluster->cov, stdout, "%e");
    }

    // Compute overall filter stats
    set->mean.v[0] = m[0] / weight;
    set->mean.v[1] = m[1] / weight;
    set->mean.v[2] = atan2(m[3], m[2]);

    // Covariance in linear components
    for (j = 0; j < 2; j++)
        for (k = 0; k < 2; k++)
            set->cov.m[j][k] = c[j][k] / weight - set->mean.v[j] * set->mean.v[k];

    // Covariance in angular components; I think this is the correct
    // formula for circular statistics.
    set->cov.m[2][2] = -2 * log(sqrt(m[2] * m[2] + m[3] * m[3]));

    return;
}


// Compute the CEP statistics (mean and variance).
void pf_get_cep_stats(pf_t *pf, pf_vector_t *mean, double *var)
{
    int i;
    double mn, mx, my, mrr;
    pf_sample_set_t *set;
    pf_sample_t *sample;

    set = pf->sets + pf->current_set;

    mn = 0.0;
    mx = 0.0;
    my = 0.0;
    mrr = 0.0;

    for (i = 0; i < set->sample_count; i++)
    {
        sample = set->samples + i;

        mn += sample->weight;
        mx += sample->weight * sample->pose.v[0];
        my += sample->weight * sample->pose.v[1];
        mrr += sample->weight * sample->pose.v[0] * sample->pose.v[0];
        mrr += sample->weight * sample->pose.v[1] * sample->pose.v[1];
    }

    mean->v[0] = mx / mn;
    mean->v[1] = my / mn;
    mean->v[2] = 0.0;

    *var = mrr / mn - (mx * mx / (mn * mn) + my * my / (mn * mn));

    return;
}


// Get the statistics for a particular cluster.
int pf_get_cluster_stats(pf_t *pf, int clabel, double *weight,
                         pf_vector_t *mean, pf_matrix_t *cov)
{
    pf_sample_set_t *set;
    pf_cluster_t *cluster;

    set = pf->sets + pf->current_set;

    if (clabel >= set->cluster_count)
        return 0;
    cluster = set->clusters + clabel;

    *weight = cluster->weight;
    *mean = cluster->mean;
    *cov = cluster->cov;

    return 1;
}

// resampling from the Dual for nested particles
pf_vector_t nested_dual_fn(void* arg, double landmark_r, double landmark_phi, pf_vector_t upper_particle_pose,
                           pf_vector_t leader_pose_estimation, pf_vector_t leader_vel){
    int x1, y1;
    double gamma = 0.0;
    pf_vector_t hit, return_pose;

    double noise_in_laser_ranges = 0.01;
    double size_of_turtlebot = 0.03;

    double obs_range = landmark_r + pf_ran_gaussian(noise_in_laser_ranges);
    double obs_bearing = landmark_phi;
    map_t* map = (map_t*) arg;

    // Compute the endpoint of the beam
    hit.v[0] = upper_particle_pose.v[0] + obs_range * cos(upper_particle_pose.v[2] + obs_bearing);
    hit.v[1] = upper_particle_pose.v[1] + obs_range * sin(upper_particle_pose.v[2] + obs_bearing);

    double angle_correction = asin(leader_vel.v[1] / sqrt(pow(leader_vel.v[1], 2) + pow(leader_vel.v[0], 2)));

    do{

        // SINA: The dual samples are aligned with the follower's orientation
        //       with a PI/6 deviation

        double dice = drand48() * 100;
        double offset = (drand48() * M_PI / 6) - (M_PI / 12);  // 30 Degree of error applied to the estimated orientation
        if (dice < 20){
            gamma = (upper_particle_pose.v[2]) + offset;
        }else if (dice < 40){
            gamma = (leader_pose_estimation.v[2]) + offset;
        }else{
            gamma = (angle_correction) + offset;
        }

        return_pose.v[0] = hit.v[0] + pf_ran_gaussian(size_of_turtlebot / 2);
        return_pose.v[1] = hit.v[1] + pf_ran_gaussian(size_of_turtlebot / 2);

        return_pose.v[2] = gamma;

        x1 = MAP_GXWX(map, return_pose.v[0]);
        y1 = MAP_GYWY(map, return_pose.v[1]);

    }while(!MAP_VALID(map,x1,y1));

    return return_pose;
}

static void pf_copy(pf_t *pf_source, pf_t *pf_dest)
{
    //    pf_dest = calloc(1, sizeof(pf_t));

    pf_sample_t *new_samples, *old_samples;
    pf_sample_t *new_sample, *old_sample;

    pf_dest->min_samples = pf_source->min_samples;
    pf_dest->max_samples = pf_source->max_samples;

    pf_dest->pop_err = pf_source->pop_err;
    pf_dest->pop_z = pf_source->pop_z;

    pf_dest->current_set = pf_source->current_set;

    int i;

    for(i = 0; i < 2 ; i++){

        if(pf_source->sets[i].samples != NULL){


            pf_dest->sets[i].sample_count = pf_source->sets[i].sample_count;

            // This might be causing several different nested particles to point to same pool of samples
            //pf_dest->sets[i].samples = pf_source->sets[i].samples;

            int j=0;

            // It's possible that max_samples happens to be less than sample count for
            // an old set in the pf due to adaptive nature of Adaptive-NPF,
            // hence allocating space for sample_count instead of max_samples
            //pf_dest->sets[i].samples = calloc(pf_source->max_samples, sizeof(pf_sample_t));

            pf_dest->sets[i].samples = calloc(pf_source->sets[i].sample_count, sizeof(pf_sample_t));

            new_samples = pf_dest->sets[i].samples;
            old_samples = pf_source->sets[i].samples;

            // Not inserting nodes in the kdtree or even computing cluster statistics.
            // Just allocating memory, since the insertion and cluster stats will happen
            // after resampling anyways.
            pf_dest->sets[i].kdtree =  pf_kdtree_alloc(3 * pf_source->sets[i].sample_count);

            pf_dest->sets[i].cluster_count = pf_source->sets[i].cluster_count;
            pf_dest->sets[i].cluster_max_count = pf_source->sets[i].cluster_max_count;
            pf_dest->sets[i].clusters = calloc(pf_dest->sets[i].cluster_max_count, sizeof(pf_cluster_t));


            pf_dest->sets[i].mean = pf_source->sets[i].mean;
            pf_dest->sets[i].cov = pf_source->sets[i].cov;

            for(j=0; j<pf_dest->sets[i].sample_count ; j++ ){
                new_sample = new_samples + j;
                old_sample = old_samples + j;

                //                ROS_INFO("i: %d \t j: %d", i, j);

                //                pf_vector_t temp_pose = calloc(1, sizeof(pf_vector_t));

                //                pf_vector_t temp_pose;
                //                temp_pose = old_samples->pose;

                new_sample->pose = old_sample->pose;

                //                new_samples->pose.v[0] = old_samples->pose.v[0];
                //                new_samples->pose.v[1] = old_samples->pose.v[1];
                //                new_samples->pose.v[2] = old_samples->pose.v[2];

                new_sample->weight = old_sample->weight;
            }

        }
    }

    pf_dest->nesting_lvl = pf_source->nesting_lvl;
    pf_dest->min_nested_samples = pf_source->min_nested_samples;
    pf_dest->max_nested_samples = pf_source->max_nested_samples;


    pf_dest->fake_nested_odomPose = pf_source->fake_nested_odomPose;
    pf_dest->fake_nested_odomDelta = pf_source->fake_nested_odomDelta;

    pf_dest->w_slow = pf_source->w_slow;
    pf_dest->w_fast = pf_source->w_fast;

    pf_dest->alpha_slow = pf_source->alpha_slow;
    pf_dest->alpha_fast = pf_source->alpha_fast;

    pf_dest->random_pose_fn = pf_source->random_pose_fn;
    pf_dest->dual_pose_fn = pf_source->dual_pose_fn;

    pf_dest->random_pose_data = pf_source->random_pose_data;

    if(pf_source->nesting_lvl > 0){
        pf_dest->nested_pf_set_0 = pf_source->nested_pf_set_0;
        pf_dest->nested_pf_set_1 = pf_source->nested_pf_set_1;
    }

    return;
}


pf_t* pf_get_this_nested_set(pf_t *pf, int current_set){

    if(current_set == 0){
        /*
        if(pf->nested_pf_set_0 == NULL)
            pf->nested_pf_set_0 = calloc(pf->max_samples, sizeof(pf_t));
        */
        return pf->nested_pf_set_0;
    }
    else{
        /*
        if(pf->nested_pf_set_1 == NULL)
            pf->nested_pf_set_1 = calloc(pf->max_samples, sizeof(pf_t));
        */
        return pf->nested_pf_set_1;
    }
}

pf_t* pf_get_other_nested_set(pf_t *pf, int current_set){

    if(current_set == 1){
        /*
        if(pf->nested_pf_set_0 == NULL)
            pf->nested_pf_set_0 = calloc(pf->max_samples, sizeof(pf_t));
*/
        return pf->nested_pf_set_0;
    }
    else{
        /*
        if(pf->nested_pf_set_1 == NULL)
            pf->nested_pf_set_1 = calloc(pf->max_samples, sizeof(pf_t));
*/
        return pf->nested_pf_set_1;
    }
}

void pf_set_this_nested_set(pf_t *pf, int current_set, pf_t *allocated_pf){
    if(current_set == 0){
        pf->nested_pf_set_0 = allocated_pf;
    }
    else{
        pf->nested_pf_set_1 = allocated_pf;
    }
}


