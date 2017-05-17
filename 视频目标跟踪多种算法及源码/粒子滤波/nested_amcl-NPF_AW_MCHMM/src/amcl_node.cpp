/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
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

/* Author 1: Brian Gerkey
 * Author 2: Sina Solaimanpour
 */

#include <algorithm>
#include <vector>
#include <map>

// Signal handling
#include <signal.h>

//KPM
#include <cmath>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

#include "map.h"
#include "pf.h"
#include "amcl_odom.h"
#include "amcl_laser.h"

#include "ros/assert.h"

// roscpp
#include "ros/ros.h"
#include <ros/console.h>

// Messages that I need
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/GetMap.h"
#include "std_srvs/Empty.h"
#include "std_msgs/String.h"

// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/transform_datatypes.h"
#include "message_filters/subscriber.h"

// Dynamic_reconfigure
#include "dynamic_reconfigure/server.h"
#include "amcl/AMCLConfig.h"

// Added by KPM to enable color detection
#include "cmvision/Blobs.h"
#include "cmvision/Blob.h"

// SINA: Header files needed for the Monte Carlo HMM
#include "MCHMM.h"
#include "Sampler.h"

#include "gazebo_msgs/GetModelState.h"

#include <fstream>
#include <iostream>

#define COLLECT_DATA 1

#define NEW_UNIFORM_SAMPLING 1

//This is = (57/640)*(M_PI/180)
#define RADIANS_PER_PIXEL 0.001554434

//KPM: Since Kinect camera has a field of view of 57 degrees
// This is = (-28.5 * M_PI)/180
#define COLOR_MIN_ANGLE -0.497419
// This is = (28.5 * M_PI)/180
#define COLOR_MAX_ANGLE 0.497419

//#define MX = 3.7
//#define MY = 1.0


using namespace amcl;

// Pose hypothesis
typedef struct
{
    // Total weight (weights sum to 1)
    double weight;

    // Mean of pose esimate
    pf_vector_t pf_pose_mean;

    // Covariance of pose estimate
    pf_matrix_t pf_pose_cov;

} amcl_hyp_t;

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

// Function for comparing int numbers. This is needed for passing to the qsort function
int compare_int(const void * a, const void * b){
    return ( *(int*)a - *(int*)b );
}


class AmclNode
{
public:
    AmclNode();
    ~AmclNode();

    int process();

    // SINA: Collect samples for a separate HMM run
    std::string obs_file_path;
    std::string m_file_path;
    std::string v_file_path;
    std::string results_file_path;
    std::ofstream obs_out;
    std::ofstream m_out;
    std::ofstream v_out;
    std::ofstream results_out;

private:
    /* KPM */
    bool color_angles[641];
    int landmark_types;
    int type_object;
    double landmark_phi;
    double landmark_r, other_robot_distance;
    bool isLandmarkObserved;
    int nested_particle_count, total_nested_particle_count;
    double normal_MSE, nested_MSE, normal_RootMSE, nested_RootMSE;
    int normal_particles_within_1m, nested_particles_within_1m;
    double occlusion_proportion;

    /* SINA: this object will be used to train and reason from the hmm */
    MCHMM hmm;

    // SINA: These two variables will be used to find out when to learn the HMM
    bool learn_criteria;
    int collected_sample;

    // SINA: when landmark_r and landmark_phi are found, copy them here and reset them when collected
    double landmark_r_sample;
    double landmark_phi_sample;

    // SINA: Angle correction for the nested particles based on the new velocity estimation
    double angle_correction;

    /* SINA: this vector will hold all of the observations throughout the run
     *
     * We should add observations to this vector and keep updating the HMM whenever
     * we have new data available (or we can have intervals, still not decided)
     */
    vector<Observation> observations;
    vector<Sample>      pi_;
    vector<Sample>      m_;
    vector<Sample>      v_;

    ros::Time           sampling_time[3];
    ros::Time           hmm_use_time;
    pf_vector_t         our_previous_pose[3];
    pf_vector_t         nested_previous_pose[3];
    pf_vector_t         velocity_samples[2];
    pf_vector_t         accel_sample[2];
    bool                propagate_based_on_observation;

#if COLLECT_DATA
    /* Initializing the file into which we'll collect all our data */
    std::stringstream headers;
    std::string filename_abs;

    std::string file_path_from_home;
    std::string algo_name;
    std::string robot_start_config_id;
    std::string trajectory_id;
    long int start_timestamp;
    int run_number;

    /* *** */
#endif

    tf::TransformBroadcaster* tfb_;
    tf::TransformListener* tf_;

    bool sent_first_transform_;

    tf::Transform latest_tf_;
    tf::Transform latest_tf_true;
    bool latest_tf_valid_;

    // Pose-generating function used to uniformly distribute particles over
    // the map
    static pf_vector_t uniformPoseGenerator(void* arg);
#if NEW_UNIFORM_SAMPLING
    static std::vector<std::pair<int,int> > free_space_indices;
#endif

    // KPM: pose generating function for Dual of MCL
    static pf_vector_t dualMCL_PoseGenerator(void* arg, double r, double phi, double landmark_x,
                                             double landmark_y);

    /* SINA: Function to init the HMM object */
    void init_HMM();

    /* SINA: This function will be called whenever we have new data to update the HMM structure */
    void learn_HMM();

    /* SINA: This method is called to collect an observation based on the application */
    void collect_sample(geometry_msgs::PoseWithCovarianceStamped* our_pose,
                        geometry_msgs::PoseWithCovarianceStamped* leader_pose,
                        double landmark_r, double landmark_phi,
                        double nested_MSE);

    // Message callbacks
    bool globalLocalizationCallback(std_srvs::Empty::Request& req,
                                    std_srvs::Empty::Response& res);
    void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);
    void colorReceived(const cmvision::BlobsConstPtr& Blobs);
    void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void initialPoseReceivedOld(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void mapReceived(const nav_msgs::OccupancyGridConstPtr& msg);

    void handleMapMessage(const nav_msgs::OccupancyGrid& msg);
    void freeMapDependentMemory();
    map_t* convertMap( const nav_msgs::OccupancyGrid& map_msg, int threshold , bool use_exact_threshold);
    void applyInitialPose();

    double getYaw(tf::Pose& t);

    void log_data(geometry_msgs::PoseWithCovarianceStamped pose_bestEstimate);

    //parameter for what odom to use
    std::string odom_frame_id_;
    //parameter for what base to use
    std::string base_frame_id_;
    std::string global_frame_id_;

    // SINA: used to keep the nested odom frame_id
    //parameter for what odom to use
    std::string nested_odom_frame_id_;

    // Used to store the features like crosswalk and junction data
    std::string feature_filename;

    bool use_map_topic_;
    bool first_map_only_;

    ros::Duration gui_publish_period;
    ros::Time save_pose_last_time;
    ros::Duration save_pose_period;

    geometry_msgs::PoseWithCovarianceStamped last_published_pose;

    geometry_msgs::PoseWithCovarianceStamped nested_last_published_pose;

    map_t* map_;
    map_t* color_map;
    char* mapdata;
    int sx, sy;
    double resolution;

    message_filters::Subscriber<sensor_msgs::LaserScan>* laser_scan_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* laser_scan_filter_;
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>* initial_pose_sub_;
    tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>* initial_pose_filter_;
    std::vector< AMCLLaser* > lasers_;
    std::vector< bool > lasers_update_;
    std::map< std::string, int > frame_to_laser_;

    // Particle filter
    pf_t *pf_;
    double pf_err_, pf_z_;
    bool pf_init_;
    pf_vector_t pf_odom_pose_;
    double d_thresh_, a_thresh_;
    int resample_interval_;
    int resample_count_;
    double laser_min_range_;
    double laser_max_range_;

    AMCLOdom* odom_;
    AMCLLaser* laser_;

    ros::Duration cloud_pub_interval;
    ros::Time last_cloud_pub_time;

    void requestMap();

    // Helper to get odometric pose from transform system
    bool getOdomPose(tf::Stamped<tf::Pose>& pose,
                     double& x, double& y, double& yaw,
                     const ros::Time& t, const std::string& f);

    //time for tolerance on the published transform,
    //basically defines how long a map->odom transform is good for
    ros::Duration transform_tolerance_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher  pose_pub_;
    //    ros::Publisher true_pose_pub_;
    ros::Publisher particlecloud_pub_;

    ros::Publisher nested_pose_pub_;
    ros::Publisher nested_particlecloud_pub_; // For publishing cloud of nested particles

    ros::Publisher npf_data_pub_; // For publishing NPF data
    ros::Publisher data_fname_pub_;
    ros::Publisher data_headers_pub_;

    ros::ServiceServer global_loc_srv_;
    ros::Subscriber initial_pose_sub_old_;
    ros::Subscriber map_sub_;

    ros::Subscriber color_subscriber;

    amcl_hyp_t* initial_pose_hyp_;
    bool first_map_received_;
    bool first_reconfigure_call_;

    boost::recursive_mutex configuration_mutex_;
    dynamic_reconfigure::Server<amcl::AMCLConfig> *dsrv_;
    amcl::AMCLConfig default_config_;

    int max_beams_, min_particles_, max_particles_;
    int min_nested_particles_, max_nested_particles_;
    int nesting_lvl_; // indicates how many levels of nesting you want in the filter

    double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
    double alpha_slow_, alpha_fast_;
    double z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_;
    double laser_likelihood_max_dist_;
    odom_model_t odom_model_type_;
    double init_pose_[3];
    double init_cov_[3];

    //SINA: Initial guess for the leader pose
    double init_leader_pose_[3];
    double init_leader_cov_[3];

    laser_model_t laser_model_type_;

    //KPM: landmark location
    double landmark_loc_x;
    double landmark_loc_y;

    /* Initialize true pose related stuff*/

    ros::ServiceClient true_pose_client;
    gazebo_msgs::GetModelState true_pose_service;

    pf_vector_t true_pose_normal;
    pf_vector_t true_pose_nested;


    /* *** */
    void reconfigureCB(amcl::AMCLConfig &config, uint32_t level);
};

std::vector<std::pair<int,int> > AmclNode::free_space_indices;

#define USAGE "USAGE: nested_amcl"

boost::shared_ptr<AmclNode> amcl_node_ptr;

void sigintHandler(int sig)
{
    // Save latest pose as we're shutting down.
    ROS_FATAL("Exiting the process completely!");

    amcl_node_ptr.get()->obs_out.close();
    amcl_node_ptr.get()->m_out.close();
    amcl_node_ptr.get()->v_out.close();

    ros::shutdown();
    exit(0);
}

int
main(int argc, char** argv)
{
    // if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    //   		ros::console::notifyLoggerLevelsChanged();
    // }

    ros::init(argc, argv, "nested_amcl");
    ros::NodeHandle nh;

    // Override default sigint handler
    signal(SIGINT, sigintHandler);

    amcl_node_ptr.reset(new AmclNode());

    ros::spin();

    // Without this, our boost locks are not shut down nicely
    amcl_node_ptr.reset();

    // To quote Morgan, Hooray!
    return(0);
}

AmclNode::AmclNode() :
    sent_first_transform_(false),
    latest_tf_valid_(false),
    map_(NULL),
    pf_(NULL),
    resample_count_(0),
    odom_(NULL),
    laser_(NULL),
    private_nh_("~"),
    initial_pose_hyp_(NULL),
    first_map_received_(false),
    first_reconfigure_call_(true)
{
    boost::recursive_mutex::scoped_lock l(configuration_mutex_);

    // SINA: Use this to decide whether we should use the HMM or pure observation to estimate the motion
    propagate_based_on_observation = false;

    //KPM: innitializing some variables
    landmark_phi = 0;
    landmark_r = 0;

    //SINA: Initialize the landmark_r_sample and landmark_phi_sample to -1
    landmark_r_sample = -1;
    landmark_phi_sample = -1;

    //SINA: Initialize the learn criteria and collected_samples
    learn_criteria = false;
    collected_sample = 0;

    angle_correction = 0;

    other_robot_distance = 0.0;
    isLandmarkObserved = false;
    nested_particle_count = 0;
    total_nested_particle_count = 0;
    normal_MSE = 0.0;
    nested_MSE = 0.0;
    normal_RootMSE = 0.0;
    nested_RootMSE = 0.0;
    normal_particles_within_1m = 0;
    nested_particles_within_1m = 0;

    // Grab params off the param server
    private_nh_.param("use_map_topic", use_map_topic_, false);
    private_nh_.param("first_map_only", first_map_only_, false);

    double tmp;
    private_nh_.param("gui_publish_rate", tmp, 10.0);
    gui_publish_period = ros::Duration(1.0/tmp);
    private_nh_.param("save_pose_rate", tmp, 0.5);
    save_pose_period = ros::Duration(1.0/tmp);

    private_nh_.param("laser_min_range", laser_min_range_, -1.0);
    private_nh_.param("laser_max_range", laser_max_range_, 4.0);
    private_nh_.param("laser_max_beams", max_beams_, 640);
    private_nh_.param("min_particles", min_particles_, 1);
    private_nh_.param("max_particles", max_particles_, 2);

    private_nh_.param("min_nested_particles", min_nested_particles_, 1);
    private_nh_.param("max_nested_particles", max_nested_particles_, 1);
    private_nh_.param("nesting_lvl", nesting_lvl_, 1);

    private_nh_.param("kld_err", pf_err_, 0.05);
    private_nh_.param("kld_z", pf_z_, 0.99);
    private_nh_.param("odom_alpha1", alpha1_, 0.2);
    private_nh_.param("odom_alpha2", alpha2_, 0.2);
    private_nh_.param("odom_alpha3", alpha3_, 0.2);
    private_nh_.param("odom_alpha4", alpha4_, 0.2);
    private_nh_.param("odom_alpha5", alpha5_, 0.2);

    private_nh_.param("laser_z_hit", z_hit_, 0.5);
    private_nh_.param("laser_z_short", z_short_, 0.05);
    private_nh_.param("laser_z_max", z_max_, 0.05);
    private_nh_.param("laser_z_rand", z_rand_, 0.5);
    private_nh_.param("laser_sigma_hit", sigma_hit_, 0.2);
    private_nh_.param("laser_lambda_short", lambda_short_, 0.1);
    private_nh_.param("laser_likelihood_max_dist", laser_likelihood_max_dist_, 2.0);


    //KPM: landmark location params
    private_nh_.param("landmark_loc_x", landmark_loc_x, 0.0);
    private_nh_.param("landmark_loc_y", landmark_loc_y, 0.0);


    true_pose_client = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

#if COLLECT_DATA
    /* ****Data Collection**** */
    std::string home_path = std::string(getenv("HOME"));


    algo_name = "MCHMM";
    std::string file_name;

    private_nh_.param("robot_start_config_id", robot_start_config_id, std::string("DefaultConfig") );
    private_nh_.param("trajectory_id", trajectory_id, std::string("DefaultTrajectory"));
    private_nh_.param("file_path_from_home", file_path_from_home, std::string("/indigo_workspace/data_dump"));
    private_nh_.param("run_number", run_number, 0);


    /* --- Get current timestamp in milliseconds to append to filename --- */

    // Get timestamp
    struct timeval tp;
    gettimeofday(&tp, NULL);
    start_timestamp = tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds

    /* --- End of timestamp getting --- */

    // Putting all name related data into a stream so we can
    // convert it all together and put it into a string

    std::stringstream file_name_strstream;

    file_name_strstream << algo_name << "_"
                        << robot_start_config_id << "_"
                        << trajectory_id << "_"
                        << run_number << "_"
                        << max_particles_ << "_"
                        << max_nested_particles_ << "_"
                        << start_timestamp;

    file_name_strstream >> file_name;

    filename_abs = home_path + file_path_from_home + "/" + file_name + ".txt";


    headers << "algo_name" << "\t"
               //<< "robot_start_config_id" << "\t"
               //<< "trajectory_id" << "\t"
            << "run_number" << "\t"
            << "start_timestamp" << "\t"
               //<< "max_particles" << "\t"
               //<< "max_nested_particles" << "\t"
            << "elapsed_time" << "\t"
            << "current normal particle count" << "\t"

               //<< "True Pose" << "\t"
               //<< "Estimated Pose" << "\t"
               //<< "distance betwn true and estimate" << "\t"
               //<< "Particle MSE" << "\t"
               //<< "Particle RMS Error" << "\t"
               //<< "Normal particles within 1m of true pose" << "\t"

               //<< "Occlusion proportion (0.0-1.0)" << "\t"

               //<< "True Nested Pose" << "\t"
               //<< "Estimated Nested Pose" << "\t"
               //<< "Distance between nested true and nested estimate" << "\t"
            << "Nested Particle MSE" << "\t"
            << "Nested Particle RMS Error" << "\t"
            << "Nested particles within 1m of true nested pose" << "\t"

               //<< "avg weight"<< "\t"
               //<< "cov.m[0][0]" << "\t"
               //<< "cov.m[1][1]" << "\t"
               //<< "cov.m[2][2]" << "\t"
               //<< "covariance sum" << "\t"
            << "other robot observed" << "\t"
            << "other robot distance" << "\t"
            << "current nested_particle count" << "\t"
            << "total_nested_particle_count";


    /* **** End of Data collection related stuff **** */

    // SINA: Initialize the file paths for HMM data collection
    obs_file_path = home_path + "/Desktop/obs.txt";
    m_file_path = home_path + "/Desktop/m.txt";
    v_file_path = home_path + "/Desktop/v.txt";
    results_file_path = home_path + "/indigo_workspace/dump_files/" + file_name.c_str();

    obs_out.open(obs_file_path, ios::out);
    m_out.open(m_file_path, ios::out);
    v_out.open(v_file_path, ios::out);
    results_out.open(results_file_path, ios::out);

    // Write the headers to the collected data file
    {
        obs_out << "CrossWALK" << "\t" << "TurnPOINT" << "\t"
                << "Junction" << "\t" << "WallLeft" << "\t"
                << "WallRight" << std::endl;

        m_out << "OldVel_X" << "\t" << "OldVel_Y" << "\t"
              << "NewVel_X" << "\t" << "NewVel_Y" << std::endl;

        v_out << "CrossWALK" << "\t" << "TurnPOINT" << "\t"
              << "Junction" << "\t" << "WallLeft" << "\t"
              << "WallRight" << "\t" << "NewVel_X" << "\t"
              << "NewVel_Y" << std::endl;

        results_out << headers.str() << std::endl;

        obs_out.flush();
        m_out.flush();
        v_out.flush();
        results_out.flush();
    }

#endif

    /**** Initializing Laser Model ****/
    std::string tmp_model_type;
    private_nh_.param("laser_model_type", tmp_model_type, std::string("likelihood_field"));
    if(tmp_model_type == "beam")
        laser_model_type_ = LASER_MODEL_BEAM;
    else if(tmp_model_type == "likelihood_field")
        laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
    else
    {
        ROS_WARN("Unknown laser model type \"%s\"; defaulting to likelihood_field model",
                 tmp_model_type.c_str());
        laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
    }

    /**** Initializing Odom Model ****/
    private_nh_.param("odom_model_type", tmp_model_type, std::string("diff"));
    if(tmp_model_type == "diff")
        odom_model_type_ = ODOM_MODEL_DIFF;
    else if(tmp_model_type == "omni")
        odom_model_type_ = ODOM_MODEL_OMNI;
    else
    {
        ROS_WARN("Unknown odom model type \"%s\"; defaulting to diff model",
                 tmp_model_type.c_str());
        odom_model_type_ = ODOM_MODEL_DIFF;
    }

    private_nh_.param("feature_filename", feature_filename, std::string(""));

    private_nh_.param("update_min_d", d_thresh_, 0.25);
    private_nh_.param("update_min_a", a_thresh_, M_PI/6.0);
    private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
    private_nh_.param("nested_odom_frame_id", nested_odom_frame_id_, std::string("nested_odom"));
    private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
    private_nh_.param("global_frame_id", global_frame_id_, std::string("/map"));
    private_nh_.param("resample_interval", resample_interval_, 1);
    double tmp_tol;
    private_nh_.param("transform_tolerance", tmp_tol, 1.0);
    private_nh_.param("recovery_alpha_slow", alpha_slow_, 0.001);
    private_nh_.param("recovery_alpha_fast", alpha_fast_, 0.1);

    transform_tolerance_.fromSec(tmp_tol);

    private_nh_.param("initial_pose_x", init_pose_[0], 0.0);
    private_nh_.param("initial_pose_y", init_pose_[1], 0.0);
    private_nh_.param("initial_pose_a", init_pose_[2], 0.0);
    private_nh_.param("initial_cov_xx", init_cov_[0], 0.5 * 0.5);
    private_nh_.param("initial_cov_yy", init_cov_[1], 0.5 * 0.5);
    private_nh_.param("initial_cov_aa", init_cov_[2], (M_PI/12.0) * (M_PI/12.0));

    private_nh_.param("initial_leader_pose_x", init_leader_pose_[0], 0.0);
    private_nh_.param("initial_leader_pose_y", init_leader_pose_[1], 0.0);
    private_nh_.param("initial_leader_pose_a", init_leader_pose_[2], 0.0);
    private_nh_.param("initial_leader_cov_xx", init_leader_cov_[0], 0.5 * 0.5);
    private_nh_.param("initial_leader_cov_yy", init_leader_cov_[1], 0.5 * 0.5);
    private_nh_.param("initial_leader_cov_aa", init_leader_cov_[2], (M_PI/12.0) * (M_PI/12.0));

    cloud_pub_interval.fromSec(1.0);
    tfb_ = new tf::TransformBroadcaster();
    tf_ = new tf::TransformListener();

    /**** Publishers and Subscribers ****/
    pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 2);
    //true_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("true_pose", 2);
    particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 2);

    nested_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("nested_amcl_pose", 2);
    nested_particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("nested_particlecloud", 2);

    npf_data_pub_ = nh_.advertise<std_msgs::String>("nested_amcl_data", 2);
#if COLLECT_DATA
    data_fname_pub_ = nh_.advertise<std_msgs::String>("data_file_name", 2);
    data_headers_pub_ = nh_.advertise<std_msgs::String>("data_headers", 2);
#endif

    global_loc_srv_ = nh_.advertiseService("global_localization",
                                           &AmclNode::globalLocalizationCallback,
                                           this);
    laser_scan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, "scan", 100);
    laser_scan_filter_ =
            new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_,
                                                          *tf_,
                                                          odom_frame_id_,
                                                          100);
    laser_scan_filter_->registerCallback(boost::bind(&AmclNode::laserReceived,
                                                     this, _1));

    initial_pose_sub_ = new message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>(nh_, "initialpose", 2);
    initial_pose_filter_ = new tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>(*initial_pose_sub_, *tf_, global_frame_id_, 2);
    initial_pose_filter_->registerCallback(boost::bind(&AmclNode::initialPoseReceived, this, _1));

    initial_pose_sub_old_ = nh_.subscribe("initialpose", 2, &AmclNode::initialPoseReceivedOld, this);

    color_subscriber = nh_.subscribe("blobs", 2, &AmclNode::colorReceived, this); //KPM


    ///
    if(use_map_topic_) {
        map_sub_ = nh_.subscribe("map", 1, &AmclNode::mapReceived, this);
        ROS_INFO("Subscribed to map topic.");
    } else {
        requestMap();
    }

    dsrv_ = new dynamic_reconfigure::Server<amcl::AMCLConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<amcl::AMCLConfig>::CallbackType cb = boost::bind(&AmclNode::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    //Initializing bool color_angles[641]

    for(int i=0; i<= 641; i++){
        color_angles[i] = false;
    }

    /* SINA: Call the init_HMM function to init the HMM model once */
    init_HMM();
}

void AmclNode::reconfigureCB(AMCLConfig &config, uint32_t level)
{
    ROS_INFO("Calling reconfigureCB but HMM will not change.");

    boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);

    //we don't want to do anything on the first call
    //which corresponds to startup
    if(first_reconfigure_call_)
    {
        first_reconfigure_call_ = false;
        default_config_ = config;
        return;
    }

    if(config.restore_defaults) {
        config = default_config_;
        //avoid looping
        config.restore_defaults = false;
    }

    d_thresh_ = config.update_min_d;
    a_thresh_ = config.update_min_a;

    resample_interval_ = config.resample_interval;

    laser_min_range_ = config.laser_min_range;
    laser_max_range_ = config.laser_max_range;

    gui_publish_period = ros::Duration(1.0/config.gui_publish_rate);
    save_pose_period = ros::Duration(1.0/config.save_pose_rate);

    transform_tolerance_.fromSec(config.transform_tolerance);

    //    max_beams_ = config.laser_max_beams;
    ROS_INFO("Max Beam changed to 640!");
    max_beams_ = 640;
    alpha1_ = config.odom_alpha1;
    alpha2_ = config.odom_alpha2;
    alpha3_ = config.odom_alpha3;
    alpha4_ = config.odom_alpha4;
    alpha5_ = config.odom_alpha5;

    z_hit_ = config.laser_z_hit;
    z_short_ = config.laser_z_short;
    z_max_ = config.laser_z_max;
    z_rand_ = config.laser_z_rand;
    sigma_hit_ = config.laser_sigma_hit;
    lambda_short_ = config.laser_lambda_short;
    laser_likelihood_max_dist_ = config.laser_likelihood_max_dist;

    if(config.laser_model_type == "beam")
        laser_model_type_ = LASER_MODEL_BEAM;
    else if(config.laser_model_type == "likelihood_field")
        laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;

    if(config.odom_model_type == "diff")
        odom_model_type_ = ODOM_MODEL_DIFF;
    else if(config.odom_model_type == "omni")
        odom_model_type_ = ODOM_MODEL_OMNI;

    if(config.min_particles > config.max_particles)
    {
        ROS_WARN("You've set min_particles to be less than max particles, this isn't allowed so they'll be set to be equal.");
        config.max_particles = config.min_particles;
    }

    min_particles_ = config.min_particles;
    max_particles_ = config.max_particles;
    alpha_slow_ = config.recovery_alpha_slow;
    alpha_fast_ = config.recovery_alpha_fast;

    pf_ = pf_alloc(min_particles_, max_particles_,
                   alpha_slow_, alpha_fast_,
                   (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                   (pf_dual_model_fn_t)AmclNode::dualMCL_PoseGenerator, //Added by KPM
                   (void *)map_,
                   //Added by KPM
                   nesting_lvl_, min_nested_particles_, max_nested_particles_);
    pf_err_ = config.kld_err;
    pf_z_ = config.kld_z;
    pf_->pop_err = pf_err_;
    pf_->pop_z = pf_z_;

    // Initialize the filter
    pf_vector_t pf_init_pose_mean = pf_vector_zero();
    pf_init_pose_mean.v[0] = last_published_pose.pose.pose.position.x;
    pf_init_pose_mean.v[1] = last_published_pose.pose.pose.position.y;
    pf_init_pose_mean.v[2] = tf::getYaw(last_published_pose.pose.pose.orientation);
    pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
    pf_init_pose_cov.m[0][0] = last_published_pose.pose.covariance[6*0+0];
    pf_init_pose_cov.m[1][1] = last_published_pose.pose.covariance[6*1+1];
    pf_init_pose_cov.m[2][2] = last_published_pose.pose.covariance[6*5+5];
    pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov, map_);
    pf_init_ = false;

    // Instantiate the sensor objects
    // Odometry
    delete odom_;

    // KPM: Replacing the original constructor with one that initializes the map
    //odom_ = new AMCLOdom();
    odom_ = new AMCLOdom(map_, &hmm);

    ROS_ASSERT(odom_);
    if(odom_model_type_ == ODOM_MODEL_OMNI)
        odom_->SetModelOmni(alpha1_, alpha2_, alpha3_, alpha4_, alpha5_);
    else
        odom_->SetModelDiff(alpha1_, alpha2_, alpha3_, alpha4_);
    // Laser
    delete laser_;
    laser_ = new AMCLLaser(max_beams_, map_, color_map);
    ROS_ASSERT(laser_);
    if(laser_model_type_ == LASER_MODEL_BEAM)
        laser_->SetModelBeam(z_hit_, z_short_, z_max_, z_rand_,
                             sigma_hit_, lambda_short_, 0.0);
    else
    {
        ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
        laser_->SetModelLikelihoodField(z_hit_, z_rand_, sigma_hit_,
                                        laser_likelihood_max_dist_);
        ROS_INFO("Done initializing likelihood field model.");
    }

    odom_frame_id_ = config.odom_frame_id;
    base_frame_id_ = config.base_frame_id;
    global_frame_id_ = config.global_frame_id;

    delete laser_scan_filter_;
    laser_scan_filter_ =
            new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_,
                                                          *tf_,
                                                          odom_frame_id_,
                                                          100);
    laser_scan_filter_->registerCallback(boost::bind(&AmclNode::laserReceived,
                                                     this, _1));

    delete initial_pose_filter_;
    initial_pose_filter_ = new tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>(*initial_pose_sub_, *tf_, global_frame_id_, 2);
    initial_pose_filter_->registerCallback(boost::bind(&AmclNode::initialPoseReceived, this, _1));
}

// TODO: still not decided how and when to actually call this function, but
//       it is meant to update the HMM strucuture and learn the distributions
void AmclNode::learn_HMM(){
    int max_iterations = 2;
    int N = 20;

    for (size_t i = 0; i < pi_.size(); i++){
        pi_[i].p = 1.0 / pi_.size();
    }

    for (size_t i = 0; i < m_.size(); i++){
        m_[i].p = 1.0 / m_.size();
    }

    for (size_t i = 0; i < v_.size(); i++){
        v_[i].p = 1.0 / v_.size();
    }

    hmm.set_distributions(&pi_, &m_, &v_, 0.5);

    hmm.learn_hmm(&observations, max_iterations, N);

    ROS_WARN("Finished Learning");
}

void AmclNode::collect_sample(geometry_msgs::PoseWithCovarianceStamped *our_pose,
                              geometry_msgs::PoseWithCovarianceStamped *leader_pose,
                              double landmark_r, double landmark_phi,
                              double nested_MSE){
    size_t current_pose_index = 2;

    // Get the leader pose from the nested particles
    pf_vector_t leader_pose_v;
    leader_pose_v.v[0] = leader_pose->pose.pose.position.x;
    leader_pose_v.v[1] = leader_pose->pose.pose.position.y;
    leader_pose_v.v[2] = tf::getYaw(leader_pose->pose.pose.orientation);

    // Get our own pose from the particles
    pf_vector_t our_pose_v;
    our_pose_v.v[0] = our_pose->pose.pose.position.x;
    our_pose_v.v[1] = our_pose->pose.pose.position.y;
    our_pose_v.v[2] = tf::getYaw(our_pose->pose.pose.orientation);

    // Compute the leader pose based on observation and our own pose (This is more reliable than nested particles)
    pf_vector_t leader_pose_v_from_obs;
    if (landmark_r_sample != -1 && landmark_phi_sample != -1){

        pf_vector_t add_vec;

        // Compute the vector that needs to be added to our own pose to get the leader's pose
        add_vec.v[0] = cos(our_pose_v.v[2] + landmark_phi) * landmark_r;
        add_vec.v[1] = sin(our_pose_v.v[2] + landmark_phi) * landmark_r;
        add_vec.v[2] = 0.0;

        // Here we have the leader pose from the addition process
        leader_pose_v_from_obs = pf_vector_add(our_pose_v, add_vec);
        // Set leader's pose to the one estimated using nested particles
        leader_pose_v_from_obs.v[2] = leader_pose_v.v[2];

        // Reset the landmark values
        landmark_r_sample = -1;
        landmark_phi_sample = -1;

        // We can use the mean of the two computed poses! But I preferred the obs pose
        //        leader_pose_v.v[0] = (leader_pose_v.v[0] + leader_pose_v_from_obs.v[0]) / 2.0;
        //        leader_pose_v.v[1] = (leader_pose_v.v[1] + leader_pose_v_from_obs.v[1]) / 2.0;

        // We use this computed pose if we have observation
        leader_pose_v.v[0] = (leader_pose_v_from_obs.v[0]);
        leader_pose_v.v[1] = (leader_pose_v_from_obs.v[1]);
    }

    // Shift all of the values in our history to left
    for (size_t i = 0; i < current_pose_index; i++){
        our_previous_pose[i] = our_previous_pose[i + 1];
        nested_previous_pose[i] = nested_previous_pose[i + 1];
        sampling_time[i] = sampling_time[i + 1];
    }

    // Set the current values to use in the rest of the calculations
    our_previous_pose[current_pose_index] = our_pose_v;
    nested_previous_pose[current_pose_index] = leader_pose_v;
    sampling_time[current_pose_index] = ros::Time::now();

    // Shift the velocities to open a space for the new velocity (We only have two velocities)
    for (size_t i = 0; i < current_pose_index - 1; i++){
        velocity_samples[i] = velocity_samples[i + 1];
    }

    // Compute the different in time since last sampling
    double delta_t = sampling_time[current_pose_index].toSec() - sampling_time[current_pose_index - 1].toSec();

    // Computer the new velocity based on the previous poses and the delta_t
    velocity_samples[current_pose_index - 1].v[0] = (nested_previous_pose[current_pose_index].v[0] -
            nested_previous_pose[current_pose_index - 1].v[0]) / delta_t;
    velocity_samples[current_pose_index - 1].v[1] = (nested_previous_pose[current_pose_index].v[1] -
            nested_previous_pose[current_pose_index - 1].v[1]) / delta_t;

    // Shift the acceleration values to the left
    for (size_t i = 0; i < current_pose_index - 1; i++){
        accel_sample[i] = accel_sample[i + 1];
    }

    // Compute the new acceleration based on the new velocities
    accel_sample[current_pose_index - 1].v[0] = (velocity_samples[current_pose_index - 1].v[0] -
            velocity_samples[current_pose_index - 2].v[0]) / delta_t;
    accel_sample[current_pose_index - 1].v[1] = (velocity_samples[current_pose_index - 1].v[1] -
            velocity_samples[current_pose_index - 2].v[1]) / delta_t;

    // Find the features based on the new leader pose
    int cross_walk_seen = map_see_crosswalk(this->map_, leader_pose_v, 0.2, 5);
    int junction_seen   = map_see_junction (this->map_, leader_pose_v, 0.2, 5);
    double* walls_dists = map_side_walls   (this->map_, leader_pose_v, 3     );

    if (!walls_dists){
        return;
    }

    // Collect Samples Here!!!
    if (/*observations.size() != 0 || */nested_MSE <= .3){

        //        ROS_INFO("Times: %f\t%f\t%f", sampling_time[0].toSec(), sampling_time[1].toSec(), sampling_time[2].toSec());

        //        ROS_INFO("Delta_T: %f", delta_t);

        //        ROS_INFO("Landmark R: %f\tLandmark Phi: %f", landmark_r, landmark_phi);

        //        ROS_INFO("Leader Pose: %f\t%f\t%f",
        //                 leader_pose_v.v[0],
        //                leader_pose_v.v[1],
        //                leader_pose_v.v[2]);

        //        if (print_)
        //            ROS_INFO("Leader Pose From Obs: %f\t%f\t%f",
        //                     leader_pose_v_from_obs.v[0],
        //                    leader_pose_v_from_obs.v[1],
        //                    leader_pose_v_from_obs.v[2]);

        //        ROS_INFO("Our Pose: %f\t%f\t%f",
        //                 our_pose_v.v[0],
        //                our_pose_v.v[1],
        //                our_pose_v.v[2]);

        //        ROS_INFO("Current Velocity: %f\t%f\t%f",
        //                 velocity_samples[current_pose_index - 1].v[0],
        //                velocity_samples[current_pose_index - 1].v[1],
        //                velocity_samples[current_pose_index - 1].v[2]);

        //        ROS_INFO("Current Acceleration: %f\t%f\t%f",
        //                 accel_sample[current_pose_index - 2].v[0],
        //                accel_sample[current_pose_index - 2].v[1],
        //                accel_sample[current_pose_index - 2].v[2]);

        //        ROS_INFO("Features:");
        //        ROS_INFO("\tCross Walk: %d", cross_walk_seen);
        //        ROS_INFO("\tJunction: %d  ", junction_seen);
        //        ROS_INFO("\tLeft Wall: %f ", walls_dists[0]);
        //        ROS_INFO("\tRight Wall: %f", walls_dists[1]);

        Observation obs;
        obs.values.push_back(cross_walk_seen);
        obs.values.push_back(junction_seen);
        obs.values.push_back(walls_dists[0]);
        obs.values.push_back(walls_dists[1]);

        observations.push_back(obs);

        obs_out << std::setprecision(3)
                << obs.values[0] << "\t" << obs.values[1] << "\t"
                << obs.values[2] << "\t" << obs.values[3] << "\t"
                << std::endl;
        obs_out.flush();

        ROS_DEBUG("Observation: %f, %f, %f, %f",
                  obs.values[0],
                obs.values[1],
                obs.values[2],
                obs.values[3]
                );

        Sample sample_m;
        sample_m.values.push_back(velocity_samples[current_pose_index - 2].v[0]);
        sample_m.values.push_back(velocity_samples[current_pose_index - 2].v[1]);
        sample_m.values.push_back(velocity_samples[current_pose_index - 1].v[0]);
        sample_m.values.push_back(velocity_samples[current_pose_index - 1].v[1]);

        m_.push_back(sample_m);

        m_out << std::setprecision(3)
              << sample_m.values[0] << "\t" << sample_m.values[1] << "\t"
              << sample_m.values[2] << "\t" << sample_m.values[3] << "\t"
              << std::endl;
        m_out.flush();

        ROS_DEBUG("M Sample: %f, %f, %f, %f",
                  sample_m.values[0],
                sample_m.values[1],
                sample_m.values[2],
                sample_m.values[3]
                );

        Sample sample_v;
        sample_v.values.push_back(cross_walk_seen);
        sample_v.values.push_back(junction_seen);
        sample_v.values.push_back(walls_dists[0]);
        sample_v.values.push_back(walls_dists[1]);
        sample_v.values.push_back(velocity_samples[current_pose_index - 1].v[0]);
        sample_v.values.push_back(velocity_samples[current_pose_index - 1].v[1]);

        v_.push_back(sample_v);

        v_out << std::setprecision(3)
              << sample_v.values[0] << "\t" << sample_v.values[1] << "\t"
              << sample_v.values[2] << "\t" << sample_v.values[3] << "\t"
              << sample_v.values[4] << "\t" << sample_v.values[5] << "\t"
              << std::endl;
        v_out.flush();

        ROS_DEBUG("V Sample: %f, %f, %f, %f, %f, %f",
                  sample_v.values[0],
                sample_v.values[1],
                sample_v.values[2],
                sample_v.values[3],
                sample_v.values[4],
                sample_v.values[5]
                );

        collected_sample++;
        ROS_WARN("Collected samples: %d", collected_sample);

        if (collected_sample >= 10){
            collected_sample = 0;
            learn_criteria = true;

            ROS_WARN("Let's learn something!");
        }
    }

}

// init the variable bounds in the HMM (the variables are continuous, but we assume they are bounded)
void AmclNode::init_HMM(){
    // Initializing the limits for the values that each variable can take
    // NOTE: Each vector should contain as many values as the dimension of the corresponding distribution
    vector<double> * pi_low_limits = new vector<double>();
    vector<double> * pi_high_limits = new vector<double>();
    vector<double> * m_low_limits = new vector<double>();
    vector<double> * m_high_limits = new vector<double>();
    vector<double> * v_low_limits = new vector<double>();
    vector<double> * v_high_limits = new vector<double>();

    // TODO: Add bounds to the vectors here!
    double vel_min = -0.4;

    double vel_max = 0.4;

    double crosswalk_min = 0;
    double crosswalk_max = 1;

    double turn_point_min = 0;
    double turn_point_max = 2;

    double junction_min = 0;
    double junction_max = 2;

    double wall_min = 0;
    double wall_max = 3.0;

    ////////// INIT PI BOUNDS //////////
    pi_low_limits->push_back(vel_min);
    pi_low_limits->push_back(vel_min);

    pi_high_limits->push_back(vel_max);
    pi_high_limits->push_back(vel_max);

    ////////// INIT M  BOUNDS //////////
    m_low_limits->push_back(vel_min);
    m_low_limits->push_back(vel_min);
    m_low_limits->push_back(vel_min);
    m_low_limits->push_back(vel_min);

    m_high_limits->push_back(vel_max);
    m_high_limits->push_back(vel_max);
    m_high_limits->push_back(vel_max);
    m_high_limits->push_back(vel_max);

    ////////// INIT V  BOUNDS //////////
    v_low_limits->push_back(crosswalk_min);
    v_low_limits->push_back(junction_min);
    v_low_limits->push_back(wall_min);
    v_low_limits->push_back(wall_min);
    v_low_limits->push_back(vel_min);
    v_low_limits->push_back(vel_min);

    v_high_limits->push_back(crosswalk_max);
    v_high_limits->push_back(junction_max);
    v_high_limits->push_back(wall_max);
    v_high_limits->push_back(wall_max);
    v_high_limits->push_back(vel_max);
    v_high_limits->push_back(vel_max);

    for (size_t i = 0; i < 20; i++){
        Sample pi_temp;
        pi_temp.values.push_back(0.0);
        pi_temp.values.push_back(0.0);

        pi_.push_back(pi_temp);
    }

    hmm.set_limits(pi_low_limits, pi_high_limits, m_low_limits, m_high_limits, v_low_limits, v_high_limits);

    // After setting these limits, all will be left is to learn the HMM structure! :)
}

void
AmclNode::requestMap()
{
    boost::recursive_mutex::scoped_lock ml(configuration_mutex_);

    // get map via RPC
    nav_msgs::GetMap::Request  req;
    nav_msgs::GetMap::Response resp;
    ROS_INFO("Requesting the map...");

    //changing "static_map" to "/static_map" for subscribing to the same topic when running multiple amcl nodes for multiple robots
    while(!ros::service::call("/static_map", req, resp))
    {
        ROS_WARN("Request for map failed; trying again...");
        ros::Duration d(0.5);
        d.sleep();
    }
    handleMapMessage( resp.map );
}

void
AmclNode::mapReceived(const nav_msgs::OccupancyGridConstPtr& msg)
{
    if( first_map_only_ && first_map_received_ ) {
        return;
    }

    handleMapMessage( *msg );

    first_map_received_ = true;
}

void
AmclNode::handleMapMessage(const nav_msgs::OccupancyGrid& msg)
{
    boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);

    ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
             msg.info.width,
             msg.info.height,
             msg.info.resolution);

    freeMapDependentMemory();

    map_ = convertMap(msg, 100, false);
    map_feature_load(map_, feature_filename.c_str());

    color_map = convertMap(msg, 50, true);

#if NEW_UNIFORM_SAMPLING
    ROS_INFO("NEW_UNIFORM_SAMPLING is %d",NEW_UNIFORM_SAMPLING);
    // Index of free space
    free_space_indices.resize(0);
    for(int i = 0; i < map_->size_x; i++)
        for(int j = 0; j < map_->size_y; j++)
            if(map_->cells[MAP_INDEX(map_,i,j)].occ_state == -1)
                free_space_indices.push_back(std::make_pair(i,j));
#endif
    // Create the particle filter
    pf_ = pf_alloc(min_particles_, max_particles_,
                   alpha_slow_, alpha_fast_,
                   (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                   (pf_dual_model_fn_t)AmclNode::dualMCL_PoseGenerator, //Added by KPM
                   (void *)map_,
                   //Added by KPM
                   nesting_lvl_, min_nested_particles_, max_nested_particles_);
    pf_->pop_err = pf_err_;
    pf_->pop_z = pf_z_;

    // Initialize the filter
    pf_vector_t pf_init_pose_mean = pf_vector_zero();
    pf_init_pose_mean.v[0] = init_pose_[0];
    pf_init_pose_mean.v[1] = init_pose_[1];
    pf_init_pose_mean.v[2] = init_pose_[2];
    pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
    pf_init_pose_cov.m[0][0] = init_cov_[0];
    pf_init_pose_cov.m[1][1] = init_cov_[1];
    pf_init_pose_cov.m[2][2] = init_cov_[2];
    pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov, map_);
    pf_init_ = false;

    // Instantiate the sensor objects
    // Odometry
    delete odom_;

    // KPM: Replacing the original constructor with one that initializes the map
    // odom_ = new AMCLOdom();
    odom_ = new AMCLOdom(map_, &hmm);
    ROS_ASSERT(odom_);
    if(odom_model_type_ == ODOM_MODEL_OMNI)
        odom_->SetModelOmni(alpha1_, alpha2_, alpha3_, alpha4_, alpha5_);
    else
        odom_->SetModelDiff(alpha1_, alpha2_, alpha3_, alpha4_);

    // Laser
    delete laser_;
    laser_ = new AMCLLaser(max_beams_, map_, color_map);
    ROS_ASSERT(laser_);
    if(laser_model_type_ == LASER_MODEL_BEAM)
        laser_->SetModelBeam(z_hit_, z_short_, z_max_, z_rand_,
                             sigma_hit_, lambda_short_, 0.0);
    else
    {
        ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
        laser_->SetModelLikelihoodField(z_hit_, z_rand_, sigma_hit_,
                                        laser_likelihood_max_dist_);
        ROS_INFO("Done initializing likelihood field model.");
    }

    // In case the initial pose message arrived before the first map,
    // try to apply the initial pose now that the map has arrived.
    applyInitialPose();
}

void
AmclNode::freeMapDependentMemory()
{
    if( map_ != NULL ) {
        map_free( map_ );
        map_ = NULL;
    }
    if( pf_ != NULL ) {
        pf_free( pf_ );
        pf_ = NULL;
    }
    delete odom_;
    odom_ = NULL;
    delete laser_;
    laser_ = NULL;
}

/**
 * Convert an OccupancyGrid map message into the internal
 * representation.  This allocates a map_t and returns it.
 */
map_t*
AmclNode::convertMap( const nav_msgs::OccupancyGrid& map_msg, int threshold , bool use_exact_threshold)
{
    map_t* map = map_alloc();
    ROS_ASSERT(map);

    map->size_x = map_msg.info.width;
    map->size_y = map_msg.info.height;
    map->scale = map_msg.info.resolution;
    map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
    map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;
    // Convert to player format
    map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
    ROS_ASSERT(map->cells);
    for(int i=0;i<map->size_x * map->size_y;i++)
    {
        if(use_exact_threshold){
            if(map_msg.data[i] == 0)        //free
                map->cells[i].occ_state = -1;

            else if(map_msg.data[i] == threshold)       //occupied
                map->cells[i].occ_state = +1;

            else                            //unknown
                map->cells[i].occ_state = 0;
        }
        else{
            if(map_msg.data[i] == 0)        //free
                map->cells[i].occ_state = -1;

            else if(map_msg.data[i] >= threshold)       //occupied
                map->cells[i].occ_state = +1;

            else                            //unknown
                map->cells[i].occ_state = 0;
        }
    }

    return map;
}

AmclNode::~AmclNode()
{
    delete dsrv_;
    freeMapDependentMemory();
    delete laser_scan_filter_;
    delete laser_scan_sub_;
    delete initial_pose_filter_;
    delete initial_pose_sub_;
    delete tfb_;
    delete tf_;

    // TODO: delete everything allocated in constructor
}


/*!
  * Determines robot's pose within the map when a particular laser scan was taken
  */
bool
AmclNode::getOdomPose(tf::Stamped<tf::Pose>& odom_pose,
                      double& x, double& y, double& yaw,
                      const ros::Time& t, const std::string& f)
{
    // Get the robot's pose
    tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0,0,0)), t, f);
    try
    {
        this->tf_->transformPose(odom_frame_id_, ident, odom_pose);
    }
    catch(tf::TransformException e)
    {
        ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
        return false;
    }
    x = odom_pose.getOrigin().x();
    y = odom_pose.getOrigin().y();
    double pitch,roll;
    odom_pose.getBasis().getEulerYPR(yaw, pitch, roll);

    return true;
}


pf_vector_t
AmclNode::uniformPoseGenerator(void* arg)
{
    map_t* map = (map_t*)arg;
#if NEW_UNIFORM_SAMPLING
    unsigned int rand_index = drand48() * free_space_indices.size();
    std::pair<int,int> free_point = free_space_indices[rand_index];
    pf_vector_t p;
    p.v[0] = MAP_WXGX(map, free_point.first);
    p.v[1] = MAP_WYGY(map, free_point.second);
    p.v[2] = drand48() * 2 * M_PI - M_PI; //KPM p.v[2] = 0;
#else
    double min_x, max_x, min_y, max_y;

    min_x = (map->size_x * map->scale)/2.0 - map->origin_x;
    max_x = (map->size_x * map->scale)/2.0 + map->origin_x;
    min_y = (map->size_y * map->scale)/2.0 - map->origin_y;
    max_y = (map->size_y * map->scale)/2.0 + map->origin_y;

    pf_vector_t p;

    ROS_DEBUG("Generating new uniform sample");
    for(;;)
    {
        p.v[0] = min_x + drand48() * (max_x - min_x);
        p.v[1] = min_y + drand48() * (max_y - min_y);
        p.v[2] = drand48() * 2 * M_PI - M_PI;
        // Check that it's a free cell
        int i,j;
        i = MAP_GXWX(map, p.v[0]);
        j = MAP_GYWY(map, p.v[1]);
        if(MAP_VALID(map,i,j) && (map->cells[MAP_INDEX(map,i,j)].occ_state == -1))
            break;
    }
#endif
    return p;
}

pf_vector_t AmclNode::  dualMCL_PoseGenerator(void* arg, double landmark_r, double landmark_phi, double landmark_x_param,
                                              double landmark_y_param){
    pf_vector_t newPose;

    map_t* map = (map_t*) arg;

    double landmark_x = MAP_WXGX(map, landmark_x_param); //222
    double landmark_y = MAP_WYGY(map, map->size_y - landmark_y_param); // 480-195

    ROS_DEBUG("landmark_loc_x: %f = landmark_x: %f", landmark_x_param, landmark_x);
    ROS_DEBUG("landmark_loc_y: %f = landmark_y: %f \n", landmark_y_param, landmark_y);

    if (landmark_r <= 0){
        newPose = AmclNode::uniformPoseGenerator(arg);
        //newPose.v[0] = map->size_x;
        //newPose.v[1] = map->size_y;
        //newPose.v[2] = 0;
        ROS_DEBUG("Random Pose_1: %f, %f, %f", newPose.v[0], newPose.v[1], newPose.v[2]);
        return newPose;
    }

    if( (landmark_phi < ((-28.5)*M_PI/180) ) || (landmark_phi > ((28.5)*M_PI/180)) ){
        newPose = AmclNode::uniformPoseGenerator(arg);
        //newPose.v[0] = map->size_x;
        //newPose.v[1] = map->size_y;
        //newPose.v[2] = 0;
        ROS_DEBUG("Random Pose_2: %f, %f, %f", newPose.v[0], newPose.v[1], newPose.v[2]);
        return newPose;
    }

    double gamma = 0;

    if(landmark_r > 0){
        for(;;){
            double max_phi_range = 2 * M_PI;
            gamma = drand48() * max_phi_range;

            double landmark_size_ambiguity = (drand48() - 0.5) * 0.1;

            double landmark_phi_ambiguity = (drand48() - 0.5) * 0.1 * M_PI / 180;

            newPose.v[0] = (landmark_r + landmark_size_ambiguity) *  cos(gamma) + landmark_x;
            newPose.v[1] = (landmark_r + landmark_size_ambiguity) *  sin(gamma) + landmark_y;

            newPose.v[2] = gamma - (max_phi_range / 2.0) - (landmark_phi + landmark_phi_ambiguity);

            int i = MAP_GXWX(map, newPose.v[0]);
            int j = MAP_GYWY(map, newPose.v[1]);
            if(MAP_VALID(map, i, j) && (map->cells[MAP_INDEX(map, i, j)].occ_state == -1))
                break;
        }
    }

    return newPose;
}


bool
AmclNode::globalLocalizationCallback(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& res)
{
    if( map_ == NULL ) {
        return true;
    }
    boost::recursive_mutex::scoped_lock gl(configuration_mutex_);
    ROS_INFO("Initializing with uniform distribution");
    pf_init_model(pf_, (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                  (void *)map_);
    ROS_INFO("Global initialisation done!");
    pf_init_ = false;
    return true;
}


/* KPM
  * This function is the color subscriber function that sets the color_angles array
  */

void
AmclNode::colorReceived(const cmvision::BlobsConstPtr &Blobs){
    const cmvision::Blobs Blob_holder = *Blobs;
    //ROS_INFO("Blob received...blob count: [%d]", Blob_holder.blob_count);

    int blob_index = 0;
    int i = 0;

    int color_count = 0;


    /**Non-Contiguous Assumption**/
    /*
      // This is for when we assume that the landmark can have holes in it that correspond to
      // the places where color is not detected
    while( blob_index < (int)Blob_holder.blob_count){
        while(i< 640-(int)Blob_holder.blobs[blob_index].right){
            AmclNode::color_angles[i++] = false;
        }
        if(i >= (640-Blob_holder.blobs[blob_index].right)){
            while(i <= (640-Blob_holder.blobs[blob_index].left)){
                AmclNode::color_angles[i++] = true;
            }
            blob_index++;
            //landmark_types++;
            //type_object++;
            //landmark_phi = ((((640-Blob_holder.blobs[blob_index].right) + (640-Blob_holder.blobs[blob_index].left))/2) - 320) * RADIANS_PER_PIXEL;
        }
    }
    */
    /****/



    /**Contiguous Assumption**/
    // This is for when we assume that landmark is contiguous even
    // if color is sometimes not detected in between two blobs

    if((int)Blob_holder.blob_count > 0){
        while (i <= 640 - (int)Blob_holder.blobs[0].right){
            AmclNode::color_angles[i++] = false;
        }
        while (i <= 640-(int)Blob_holder.blobs[(int)Blob_holder.blob_count - 1].left){
            AmclNode::color_angles[i++] = true;
            color_count++;
        }
    }
    /****/

    // This is true regardless of whether landmark is assumed contiguous or not.
    while(i <= 640){
        AmclNode::color_angles[i++] = false;
    }

    // Uncomment if you want to see the color_angles values!
    //    std::stringbuf buffer;
    //    std::ostream os (&buffer);
    //    for (size_t j = 0; j < 640; j++){
    //        if (AmclNode::color_angles[j])
    //            os << "T" << "";
    //        else
    //            os << "F" << "";
    //    }
    //    ROS_INFO("%s", buffer.str().c_str());

    occlusion_proportion = color_count / 640.0;

    return;
}

void
AmclNode::laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
    if( map_ == NULL ) {
        return;
    }

    if ( learn_criteria ){
        // TODO: Not sure if it should reside here or not! :)
        // SINA: Calling this to learn the HMM
        learn_HMM();

        // TODO: It should start collecting samples for a while and then learn so it should be false
        learn_criteria = false;
    }

    boost::recursive_mutex::scoped_lock lr(configuration_mutex_);
    int laser_index = -1;

    // Do we have the base->base_laser Tx (transform) yet?
    if(frame_to_laser_.find(laser_scan->header.frame_id) == frame_to_laser_.end())
    {
        ROS_DEBUG("Setting up laser %d (frame_id=%s)\n", (int)frame_to_laser_.size(), laser_scan->header.frame_id.c_str());
        lasers_.push_back(new AMCLLaser(*laser_));
        lasers_update_.push_back(true);
        laser_index = frame_to_laser_.size();


        // This creates a time-stamped Pose that has the following values:
        // It is a Pose that is an identity (0,0,0,0) (x,y,z,theta) and
        // It is time-stamped with the time ros::Time()
        // It has the frame_id (i.e. like its uniquely identifiable name) of the laser that we are looking at right now
        //
        // In short, we are creating a new frame for the laser and initializing the location of
        // the laser within that frame at (0,0,0,0) and current time.
        tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),
                                                   tf::Vector3(0,0,0)),
                                     ros::Time(), laser_scan->header.frame_id);
        tf::Stamped<tf::Pose> laser_pose;
        try
        {
            // This is what is going on here i believe:
            //
            // The TranformListener (tf_) knows the transformation between the frame_ids of
            // the base_frame_id_ and the laser's frame_id (probably gets it from gazebo? or
            // might be from the turtlebot_description package)
            // Thus, it uses those (from base_frame_id_ and ident) to get the transform
            // between the two frames so as to get the relative pose of the laser from the
            // robot's base. That is, the laser's pose in the frame of the robot's base_frame.

            this->tf_->transformPose(base_frame_id_, ident, laser_pose);
        }
        catch(tf::TransformException& e)
        {
            ROS_ERROR("Couldn't transform from %s to %s, "
                      "even though the message notifier is in use",
                      laser_scan->header.frame_id.c_str(),
                      base_frame_id_.c_str());
            return;
        }

        pf_vector_t laser_pose_v;
        laser_pose_v.v[0] = laser_pose.getOrigin().x();
        laser_pose_v.v[1] = laser_pose.getOrigin().y();
        // laser mounting angle gets computed later -> set to 0 here!
        laser_pose_v.v[2] = 0;
        lasers_[laser_index]->SetLaserPose(laser_pose_v);
        ROS_DEBUG("Received laser's pose wrt robot: %.3f %.3f %.3f",
                  laser_pose_v.v[0],
                laser_pose_v.v[1],
                laser_pose_v.v[2]);

        frame_to_laser_[laser_scan->header.frame_id] = laser_index;
    } else {
        // we have the laser pose, retrieve laser index
        laser_index = frame_to_laser_[laser_scan->header.frame_id];
    }

    // Where was the robot when this scan was taken?
    tf::Stamped<tf::Pose> odom_pose;
    pf_vector_t pose;
    if(!getOdomPose(odom_pose, pose.v[0], pose.v[1], pose.v[2],
                    laser_scan->header.stamp, base_frame_id_))
    {
        ROS_ERROR("Couldn't determine robot's pose associated with laser scan");
        return;
    }

    pf_vector_t delta = pf_vector_zero();

    if(pf_init_)
    {
        // Compute change in pose
        //delta = pf_vector_coord_sub(pose, pf_odom_pose_);
        delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
        delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
        delta.v[2] = angle_diff(pose.v[2], pf_odom_pose_.v[2]);

        // See if we should update the filter
        bool update = fabs(delta.v[0]) > d_thresh_ ||
                fabs(delta.v[1]) > d_thresh_ ||
                fabs(delta.v[2]) > a_thresh_;

        // Set the laser update flags
        if(update)
            for(unsigned int i=0; i < lasers_update_.size(); i++)
                lasers_update_[i] = true;
    }

    bool force_publication = false;
    if(!pf_init_)
    {
        // Pose at last filter update
        pf_odom_pose_ = pose;

        // Filter is now initialized
        pf_init_ = true;

        // Should update sensor data
        for(unsigned int i=0; i < lasers_update_.size(); i++)
            lasers_update_[i] = true;

        force_publication = true;

        resample_count_ = 0;
    }
    // If the robot has moved, update the filter
    else if(pf_init_ && lasers_update_[laser_index])
    {
        //printf("pose\n");
        //pf_vector_fprintf(pose, stdout, "%.3f");

        AMCLOdomData odata;
        odata.pose = pose;

        // Modify the delta in the action data so the filter gets
        // updated correctly
        odata.delta = delta;

        odata.time = (ros::Time::now().toSec() - hmm_use_time.toSec());
        hmm_use_time = ros::Time::now();
        odata.observations = &observations;

        if (propagate_based_on_observation){
            odata.nested_velocity.v[0] = velocity_samples[1].v[0];
            odata.nested_velocity.v[1] = velocity_samples[1].v[1];

            if (hmm.initialized_()){
                int number_of_forward_samples = 100;

                if (observations.size() > 50){
                    observations = vector<Observation>(observations.end() - 30, observations.end());
                    ROS_INFO("Shrinking the observation vector to the last 30 only!");
                }

                DETree result_alpha = hmm.forward(&observations, number_of_forward_samples);
                Sampler sampler;

                Sample sample_vel1;
                sample_vel1 = sampler.sample(&result_alpha);

                int counter = 0;
                while (std::abs(sample_vel1.values[0] - velocity_samples[1].v[0]) > 0.1
                       || std::abs(sample_vel1.values[1] - velocity_samples[1].v[1]) > 0.1){
                    sample_vel1 = sampler.sample(&result_alpha);
                    counter++;

                    if (counter > 20){
                        sample_vel1.values[0] = velocity_samples[1].v[0];
                        sample_vel1.values[1] = velocity_samples[1].v[1];
                        break;
                    }
                }

                double my_yaw = tf::getYaw(last_published_pose.pose.pose.orientation);
                pf_vector_t sample;
                sample.v[0] = sample_vel1.values[0];
                sample.v[1] = sample_vel1.values[1];
                if (std::abs(my_yaw - pf_vector_angle(sample)) > M_PI / 6){
                    double s = std::sqrt(std::pow(sample_vel1.values[0], 2) + std::pow(sample_vel1.values[1], 2));
                    sample_vel1.values[0] = s * std::cos(my_yaw);
                    sample_vel1.values[1] = s * std::sin(my_yaw);
                }

                odata.nested_velocity.v[0] = (sample_vel1.values[0] + odata.nested_velocity.v[0]) / 2.0;
                odata.nested_velocity.v[1] = (sample_vel1.values[1] + odata.nested_velocity.v[1]) / 2.0;

                ROS_WARN("Velocity Sample: %f, %f",
                         sample_vel1.values[0],
                        sample_vel1.values[1]
                        );

                // Setting the last velocity sample to the one estimated now!
                velocity_samples[1].v[0] = odata.nested_velocity.v[0];
                velocity_samples[1].v[1] = odata.nested_velocity.v[1];
            }

            propagate_based_on_observation = false;
        }else{
            if (hmm.initialized_()){
                int number_of_forward_samples = 100;

                if (observations.size() > 50){
                    observations = vector<Observation>(observations.end() - 30, observations.end());
                    ROS_INFO("Shrinking the observation vector to the last 30 only!");
                }

                DETree result_alpha = hmm.forward(&observations, number_of_forward_samples);
                Sampler sampler;

                Sample sample_vel1;
                sample_vel1 = sampler.sample(&result_alpha);

                int counter = 0;
                while (std::abs(sample_vel1.values[0] - velocity_samples[1].v[0]) > 0.1
                       || std::abs(sample_vel1.values[1] - velocity_samples[1].v[1]) > 0.1){
                    sample_vel1 = sampler.sample(&result_alpha);
                    counter++;

                    if (counter > 20){
                        sample_vel1.values[0] = velocity_samples[1].v[0];
                        sample_vel1.values[1] = velocity_samples[1].v[1];
                        break;
                    }
                }

                double my_yaw = tf::getYaw(last_published_pose.pose.pose.orientation);
                pf_vector_t sample;
                sample.v[0] = sample_vel1.values[0];
                sample.v[1] = sample_vel1.values[1];
                ROS_ERROR("My Yaw: %f", my_yaw);
                ROS_ERROR("Vel Orien: %f", pf_vector_angle(sample));
                ROS_ERROR("Diff Abs: %f", std::abs(my_yaw - pf_vector_angle(sample)));
                if (std::abs(my_yaw - pf_vector_angle(sample)) > M_PI / 6){
                    double cor = drand48() * M_PI / 6 - M_PI / 12;
                    double s = std::sqrt(std::pow(sample_vel1.values[0], 2) + std::pow(sample_vel1.values[1], 2));
                    sample_vel1.values[0] = s * std::cos(my_yaw + cor);
                    sample_vel1.values[1] = s * std::sin(my_yaw + cor);
                }

                if (std::abs(sample_vel1.values[0]) > std::abs(sample_vel1.values[1])){
                    sample_vel1.values[0] *= 1.2;
                }else{
                    sample_vel1.values[1] *= 1.2;
                }

                odata.nested_velocity.v[0] = sample_vel1.values[0];
                odata.nested_velocity.v[1] = sample_vel1.values[1];

                ROS_WARN("Velocity Sample: %f, %f",
                         sample_vel1.values[0],
                        sample_vel1.values[1]
                        );

                // Setting the last velocity sample to the one estimated now!
                velocity_samples[1].v[0] = odata.nested_velocity.v[0];
                velocity_samples[1].v[1] = odata.nested_velocity.v[1];

                ROS_DEBUG("Observation Size: %d, M Size: %d, V Size: %d", observations.size(), m_.size(), v_.size());

            }else{
                ROS_WARN("No HMM and no obsevations, just use the previous velocity as our current velocity!!!");
                odata.nested_velocity.v[0] = velocity_samples[1].v[0];
                odata.nested_velocity.v[1] = velocity_samples[1].v[1];

                if (std::isnan(velocity_samples[1].v[0])){
                    odata.nested_velocity.v[0] = 0.0;
                }

                if (std::isnan(velocity_samples[1].v[1])){
                    odata.nested_velocity.v[1] = 0.0;
                }
            }
        }

        if (odata.time > 1.2){
            odata.time = 1.2;
        }

        double max_speed_thresh = 0.6;

        if (odata.nested_velocity.v[0] > max_speed_thresh){
            odata.nested_velocity.v[0] = max_speed_thresh;
        }else if (odata.nested_velocity.v[0] < -max_speed_thresh){
            odata.nested_velocity.v[0] = -max_speed_thresh;
        }
        if (odata.nested_velocity.v[1] > max_speed_thresh){
            odata.nested_velocity.v[1] = max_speed_thresh;
        }else if (odata.nested_velocity.v[1] < -max_speed_thresh){
            odata.nested_velocity.v[1] = -max_speed_thresh;
        }

        if (std::isnan(odata.nested_velocity.v[2])){
            odata.nested_velocity.v[2] = 0.0;
        }

        double old_vel_angle = pf_vector_angle(velocity_samples[0]);
        double new_vel_angle = pf_vector_angle(velocity_samples[1]);

        odata.velocity_angle_diff = (old_vel_angle - new_vel_angle);
        if(std::isnan(odata.velocity_angle_diff)){
            odata.velocity_angle_diff = 0.0;
        }

        if (std::abs(odata.velocity_angle_diff) > M_PI / 4){ // Limiting the angle correction to M_PI/4 to
            //  avoid swirl effect in the nested particles
            odata.velocity_angle_diff = M_PI / 4;
            if (odata.velocity_angle_diff < 0)
                odata.velocity_angle_diff *= -1;
        }

        this->angle_correction = odata.velocity_angle_diff;

        ROS_WARN("Delta Time: %f", odata.time);

        ROS_WARN("Estimated Velocity: %f, %f",
                 odata.nested_velocity.v[0],
                odata.nested_velocity.v[1]
                );

        ROS_WARN("Estimated Delta: %f, %f",
                 odata.nested_velocity.v[0] * odata.time,
                odata.nested_velocity.v[1] * odata.time
                );

        //        ROS_WARN("Agnle Correction: %f", odata.velocity_angle_diff);

        /**
          Notes for me:
          UpdateAction essentially applies the transition function to all samples present in the sets of samples that are contained
          in "pf_" . There are two sets of samples in pf_
          It receives the odom data ...which is the current pose, and the delta between current pose and earlier pose. This data
          is used as the "action" which is used to sample the particles to proliferate them to to current set of pose hypotheses.
          Appropriate noise is included (for both rotation and translation) and the sampling is done using a zero-mean gaussian.
          */
        // Use the action data to update the filter
        odom_->UpdateAction(pf_, (AMCLSensorData*)&odata);

    }

    bool resampled = false;
    // If the robot has moved, update the filter
    if(lasers_update_[laser_index])
    {
        AMCLLaserData ldata;
        ldata.sensor = lasers_[laser_index];
        ldata.range_count = laser_scan->ranges.size();
        int color_index_floor = 0;
        double color_min_angle = COLOR_MAX_ANGLE; //since we need absolute value and the max and min are the same but with opposite signs


        /* Collect true pose from gazebo */

        true_pose_service.request.model_name = std::string("Robot1");
        if (true_pose_client.call(true_pose_service))
        {
            //            tf::Vector3 origin_map(0, 0, 0);
            //            tf::Quaternion quaternion_map(0, 0, -.15);

            //            tf::Transform map_rotation;
            //            map_rotation.setOrigin(origin_map);
            //            map_rotation.setRotation(quaternion_map);

            //            tf::Vector3 origin_pose(true_pose_service.response.pose.position.x, true_pose_service.response.pose.position.y, 0);
            //            tf::Quaternion quaternion_pose(0, 0, 0);

            //            tf::Transform pose_tansform;
            //            pose_tansform.setOrigin(origin_pose);
            //            pose_tansform.setRotation(quaternion_pose);

            //            tf::Transform result = pose_tansform * map_rotation;

            //            true_pose_normal.v[0] = result.getOrigin().x();
            //            true_pose_normal.v[1] = result.getOrigin().y();
            //            true_pose_normal.v[2] = 0.0;

            true_pose_normal.v[0] = true_pose_service.response.pose.position.x;
            true_pose_normal.v[1] = true_pose_service.response.pose.position.y;
            true_pose_normal.v[2] = 0.0;

            ROS_DEBUG("\n Normal true pose \n\t x: %f \n\t y: %f \n",
                      true_pose_normal.v[0],
                    true_pose_normal.v[1]);
        }
        else
        {
            ROS_ERROR("Failed to call service gazebo/get_model_state");
            //exit(1);
        }

        true_pose_service.request.model_name = std::string("Robot2");
        if (true_pose_client.call(true_pose_service))
        {
            //            tf::Vector3 origin_map(0, 0, 0);
            //            tf::Quaternion quaternion_map(0, 0, -.15);

            //            tf::Transform map_rotation;
            //            map_rotation.setOrigin(origin_map);
            //            map_rotation.setRotation(quaternion_map);

            //            tf::Vector3 origin_pose(true_pose_service.response.pose.position.x, true_pose_service.response.pose.position.y, 0);
            //            tf::Quaternion quaternion_pose(0, 0, 0);

            //            tf::Transform pose_tansform;
            //            pose_tansform.setOrigin(origin_pose);
            //            pose_tansform.setRotation(quaternion_pose);

            //            tf::Transform result = pose_tansform * map_rotation;

            //            true_pose_nested.v[0] = result.getOrigin().x();
            //            true_pose_nested.v[1] = result.getOrigin().y();
            //            true_pose_nested.v[2] = 0.0;

            true_pose_nested.v[0] = true_pose_service.response.pose.position.x;
            true_pose_nested.v[1] = true_pose_service.response.pose.position.y;
            true_pose_nested.v[2] = 0.0;

            ROS_DEBUG("\n Nested true pose \n\t x: %f \n\t y: %f \n",
                      true_pose_nested.v[0],
                    true_pose_nested.v[1]);
        }
        else
        {
            ROS_ERROR("Failed to call service gazebo/get_model_state");
            //exit(1);
        }

        /* *** */

        //ROS_INFO("\n ldata.range_count: %d",ldata.range_count);

        // To account for lasers that are mounted upside-down, we determine the
        // min, max, and increment angles of the laser in the base frame.
        //
        // Construct min and max angles of laser, in the base_link frame.
        tf::Quaternion q;
        q.setRPY(0.0, 0.0, laser_scan->angle_min);
        tf::Stamped<tf::Quaternion> min_q(q, laser_scan->header.stamp,
                                          laser_scan->header.frame_id);
        q.setRPY(0.0, 0.0, laser_scan->angle_min + laser_scan->angle_increment);
        tf::Stamped<tf::Quaternion> inc_q(q, laser_scan->header.stamp,
                                          laser_scan->header.frame_id);
        try
        {
            tf_->transformQuaternion(base_frame_id_, min_q, min_q);
            tf_->transformQuaternion(base_frame_id_, inc_q, inc_q);
        }
        catch(tf::TransformException& e)
        {
            ROS_WARN("Unable to transform min/max laser angles into base frame: %s",
                     e.what());
            return;
        }

        double angle_min = tf::getYaw(min_q);
        double angle_increment = tf::getYaw(inc_q) - angle_min;

        // The thing to note here is that both these minimum angles are actually negative,
        // so we need to take their absolute values
        double difference_in_min_angles = fabs(angle_min) - fabs(color_min_angle);

        //ROS_INFO("\n       angle_min: %f", angle_min);
        //ROS_INFO("\n angle_increment: %f", angle_increment);


        // wrapping angle to [-pi .. pi]
        angle_increment = fmod(angle_increment + 5*M_PI, 2*M_PI) - M_PI;

        ROS_DEBUG("Laser %d angles in base frame: min: %.3f inc: %.3f", laser_index, angle_min, angle_increment);

        // Apply range min/max thresholds, if the user supplied them
        if(laser_max_range_ > 0.0)
            ldata.range_max = std::min(laser_scan->range_max, (float)laser_max_range_);
        else
            ldata.range_max = laser_scan->range_max;
        double range_min;
        if(laser_min_range_ > 0.0)
            range_min = std::max(laser_scan->range_min, (float)laser_min_range_);
        else
            range_min = laser_scan->range_min;
        // The AMCLLaserData destructor will free this memory
        ldata.ranges = new double[ldata.range_count][3];//@KPM:changing array size from 2 to 3 to accomodate color
        ROS_ASSERT(ldata.ranges);

        int blob_ray_count = 0;

        //KPM: initializing landmark readings
        double temp_landmark_phi = 0;
        double temp_landmark_r = 0;
        double landmark_r_distances[max_beams_];
        bool isMaxRangeIncluded = false;

        other_robot_distance = 0.0;
        landmark_r = 0.0;
        ldata.isLandmarkObserved = false;
        ldata.landmark_r = 0;
        ldata.landmark_phi = 0;
        ldata.color_beams = 0;


        for(int i = 0; i < ldata.range_count; i++)
        {
            //ROS_INFO("range: %f",laser_scan->ranges[i]);
            // amcl doesn't (yet) have a concept of min range.  So we'll map short
            // readings to max range.

            if(laser_scan->ranges[i] <= range_min)
                ldata.ranges[i][0] = ldata.range_max;
            else
                ldata.ranges[i][0] = laser_scan->ranges[i];

            // Compute bearing
            ldata.ranges[i][1] = angle_min + (i * angle_increment);

            /**
             * @KPM:
             * Add calculation of camera frame angle in radians of brown pixels (from right).
             * Compare this to i*angle_increment and if it matches then set ldata.ranges[i][2] to 50 for that i
             *
             */

            // KPM: I think difference_in_min_angles should actually be added,
            // since its positive when laser min angle is more than color min angle
            color_index_floor = (int)floor(((i * angle_increment) + difference_in_min_angles) / RADIANS_PER_PIXEL);
            if( (color_index_floor >= 0) && (color_index_floor <= 639) ){ //checks to see if colour has been detected at all
                if(AmclNode::color_angles[color_index_floor] || AmclNode::color_angles[color_index_floor + 1]){
                    ldata.ranges[i][2] = 50;
                    ldata.color_beams++;

                    if (blob_ray_count >= max_beams_) ROS_FATAL("\n"
                                                                "\tArray out of range can  occur!\n"
                                                                "\tPlease increase the laser_max_beams parameter.");

                    // Only include one max ranged sensory input
                    // ...reject all other max range readings
                    if(ldata.ranges[i][0] == ldata.range_max){
                        if(isMaxRangeIncluded == false && !std::isnan(ldata.ranges[i][0])){
                            temp_landmark_r += ldata.ranges[i][0];
                            landmark_r_distances[blob_ray_count] = ldata.ranges[i][0];
                            blob_ray_count++;

                            isMaxRangeIncluded = true;
                        }
                    }
                    else{
                        if (!std::isnan(ldata.ranges[i][0])){
                            temp_landmark_r += ldata.ranges[i][0];
                            landmark_r_distances[blob_ray_count] = ldata.ranges[i][0];
                            blob_ray_count++;
                        }
                    }

                    temp_landmark_phi += (angle_min + (i * angle_increment));
                    ROS_DEBUG("landmark_r: %f \t landmark_phi: %f \t blob_ray_count: %d", ldata.ranges[i][0], (angle_min + (i * angle_increment)), blob_ray_count + 1 );

                }
                else{
                    if(temp_landmark_r > 0){
                        //landmark_r = temp_landmark_r/blob_ray_count;
                        landmark_phi = temp_landmark_phi / blob_ray_count;

                        qsort(landmark_r_distances, blob_ray_count, sizeof(int) ,compare_int);

                        landmark_r = landmark_r_distances[blob_ray_count/2];
                        other_robot_distance = landmark_r;

                        ldata.landmark_r = landmark_r;
                        ldata.landmark_phi = landmark_phi;
                        ldata.isLandmarkObserved = true;

                        landmark_r_sample = landmark_r;
                        landmark_phi_sample = landmark_phi;
                        propagate_based_on_observation = true;

                        ROS_DEBUG("final landmark_r: %f \t\t landmark_phi: %f \n",landmark_r,landmark_phi);
                    }
                    temp_landmark_r = 0;
                    temp_landmark_phi = 0;
                    blob_ray_count = 0;
                    ldata.ranges[i][2] = 0;
                }
            }
            else{
                ldata.ranges[i][2] = 0;
                // temp_landmark_r = 0;
                // temp_landmark_phi = 0;
                // blob_ray_count = 0;
            }
        }


        /**
          Notes for me:
          UpdateSensor function calculates the weights for the samples from the sensor readings.
        */

        //        if (propagate_based_on_observation){
        lasers_[laser_index]->UpdateSensor(pf_, (AMCLSensorData*)&ldata);
        //        }

        lasers_update_[laser_index] = false;

        pf_odom_pose_ = pose;

        // Resample the particles
        if(!(++resample_count_ % resample_interval_))
        {
            // Initialize the filter
            pf_vector_t pf_init_leader_pose_mean = pf_vector_zero();
            pf_init_leader_pose_mean.v[0] = init_leader_pose_[0];
            pf_init_leader_pose_mean.v[1] = init_leader_pose_[1];
            pf_init_leader_pose_mean.v[2] = init_leader_pose_[2];
            pf_matrix_t pf_init_leader_pose_cov = pf_matrix_zero();
            pf_init_leader_pose_cov.m[0][0] = init_leader_cov_[0];
            pf_init_leader_pose_cov.m[1][1] = init_leader_cov_[1];
            pf_init_leader_pose_cov.m[2][2] = init_leader_cov_[2];

            pf_vector_t leader_pose_temp;
            leader_pose_temp.v[0] = nested_last_published_pose.pose.pose.position.x;
            leader_pose_temp.v[1] = nested_last_published_pose.pose.pose.position.y;
            leader_pose_temp.v[2] = tf::getYaw(nested_last_published_pose.pose.pose.orientation);

            // landmark_loc_x and y are zero all the time. They are not used in resampling
            pf_update_resample(pf_, landmark_r, landmark_phi, landmark_loc_x, landmark_loc_y,
                               pf_init_leader_pose_mean,
                               pf_init_leader_pose_cov,
                               leader_pose_temp, velocity_samples[1]);
            landmark_r = 0;
            landmark_phi = 0;


            if (total_nested_particle_count > 0){
                init_leader_pose_[0] = init_leader_pose_[1] = init_leader_pose_[2] = 0.0;
                init_leader_cov_[0] = init_leader_cov_[1] = init_leader_cov_[2] = 0.0;
            }

            resampled = true;
        }

        pf_sample_set_t* set = pf_->sets + pf_->current_set;
        ROS_DEBUG("Num samples: %d\n", set->sample_count);

        // Temp variables to calculate Normal particle Mean Squared Error
        double squared_error = 0.0;

        // Publish the resulting cloud
        // TODO: set maximum rate for publishing
        geometry_msgs::PoseArray cloud_msg;
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = global_frame_id_;
        cloud_msg.poses.resize(set->sample_count);

        normal_particles_within_1m = 0;
        for(int i = 0;i < set->sample_count; i++)
        {
            tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(set->samples[i].pose.v[2]),
                            tf::Vector3(set->samples[i].pose.v[0],
                            set->samples[i].pose.v[1], 0)),
                    cloud_msg.poses[i]);

            double current_SE = ( (set->samples[i].pose.v[0]- true_pose_normal.v[0])
                    *(set->samples[i].pose.v[0]- true_pose_normal.v[0])
                    +
                    (set->samples[i].pose.v[1]- true_pose_normal.v[1])
                    *(set->samples[i].pose.v[1]- true_pose_normal.v[1])
                    );

            squared_error = squared_error + current_SE;

            if( sqrt(current_SE) <= 1.0 )
                normal_particles_within_1m++;
        }

        normal_MSE = squared_error / set->sample_count ;
        normal_RootMSE = sqrt(normal_MSE);

        ROS_DEBUG("normal_MSE: %f", normal_MSE);

        particlecloud_pub_.publish(cloud_msg);

        /** Publish the nested particle cloud **/
        if(pf_->nesting_lvl > 0){
            // Publish the resulting nested particle cloud
            // TODO: set maximum rate for publishing
            geometry_msgs::PoseArray nested_cloud_msg;
            nested_cloud_msg.header.stamp = ros::Time::now();
            nested_cloud_msg.header.frame_id = global_frame_id_;
            nested_cloud_msg.poses.resize(0);

            //ROS_INFO("pf_->nesting_lvl : %d", pf_->nesting_lvl);

            pf_t *nested_pf_set;
            pf_t *nested_pf;


            pf_sample_set_t *upper_particles_set = pf_->sets + pf_->current_set;
            pf_sample_set_t *nested_particles_set;


            nested_pf_set = pf_get_this_nested_set(pf_, pf_->current_set);

            int curr_total_nested_particle_count = 0;
            double nested_squaredError = 0.0;

            nested_particles_within_1m = 0;
            for(int i = 0; i < upper_particles_set->sample_count; i++){

                //                nested_pf_set = pf_->nested_pf_sets[i][pf_->current_set];
                nested_pf = nested_pf_set + i;
                nested_particles_set = nested_pf->sets + nested_pf->current_set;

                nested_cloud_msg.poses.resize(nested_cloud_msg.poses.size() + nested_particles_set->sample_count);

                for(int j = 0; j < nested_particles_set->sample_count; j++)
                {
                    tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(nested_particles_set->samples[j].pose.v[2]),
                                    tf::Vector3(nested_particles_set->samples[j].pose.v[0],
                                    nested_particles_set->samples[j].pose.v[1], 0)),
                            nested_cloud_msg.poses[curr_total_nested_particle_count++]);


                    double current_nested_SE = ( (nested_particles_set->samples[j].pose.v[0]- true_pose_nested.v[0])
                            *(nested_particles_set->samples[j].pose.v[0]- true_pose_nested.v[0])
                            +
                            (nested_particles_set->samples[j].pose.v[1]- true_pose_nested.v[1])
                            *(nested_particles_set->samples[j].pose.v[1]- true_pose_nested.v[1])
                            );

                    nested_squaredError = nested_squaredError + current_nested_SE;

                    if( sqrt(current_nested_SE) <= 1.0 )
                        nested_particles_within_1m++;

                }
            }

            nested_MSE = nested_squaredError / curr_total_nested_particle_count;
            nested_RootMSE = sqrt(nested_MSE);
            ROS_DEBUG("Nested normal_MSE: %f", nested_MSE);

            nested_particlecloud_pub_.publish(nested_cloud_msg);

            //printf(" This is getting printed! Yay!!!");
            ROS_INFO("\n\n\t normal_particles:\t\t %d \n"
                     "\t\t Avg Weight: %f \n"
                     "\t\t Covariance: \n"
                     "\t\t\t %f \t \t \n"
                     "\t\t\t \t %f \t \n"
                     "\t\t\t \t \t %f \n\n"
                     "\t nested_particles in one pool:\t %d \n"
                     "\t Total nested_particles:\t %d\n"
                     "\t Grand Total of particles:\t %d\n",
                     pf_->sets[pf_->current_set].sample_count,
                    pf_->sets[pf_->current_set].avg_weight,
                    pf_->sets[pf_->current_set].cov.m[0][0],
                    //pf_->sets[pf_->current_set].cov.m[0][1],
                    //pf_->sets[pf_->current_set].cov.m[0][2],
                    //pf_->sets[pf_->current_set].cov.m[1][0],
                    pf_->sets[pf_->current_set].cov.m[1][1],
                    //pf_->sets[pf_->current_set].cov.m[1][2],
                    //pf_->sets[pf_->current_set].cov.m[2][0],
                    //pf_->sets[pf_->current_set].cov.m[2][1],
                    pf_->sets[pf_->current_set].cov.m[2][2],
                    nested_particles_set->sample_count,
                    curr_total_nested_particle_count,
                    pf_->sets[pf_->current_set].sample_count + curr_total_nested_particle_count);

            // For Data Collection
            isLandmarkObserved = ldata.isLandmarkObserved;
            nested_particle_count = nested_particles_set->sample_count;
            total_nested_particle_count = curr_total_nested_particle_count;

            /* **** Write to Data Collection File **** */

            /* **** End writing to Data Collection File **** */
        }


    }

    // Calculate the pose to be published as the current pose estimate by AMCL
    if(resampled || force_publication)
    {
        // Read out the current hypotheses
        double max_weight = 0.0;
        int max_weight_hyp = -1;
        std::vector<amcl_hyp_t> hyps;
        hyps.resize(pf_->sets[pf_->current_set].cluster_count);

        for(int hyp_count = 0;
            hyp_count < pf_->sets[pf_->current_set].cluster_count; hyp_count++)
        {
            double weight;
            pf_vector_t pose_mean;
            pf_matrix_t pose_cov;
            if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean, &pose_cov))
            {
                ROS_ERROR("Couldn't get stats on cluster %d", hyp_count);
                break;
            }

            hyps[hyp_count].weight = weight;
            hyps[hyp_count].pf_pose_mean = pose_mean;
            hyps[hyp_count].pf_pose_cov = pose_cov;

            if(hyps[hyp_count].weight > max_weight)
            {
                max_weight = hyps[hyp_count].weight;
                max_weight_hyp = hyp_count;
            }
        }

        if(max_weight > 0.0)
        {
            ROS_DEBUG("Max weight pose: %.3f %.3f %.3f",
                      hyps[max_weight_hyp].pf_pose_mean.v[0],
                    hyps[max_weight_hyp].pf_pose_mean.v[1],
                    hyps[max_weight_hyp].pf_pose_mean.v[2]);

            /*
         puts("");
         pf_matrix_fprintf(hyps[max_weight_hyp].pf_pose_cov, stdout, "%6.3f");
         puts("");
       */

            geometry_msgs::PoseWithCovarianceStamped p;
            // Fill in the header
            p.header.frame_id = global_frame_id_;
            p.header.stamp = laser_scan->header.stamp;


            // Copy in the pose

            /* Best estimate of pose by amcl -- this is the actual AMCL pose we need */
            p.pose.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
            p.pose.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];

            /* Using TRUE POSE for the moment to capture videos...TURN THIS OFF for actual experiments */
            //p.pose.pose.position.x = true_pose_normal.v[0];
            //p.pose.pose.position.y = true_pose_normal.v[1];


            tf::quaternionTFToMsg(tf::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
                    p.pose.pose.orientation);
            // Copy in the covariance, converting from 3-D to 6-D
            pf_sample_set_t* set = pf_->sets + pf_->current_set;
            for(int i=0; i<2; i++)
            {
                for(int j=0; j<2; j++)
                {
                    // Report the overall filter covariance, rather than the
                    // covariance for the highest-weight cluster
                    //p.covariance[6*i+j] = hyps[max_weight_hyp].pf_pose_cov.m[i][j];
                    p.pose.covariance[6*i+j] = set->cov.m[i][j];
                }
            }
            // Report the overall filter covariance, rather than the
            // covariance for the highest-weight cluster
            //p.covariance[6*5+5] = hyps[max_weight_hyp].pf_pose_cov.m[2][2];
            p.pose.covariance[6*5+5] = set->cov.m[2][2];

            /*
             printf("cov:\n");
             for(int i=0; i<6; i++)
             {
             for(int j=0; j<6; j++)
             printf("%6.3f ", p.covariance[6*i+j]);
             puts("");
             }
           */

            // Publish amcl's best estimate for current pose
            pose_pub_.publish(p);
            last_published_pose = p;

#if COLLECT_DATA
            // Data Collection method
            log_data(p);
#endif

            ///////////////////////Sending true pose


            //            true_pose = true_pose_normal.v[0];
            //            true_pose.position.y = true_pose_normal.v[1];
            //            tf::quaternionTFToMsg(tf::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
            //                                  true_pose.orientation);

            //            geometry_msgs::PoseStamped true_pose;
            //            true_pose.header.frame_id = global_frame_id_;
            //            true_pose.header.stamp = laser_scan->header.stamp + transform_tolerance_;

            //            true_pose.pose.position.x = true_pose_normal.v[0]+0.0;
            //            true_pose.pose.position.y = true_pose_normal.v[1]-0.0;
            //            true_pose.pose.position.z = 0.0;

            //            tf::quaternionTFToMsg(tf::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
            //                    true_pose.pose.orientation);

            //            true_pose.pose.orientation.x = 0.0;
            //            true_pose.pose.orientation.y = 0.0;
            //            true_pose.pose.orientation.z = hyps[max_weight_hyp].pf_pose_mean.v[2];
            //            true_pose.pose.orientation.w = 0.0;

            //true_pose_pub_.publish(true_pose);


            ROS_DEBUG("New pose: %6.3f %6.3f %6.3f",
                      hyps[max_weight_hyp].pf_pose_mean.v[0],
                    hyps[max_weight_hyp].pf_pose_mean.v[1],
                    hyps[max_weight_hyp].pf_pose_mean.v[2]);

            // subtracting base to odom from map to base and send map to odom instead
            tf::Stamped<tf::Pose> odom_to_map;
            // tf::Stamped<tf::Pose> odom_to_map_true;
            try
            {
                /* !!!!!
                   This is the actual ODOM Pose that needs to be published by AMCL.
                   Turning this off for the moment to get videos.
                   This this back on when doing actual experiments!!!
                */

                tf::Transform tmp_tf(tf::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
                        tf::Vector3(hyps[max_weight_hyp].pf_pose_mean.v[0],
                        hyps[max_weight_hyp].pf_pose_mean.v[1],
                        0.0));

                // tf::Transform tmp_tf_true(tf::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]-0.02),
                //                      tf::Vector3(true_pose_normal.v[0]-0.8,
                //                                  true_pose_normal.v[1]-0.0,
                //                                  0.0));



                tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(),
                                                      laser_scan->header.stamp,
                                                      base_frame_id_);

                // tf::Stamped<tf::Pose> tmp_tf_stamped_true (tmp_tf_true.inverse(),
                //                                       laser_scan->header.stamp,
                //                                            "/robot1_tf_true/base_link");


                this->tf_->transformPose(odom_frame_id_,
                                         tmp_tf_stamped,
                                         odom_to_map);

                // this->tf_->transformPose("/robot1_tf_true/odom",
                //                          tmp_tf_stamped_true,
                //                          odom_to_map_true);

            }
            catch(tf::TransformException)
            {
                ROS_ERROR("Failed to subtract base to odom transform");
                return;
            }


            latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                                       tf::Point(odom_to_map.getOrigin()));

            // latest_tf_true = tf::Transform(tf::Quaternion(odom_to_map_true.getRotation()),
            //                            tf::Point(odom_to_map_true.getOrigin()));


            latest_tf_valid_ = true;

            // We want to send a transform that is good up until a
            // tolerance time so that odom can be used
            ros::Time transform_expiration = (laser_scan->header.stamp +
                                              transform_tolerance_);

            tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                                transform_expiration,
                                                global_frame_id_, odom_frame_id_);

            // tf::StampedTransform tmp_tf_stamped_true(latest_tf_true.inverse(),
            //                                     transform_expiration,
            //                                          "/map", "/robot1_tf_true/odom");




            this->tfb_->sendTransform(tmp_tf_stamped);
            // this->tfb_->sendTransform(tmp_tf_stamped_true);

            sent_first_transform_ = true;

        }
        else
        {
            ROS_ERROR("No pose!");
        }

        // Calculate the best hypothesis form the nested particles
        if (nesting_lvl_ > 0){
            // Read out the current hypotheses
            double nested_max_weight = 0.0;
            int nested_max_weight_hyp = -1;
            std::vector<amcl_hyp_t> nested_hyps;

            // TODO
            pf_t *nested_pf_ = pf_get_this_nested_set(pf_, pf_->current_set);

            nested_hyps.resize(nested_pf_->sets[pf_->current_set].cluster_count);
            for(int hyp_count = 0;
                hyp_count < nested_pf_->sets[pf_->current_set].cluster_count; hyp_count++)
            {
                double nested_weight;
                pf_vector_t nested_pose_mean;
                pf_matrix_t nested_pose_cov;
                if (!pf_get_cluster_stats(nested_pf_, hyp_count, &nested_weight, &nested_pose_mean, &nested_pose_cov))
                {
                    ROS_ERROR("Couldn't get stats on cluster %d", hyp_count);
                    break;
                }

                nested_hyps[hyp_count].weight = nested_weight;
                nested_hyps[hyp_count].pf_pose_mean = nested_pose_mean;
                nested_hyps[hyp_count].pf_pose_cov = nested_pose_cov;

                if(nested_hyps[hyp_count].weight > nested_max_weight)
                {
                    nested_max_weight = nested_hyps[hyp_count].weight;
                    nested_max_weight_hyp = hyp_count;
                }
            }

            ROS_DEBUG("HYP Count: %d", nested_pf_->sets[pf_->current_set].cluster_count);
            if (nested_hyps.size() > 0)
                ROS_DEBUG("Hyps: weigt: %f\tPose[0]: %f\tPose[1]: %f\tPose[2]: %f",
                          nested_max_weight,
                          nested_hyps[nested_max_weight_hyp].pf_pose_mean.v[0],
                        nested_hyps[nested_max_weight_hyp].pf_pose_mean.v[1],
                        nested_hyps[nested_max_weight_hyp].pf_pose_mean.v[2]
                        );

            if (nested_max_weight > 0 && nested_hyps.size() > 0){

                geometry_msgs::PoseWithCovarianceStamped nested_p;
                // Fill in the header
                nested_p.header.frame_id = global_frame_id_;
                nested_p.header.stamp = laser_scan->header.stamp;

                // Copy in the pose

                /* Best estimate of pose by amcl -- this is the actual AMCL pose we need */
                nested_p.pose.pose.position.x = nested_hyps[max_weight_hyp].pf_pose_mean.v[0];
                nested_p.pose.pose.position.y = nested_hyps[max_weight_hyp].pf_pose_mean.v[1];

                tf::quaternionTFToMsg(tf::createQuaternionFromYaw(nested_hyps[nested_max_weight_hyp].pf_pose_mean.v[2]),
                        nested_p.pose.pose.orientation);
                // Copy in the covariance, converting from 3-D to 6-D
                pf_sample_set_t* nested_set = pf_get_this_nested_set(pf_, pf_->current_set)->sets + pf_->current_set;
                for(int i = 0; i < 2; i++)
                {
                    for(int j = 0; j < 2; j++)
                    {
                        // Report the overall filter covariance, rather than the
                        // covariance for the highest-weight cluster
                        //p.covariance[6*i+j] = hyps[max_weight_hyp].pf_pose_cov.m[i][j];
                        nested_p.pose.covariance[6 * i + j] = nested_set->cov.m[i][j];
                    }
                }
                // Report the overall filter covariance, rather than the
                // covariance for the highest-weight cluster
                // nested_p.covariance[6 * 5 + 5] = hyps[max_weight_hyp].pf_pose_cov.m[2][2];
                nested_p.pose.covariance[6 * 5 + 5] = nested_set->cov.m[2][2];

                // Publish amcl's best estimate for current pose
                nested_pose_pub_.publish(nested_p);
                nested_last_published_pose = nested_p;
            }else{
                ROS_INFO("No Nested Pose Available!");
            }


            tf::Transform tmp_tf;
            tmp_tf.setOrigin(tf::Vector3(nested_last_published_pose.pose.pose.position.x,
                                         nested_last_published_pose.pose.pose.position.y,
                                         0
                                         ));
            tf::Quaternion q;
            q.setRPY(0, 0, nested_last_published_pose.pose.pose.orientation.w);
            tmp_tf.setRotation(q);

            ros::Time nested_transform_expiration = (laser_scan->header.stamp +
                                                     transform_tolerance_);
            tf::StampedTransform nested_tmp_tf_stamped(tmp_tf,
                                                       nested_transform_expiration,
                                                       global_frame_id_, nested_odom_frame_id_);

            this->tfb_->sendTransform(nested_tmp_tf_stamped);
        }

        collect_sample(&last_published_pose, &nested_last_published_pose, landmark_r_sample, landmark_phi_sample, nested_MSE);

    }
    else if(latest_tf_valid_)
    {
        // Nothing changed, so we'll just republish the last transform, to keep
        // everybody happy.
        ros::Time transform_expiration = (laser_scan->header.stamp +
                                          transform_tolerance_);
        tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                            transform_expiration,
                                            global_frame_id_, odom_frame_id_);
        this->tfb_->sendTransform(tmp_tf_stamped);


        // Is it time to save our last pose to the param server
        ros::Time now = ros::Time::now();
        if((save_pose_period.toSec() > 0.0) &&
                (now - save_pose_last_time) >= save_pose_period)
        {
            // We need to apply the last transform to the latest odom pose to get
            // the latest map pose to store.  We'll take the covariance from
            // last_published_pose.
            tf::Pose map_pose = latest_tf_.inverse() * odom_pose;
            double yaw, pitch, roll;
            map_pose.getBasis().getEulerYPR(yaw, pitch, roll);

            private_nh_.setParam("initial_pose_x", map_pose.getOrigin().x());
            private_nh_.setParam("initial_pose_y", map_pose.getOrigin().y());
            private_nh_.setParam("initial_pose_a", yaw);
            private_nh_.setParam("initial_cov_xx",
                                 last_published_pose.pose.covariance[6*0+0]);
            private_nh_.setParam("initial_cov_yy",
                                 last_published_pose.pose.covariance[6*1+1]);
            private_nh_.setParam("initial_cov_aa",
                                 last_published_pose.pose.covariance[6*5+5]);
            save_pose_last_time = now;
        }
    }

}

double
AmclNode::getYaw(tf::Pose& t)
{
    double yaw, pitch, roll;
    t.getBasis().getEulerYPR(yaw, pitch, roll);
    return yaw;
}

void
AmclNode::initialPoseReceivedOld(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    // Support old behavior, where null frame ids were accepted.
    if(msg->header.frame_id == "")
    {
        ROS_WARN("Received initialpose message with header.frame_id == "".  This behavior is deprecated; you should always set the frame_id");
        initialPoseReceived(msg);
    }
}

void
AmclNode::initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    boost::recursive_mutex::scoped_lock prl(configuration_mutex_);
    // In case the client sent us a pose estimate in the past, integrate the
    // intervening odometric change.
    tf::StampedTransform tx_odom;
    try
    {
        tf_->lookupTransform(base_frame_id_, ros::Time::now(),
                             base_frame_id_, msg->header.stamp,
                             global_frame_id_, tx_odom);
    }
    catch(tf::TransformException e)
    {
        // If we've never sent a transform, then this is normal, because the
        // global_frame_id_ frame doesn't exist.  We only care about in-time
        // transformation for on-the-move pose-setting, so ignoring this
        // startup condition doesn't really cost us anything.
        if(sent_first_transform_)
            ROS_WARN("Failed to transform initial pose in time (%s)", e.what());
        tx_odom.setIdentity();
    }

    tf::Pose pose_old, pose_new;
    tf::poseMsgToTF(msg->pose.pose, pose_old);
    pose_new = tx_odom.inverse() * pose_old;

    ROS_INFO("Setting pose (%.6f): %.3f %.3f %.3f",
             ros::Time::now().toSec(),
             pose_new.getOrigin().x(),
             pose_new.getOrigin().y(),
             getYaw(pose_new));
    // Re-initialize the filter
    pf_vector_t pf_init_pose_mean = pf_vector_zero();
    pf_init_pose_mean.v[0] = pose_new.getOrigin().x();
    pf_init_pose_mean.v[1] = pose_new.getOrigin().y();
    pf_init_pose_mean.v[2] = getYaw(pose_new);
    pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
    // Copy in the covariance, converting from 6-D to 3-D
    for(int i=0; i<2; i++)
    {
        for(int j=0; j<2; j++)
        {
            pf_init_pose_cov.m[i][j] = msg->pose.covariance[6*i+j];
        }
    }
    pf_init_pose_cov.m[2][2] = msg->pose.covariance[6*5+5];

    delete initial_pose_hyp_;
    initial_pose_hyp_ = new amcl_hyp_t();
    initial_pose_hyp_->pf_pose_mean = pf_init_pose_mean;
    initial_pose_hyp_->pf_pose_cov = pf_init_pose_cov;
    applyInitialPose();
}

/**
 * If initial_pose_hyp_ and map_ are both non-null, apply the initial
 * pose to the particle filter state.  initial_pose_hyp_ is deleted
 * and set to NULL after it is used.
 */
void
AmclNode::applyInitialPose()
{
    boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
    if( initial_pose_hyp_ != NULL && map_ != NULL ) {
        pf_init(pf_, initial_pose_hyp_->pf_pose_mean, initial_pose_hyp_->pf_pose_cov,map_);
        pf_init_ = false;

        delete initial_pose_hyp_;
        initial_pose_hyp_ = NULL;
    }
}

#if COLLECT_DATA

/**
 * Data collection method
 */
void
AmclNode::log_data(geometry_msgs::PoseWithCovarianceStamped pose_bestEstimate){
    // Get elapsed time
    struct timeval tp;
    gettimeofday(&tp, NULL);
    long int current_timestamp = tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds
    double elapsed_time = (current_timestamp - start_timestamp) * 1.0e-3; //in Sec

    double difference_in_true_and_estimate_normal = sqrt( (pose_bestEstimate.pose.pose.position.x- true_pose_normal.v[0])
            *(pose_bestEstimate.pose.pose.position.x- true_pose_normal.v[0])
            +
            (pose_bestEstimate.pose.pose.position.y- true_pose_normal.v[1])
            *(pose_bestEstimate.pose.pose.position.y- true_pose_normal.v[1])
            );

    std::stringstream data_stream;
    std_msgs::String data_msg;

    std_msgs::String filename_msg;
    std_msgs::String headers_msg;

    filename_msg.data = filename_abs;
    headers_msg.data = headers.str();

    data_fname_pub_.publish(filename_msg);
    data_headers_pub_.publish(headers_msg);

    data_stream << algo_name << "\t"
                   //<< robot_start_config_id << "\t"
                   //<< trajectory_id << "\t"
                << run_number << "\t"
                << start_timestamp << "\t"
                   //<< max_particles_ << "\t"
                   //<< max_nested_particles_ << "\t"
                << elapsed_time << "\t"
                << pf_->sets[pf_->current_set].sample_count << "\t"

                   //<< "[" << true_pose_normal.v[0] << " : " << true_pose_normal.v[1] << " : " << true_pose_normal.v[2] << "]\t"
                   //<< "[" << pose_bestEstimate.pose.pose.position.x << " : " << pose_bestEstimate.pose.pose.position.y << " : " << "0.0" << "]\t"
                   //<< difference_in_true_and_estimate_normal << "\t"
                   //<< normal_MSE << "\t"
                   //<< normal_RootMSE << "\t"
                   //<< normal_particles_within_1m << "\t"

                   //<< occlusion_proportion << "\t"

                   //<< "[" << true_pose_nested.v[0] << " : " << true_pose_nested.v[1] << " : " << true_pose_nested.v[2] << "]\t"
                << nested_MSE << "\t"
                << nested_RootMSE << "\t"
                << nested_particles_within_1m << "\t"

                   //<< pf_->sets[pf_->current_set].avg_weight << "\t"
                   //<< pf_->sets[pf_->current_set].cov.m[0][0] << "\t"
                   //<< pf_->sets[pf_->current_set].cov.m[1][1] << "\t"
                   //<< pf_->sets[pf_->current_set].cov.m[2][2] << "\t"
                   //<< pf_->sets[pf_->current_set].cov.m[0][0] + pf_->sets[pf_->current_set].cov.m[1][1] + pf_->sets[pf_->current_set].cov.m[2][2] << "\t"
                << (isLandmarkObserved ? 1 : 0 ) << "\t"
                << other_robot_distance << "\t"
                << nested_particle_count << "\t"
                << total_nested_particle_count;


    data_msg.data = data_stream.str();

    results_out << data_stream.str() << std::endl;
    results_out.flush();

    ROS_DEBUG("nested_amcl sent: ## %s ##", data_msg.data.c_str());
    npf_data_pub_.publish(data_msg);
}

#endif
