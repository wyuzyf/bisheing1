
/*mrobot ����turtlebot*/
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>  //ʵ��Ϊһ��ROS����̬�ļ��ػ�ж�ز��
                                          //��ĺ궨���б�
#include <nodelet/nodelet.h>  //ʵ����ͬһ��������ͬʱ���ж����㷨
                              //���㷨֮��0����
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Images.h>
#include <visualization_msgs/Marker.h>
#include <turtlebot_msgs/SetFollowState.h>

#include "dynamic_reconfigure/server.h"
#include "turtlebot_follower/FollowerConfig.h"

#include <depth_image_proc/depth_traits.h>

namespace turtlebot_follwer{
	
	//����3d������������Ϣ�������ٶ���Ϣ
	class TurtlebotFollower : public nodelet::Nodelet
	{
	public:
		//���캯��
		TurtlebotFollower() : min_y_(0.1), max_y_(0.5),
			min_x_(-0.2), max_x_(0.2),
			max_z_(0.8), goal_z_(0.6),
			z_scale_(1.0), x_scale_(5.0)
		{

		}
		//��������
		~TurtlebotFollower()
		{
			delete config_srv_;   //?????
		}

	private:
		double min_y_;  //��������С��y����
		double max_y_; // The maximum y position of the points in the box.
		double min_x_; // The minimum x position of the points in the box. 
		double max_x_; // The maximum x position of the points in the box. 
		double max_z_; // The maximum z position of the points in the box. 
		double goal_z_;   //Ŀ�굽�����˵ľ���
		double z_scale_;  //�ƶ�ʱ�������ٶȵ�����ϵ��
		double x_scale_;   //��תʱ�������ٶȵ�����ϵ��
		bool enable_;  //�Ƿ�ʹ�ܸ���

		//��ʼ�������
		ros::ServiceServer switch_srv_;

		// Dynamic reconfigure server
		dynamic_reconfigure::Server<turtlebot_follower::FollowerConfig>* config_srv_;

		//��ʼ��handle,�����ͻ���
		virtual void onInit()
		{
			ros::NodeHandle& nh = getNodeHandle();
			ros::NodeHandle& private_nh = getPrivateNodeHandle();

			//�Ӳ�����������ȡ������launch�ļ���������ֵ��
			private_nh.getParam("min_y", min_y_);
			private_nh.getParam("max_y", max_y_);
			private_nh.getParam("min_x", min_x_);
			private_nh.getParam("max_x", max_x_);
			private_nh.getParam("max_z", max_z_);
			private_nh.getParam("goal_z", goal_z_);
			private_nh.getParam("z_scale", z_scale_);
			private_nh.getParam("x_scale", x_scale_);
			private_nh.getParam("enabled", enabled_);

			//���û������ƶ��Ļ��⣨���ڻ������ƶ�����/mobile_base/mobile_base_controller/cmd_vel��������Ļ����˵��ƶ�topic��
			cmdpub_ = private_nh.advertise<geometry_msgs::Twist>("cmd_vel",1);

			markerpub_ = private_nh.advertise<visualization_msgs::Marker>("marker", 1);
			bboxpub_ = private_nh.advertise<visualization_msgs::Marker>("bbox", 1);
			sub_ = nh.subscribe<sensor_msgs::Image>("depth/image_rect", 1, &TurtlebotFollower::imagecb, this);

			switch_srv_ = private_nh.advertiseService("change_state", &TurtlebotFollower::changeModeSrvCb, this);

			config_srv_ = new dynamic_reconfigure::Server<turtlebot_follower::FollowerConfig>(private_nh);
			dynamic_reconfigure::Server<turtlebot_follower::FollowerConfig>::CallbackType f =
				boost::bind(&TurtlebotFollower::reconfigure, this, _1, _2);
			config_srv_->setCallback(f);
		}

		//����Ĭ��ֵ�����catkin_ws/devel/include/turtlrbot_follower/cfg
		void reconfigure(turtlebot_follower::FollowerConfig &config, uint32_t level)
		{
			min_y_ = config.min_y;
			max_y_ = config.max_y;
			min_x_ = config.min_x;
			max_x_ = config.max_x;
			max_z_ = config.max_z;
			goal_z_ = config.goal_z;
			z_scale_ = config.z_scale;
			x_scale_ = config.x_scale;
		}

		/*!
		* @brief Callback for point clouds.
		* Callback for depth images. It finds the centroid
		* of the points in a box in the center of the image.
		* ���������Ϣ���ҵ�ͼ�����Ŀ��еĵ������
		* Publishes cmd_vel messages with the goal from the image.
		* ����ͼ����Ŀ���cmd_vel ��Ϣ
		* @param cloud The point cloud message.
		* ���������Ƶ���Ϣ
		*/
		void imageCb(const sensor_msgs::ImageConstPtr& depth_msg)
		{
			// Precompute the sin function for each row and column
			uint32_t image_width = depth_msg->width;
			ROS_INFO_THROTTLE(1, "image_width=%d", image_width);
			float x_radians_per_pixel = 60.0 / 57.0 / image_width;  //ÿ�����صĻ���
			float sin_pixel_x[image_width];
			for (int x = 0; x < image_width; ++x)
			{
				//����ֵ
				sin_pixel_x[x] = sin((x - image_width / 2.0)  * x_radians_per_pixel);
			}

			uint32_t image_height = depth_msg->height;
			ROS_INFO_THROTTLE(1, "image_height=%d", image_height);
			float y_radians_per_pixel = 45.0 / 57.0 / image_width;
			float sin_pixel_y[image_height];
			for (int y = 0; y < image_height; ++y)
			{
				// Sign opposite x for y up values
				sin_pixel_y[y] = sin((image_height / 2.0 - y)  * y_radians_per_pixel);
			}

			//X,Y,Z of the centroid ���ĵ�����
			float x = 0.0;
			float y = 0.0;
			float z = 1e6;
			//Number of points observed �۲�ĵ���
			unsigned int n = 0;

			// ����ͨ������������е㣬�ҵ�λ�õ�ƽ��ֵ
			const float* depth_row = reinterpret_cast<const float*>(&depth_msg->data[0]);
			int row_step = depth_msg->step / sizeof(float);
			for (int v = 0; v < (int)depth_msg->height; ++v, depth_row += row_step)
			{
				for (int u = 0; u < (int)depth_msg->width; ++u)
				{
					float depth = depth_image_proc::DepthTraits<float>::toMeters(depth_row[u]);
					if (!depth_image_proc::DepthTraits<float>::valid(depth) || depth > max_z_) continue;
					float y_val = sin_pixel_y[v] * depth;
					float x_val = sin_pixel_x[u] * depth;
					if (y_val > min_y_ && y_val < max_y_ &&
						x_val > min_x_ && x_val < max_x_)
					{
						x += x_val;
						y += y_val;
						z = std::min(z, depth); //approximate depth as forward.
						n++;
					}
				}
			}
			//If there are points, find the centroid and calculate the command goal.
			//If there are no points, simply publish a stop goal.
			//����е㣬�ҵ����Ĳ���������Ŀ�ꡣ���û�е㣬ֻ�跢��ֹͣ��Ϣ��
			if (n > 4000)
			{
				x /= n;
				y /= n;
				if (z > max_z_)
				{
					ROS_INFO_THROTTLE(1, "Centroid too far away %f, stopping the robot\n", z);
					if (enabled_)
					{
						cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
					}
					return;
				}

				ROS_INFO_THROTTLE(1, "Centroid at %f %f %f with %d points", x, y, z, n);
				publishMarker(x, y, z);

				if (enabled_)
				{
					geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
					cmd->linear.x = (z - goal_z_) * z_scale_;
					cmd->angular.z = -x * x_scale_;
					cmdpub_.publish(cmd);
				}
			}

			else
			{
				ROS_INFO_THROTTLE(1, "Not enough points(%d) detected, stopping the robot", n);
				publishMarker(x, y, z);

				if (enabled_)
				{
					cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
				}
			}

			publishBbox(); //��һ�������壬����������������Ȥ����RIO��
		}

		//����״̬
		bool changeModeSrvCb(urtlebot_msgs::SetFollowState::Request& request,
			turtlebot_msgs::SetFollowState::Response& response)
		{
			if ((enabled_ == true) && (request.state == request.STOPPED))
			{
				ROS_INFO("Change mode service request: following stopped");
				cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
				enabled_ = false;
			}
			else if ((enabled_ == false) && (request.state == request.FOLLOW))
			{
				ROS_INFO("Change mode service request: following (re)started");
				enabled_ = true;
			}

			response.result = response.OK;
			return true;
		}

		//��һ��Բ�㣬���Բ���������
		void publishMarker(double x, double y, double z)
		{
			visualization_msgs::Marker marker;
			marker.header.frame_id = "/camera_rgb_optical_frame";
			marker.header.stamp = ros::Time();
			marker.ns = "my_namespace";
			marker.id = 0;
			marker.type = visualization_msgs::Marker::SPHERE;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = x;
			marker.pose.position.y = y;
			marker.pose.position.z = z;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;
			marker.scale.x = 0.2;
			marker.scale.y = 0.2;
			marker.scale.z = 0.2;
			marker.color.a = 1.0;
			marker.color.r = 1.0;
			marker.color.g = 0.0;
			marker.color.b = 0.0;
			//only if using a MESH_RESOURCE marker type:
			markerpub_.publish(marker);
		}
		
		//��һ�������壬����������������Ȥ����RIO��
		void publishBbox()
		{
			double x = (min_x_ + max_x_) / 2;
			double y = (min_y_ + max_y_) / 2;
			double z = (0 + max_z_) / 2;

			double scale_x = (max_x_ - x) * 2;
			double scale_y = (max_y_ - y) * 2;
			double scale_z = (max_z_ - z) * 2;

			visualization_msgs::Marker marker;
			marker.header.frame_id = "/camera_rgb_optical_frame";
			marker.header.stamp = ros::Time();
			marker.ns = "my_namespace";
			marker.id = 1;
			marker.type = visualization_msgs::Marker::CUBE;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = x;
			marker.pose.position.y = -y;
			marker.pose.position.z = z;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;
			marker.scale.x = scale_x;
			marker.scale.y = scale_y;
			marker.scale.z = scale_z;
			marker.color.a = 0.5;
			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;
			//only if using a MESH_RESOURCE marker type:
			bboxpub_.publish(marker);
		}
		
		ros::Subscriber sub_;
		ros::Publisher cmdpub_;
		ros::Publisher markerpub_;
		ros::Publisher bboxpub_;
	};

	PLUGINLIB_DECLARE_CLASS(turtlebot_follower, TurtlebotFollower, turtlebot_follower::TurtlebotFollower, nodelet::Nodelet);

}


