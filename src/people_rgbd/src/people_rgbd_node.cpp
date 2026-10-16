#include <rclcpp/rclcpp.hpp>
#include <people_msgs/msg/people.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <math.h>
#include <cstdio>
#include <iostream>

using namespace std;

/**
 * @brief Synchronization policy for receiving RGB and depth images
 */
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> syncPolicy;

/**
 * @class RgbdPeopleDetector
 * @brief Detect people in RGBD images
 */
class RgbdPeopleDetector : public rclcpp::Node
{
public:
	// Camera topic prefix, must be declared/initialized before the
	// image_transport subscriber filters below since they need it in
	// their own member-initializer-list (C++ initializes members in
	// declaration order, not initializer-list order).
	std::string cameraTopic_;

	// Ros stuff
	image_transport::SubscriberFilter imageSub_;    	/**< Image subscriber*/
	image_transport::SubscriberFilter depthSub_;    	/**< Depth subscriber*/
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cInfoSub_;      /**< Camera info subscriber*/
	message_filters::Synchronizer<syncPolicy> imageSync_;   /**< Time synchronizer filter*/
	rclcpp::Publisher<people_msgs::msg::People>::SharedPtr peoplePub_;			/**< People publisher*/
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;	/**< Markers publisher*/

	// Node parameters
	bool calibInit_, debug_img_;						/**< Node calibration and debug image flags*/
	float fx, fy, cx, cy;                 				/**< Camera intrinsic parameters matrix*/
	std_msgs::msg::Header imgHeader_, depthHeader_;     /**< Image header for publishing*/
	std::string imgEncodings_;                      	/**< Image encodings for publishing*/
	cv::Size imgSize_;                              	/**< Image size*/
	double rate_;										/**< Detection rate*/
	vector<cv::Rect> people_rect_;						/**< Rectangle of people detected into current image*/
	vector<geometry_msgs::msg::PointStamped> people_pos_;	/**< Position of people detected into current image*/
	std::string frame_id_;								/**< Detection frame_id*/
	cv::Mat rgbImg_, depthImg_;							/**< RGB and Depth images*/
	int seq;											/**< Sequence number for detections */

	// Node objects
	cv::HOGDescriptor hog_;								/**< People detector class*/
	std::shared_ptr<tf2_ros::Buffer> tfBuffer_;			/**< Transform buffer*/
	std::shared_ptr<tf2_ros::TransformListener> tfListener_;	/**< Transform listener*/

	/**
	 * \brief Constructor
	 */
	RgbdPeopleDetector():
	Node("people_rgbd_node"),
	cameraTopic_(this->declare_parameter<std::string>("camera_topic", "camera")),
	imageSub_(this, cameraTopic_ + "/rgb/image_raw", "raw"),
	depthSub_(this, cameraTopic_ + "/depth_registered/image_raw", "raw"),
	imageSync_(syncPolicy(10), imageSub_, depthSub_),
	hog_(cv::Size(48, 96), cv::Size(16, 16), cv::Size(8, 8), cv::Size(8, 8), 9)
	{
		// Topic subscription
		cInfoSub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
			cameraTopic_ + "/rgb/camera_info", 1,
			std::bind(&RgbdPeopleDetector::cInfoCallback, this, std::placeholders::_1));
		imageSync_.registerCallback(std::bind(&RgbdPeopleDetector::syncImageCallback, this, std::placeholders::_1, std::placeholders::_2));

		// Read ros parameters
		rate_ = this->declare_parameter<double>("rate", 5.0);
		frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
		debug_img_ = this->declare_parameter<bool>("debug_image", false);

		// Launch publishers
		peoplePub_ = this->create_publisher<people_msgs::msg::People>("/people_detections", 10);
		markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/people_detections/markers", 1);

		// Transform listener
		tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
		tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

		// Initialize internar variables
		seq = 0;
		calibInit_ = false;
		hog_.setSVMDetector(cv::HOGDescriptor::getDaimlerPeopleDetector());
	}

	/**
	 * \brief Destructor
	 */
	~RgbdPeopleDetector()
	{
	}

	/**
     * @brief Camera calibration info callback
     * @param cInfoMsg Camera calibration message
     */
    void cInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr cInfoMsg)
    {
        if(!calibInit_)
        {
            // Set calibration flag
            calibInit_ = true;

            // Store RGB camera parameters
            fx = cInfoMsg->k[0];
			fy = cInfoMsg->k[4];
			cx = cInfoMsg->k[2];
			cy = cInfoMsg->k[5];
        }
    }

	/**
	 * @brief Synchronized RGB and Depth images callback
	 * @param imgMsg RGB image message
	 * @param depthMsg Depth image message
	 */
    void syncImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr imgMsg, const sensor_msgs::msg::Image::ConstSharedPtr depthMsg)
    {
        // Processing does not start until first calibration is received
        if(!calibInit_)
        {
            RCLCPP_WARN(this->get_logger(), "No camera calibration received yet, skipping image processing");
            return;
        }

        // Convert to OpenCV format
        if(!convertImages(imgMsg, depthMsg))
            return;

		// Launch detection
		people_rect_.clear();
		hog_.detectMultiScale(rgbImg_, people_rect_, 0.5, cv::Size(8,8), cv::Size(32,32), 1.05, 2, true);
		adjustAndFilterDetections();

        // Get peopleo position
        computePositionFromRect();

        // Publish detections
        publishPeopleDetection();

        // Publish people markers
		publishPeopleMarkers();

        // Debug
        if(debug_img_)
        {
			for (size_t i=0; i<people_rect_.size(); i++)
				cv::rectangle(rgbImg_, people_rect_[i].tl(), people_rect_[i].br(), cv::Scalar(0, 255, 0), 2);
			cv::imshow("People detector", rgbImg_);
			cv::waitKey(5);
		}
	}

	/**
     * @brief Convert images from ROS message to OpenCV format (cv::Mat)
     * @param rgbMsg RGB image in ROS format
     * @param depthMsg Depth image in ROS format
     * @return
     */
    bool convertImages(const sensor_msgs::msg::Image::ConstSharedPtr& rgbMsg,
                       const sensor_msgs::msg::Image::ConstSharedPtr& depthMsg)
    {
        // Convert to OpenCV format without copy
        cv_bridge::CvImageConstPtr cvbRgb, cvbDepth;
        try
        {
            cvbRgb = cv_bridge::toCvCopy(rgbMsg, sensor_msgs::image_encodings::BGR8);
            cvbDepth = cv_bridge::toCvCopy(depthMsg);
        }
        catch(cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return false;
        }

        // Store header and encoding
        imgHeader_ = cvbRgb->header;
        depthHeader_ = depthMsg->header;
        imgEncodings_ = cvbRgb->encoding;

        cvbRgb->image.copyTo(rgbImg_);
        cvbDepth->image.copyTo(depthImg_);

        // Store image size
        imgSize_.width = rgbImg_.cols;
        imgSize_.height = rgbImg_.rows;

        return true;
    }

    /**
     * @brief Slightly shrink the rectangles to get a nicer detector
     */
    void adjustAndFilterDetections(void)
    {
        // The HOG detector returns slightly larger rectangles than the real objects,
        // so we slightly shrink the rectangles to get a nicer output.
        for(size_t i=0; i<people_rect_.size(); i++)
        {
			people_rect_[i].x += cvRound(people_rect_[i].width*0.2);
			people_rect_[i].width = cvRound(people_rect_[i].width*0.6);
			people_rect_[i].y += cvRound(people_rect_[i].height*0.07);
			people_rect_[i].height = cvRound(people_rect_[i].height*0.8);
		}

		// Filter out detections inside other detections
		std::vector<cv::Rect> filter;
		for(size_t i=0; i<people_rect_.size(); i++)
        {
			bool valid = true;
			for(size_t j=0; j<people_rect_.size(); j++)
			{
				if(i==j)
					continue;
				if(1.2*people_rect_[i].tl().x >= people_rect_[j].tl().x &&
				   1.2*people_rect_[i].tl().y >= people_rect_[j].tl().y &&
				   0.8*people_rect_[i].br().x <= people_rect_[j].br().x &&
				   0.8*people_rect_[i].br().y <= people_rect_[j].br().y)
					valid = false;
			}

			if(valid && people_rect_[i].height > 2*people_rect_[i].width)
				filter.push_back(people_rect_[i]);
		}
		people_rect_ = filter;
    }

    void computePositionFromRect(void)
    {
		people_pos_.clear();
		for (size_t i=0; i<people_rect_.size(); i++)
		{
			cv::Rect &r = people_rect_[i];

			// Compute averaged depth
			int N = 0;
			float u, v, depth = 0;
			for(int i=r.tl().y; i<r.br().y; i++)
				for(int j=r.tl().x; j<r.br().x; j++)
					if(depthImg_.at<float>(i,j) > 0.3 && depthImg_.at<float>(i,j) < 8.0)
					{
						depth += depthImg_.at<float>(i,j);
						N++;
					}
			if(N == 0)
				continue;
			depth = depth/N;
			u = (r.br().x - r.tl().x)/2.0;
			v = (r.br().y - r.tl().y)/2.0;

			// Compute position in camera frame
			geometry_msgs::msg::PointStamped camPt;
			camPt.header = depthHeader_;
			camPt.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
			camPt.point.z = depth;
			camPt.point.x = (float)(u-cx) * depth / fx;
			camPt.point.y = (float)(v-cy) * depth / fx;

			// Convert point in camera frame to this node frame_id
			try
			{
				geometry_msgs::msg::PointStamped pt = tfBuffer_->transform(camPt, frame_id_);
				people_pos_.push_back(pt);
			}
			catch(const tf2::TransformException& ex)
			{
				std::cout << "TF exception: " << ex.what() << std::endl;
			}
		}
	}

	void publishPeopleDetection(void)
	{
		char buf[100];
		people_msgs::msg::People people;

		people.header.frame_id = frame_id_;
		people.header.stamp = this->now();
		for(int i=0; i<(int)people_pos_.size(); i++)
		{
			people_msgs::msg::Person person;
			person.name = "";
			person.position.x = people_pos_[i].point.x;
			person.position.y = people_pos_[i].point.y;
			person.position.z = 0;
			person.velocity.x = 0;
			person.velocity.y = 0;
			person.velocity.z = 0;
			person.reliability = 0.8;
			person.tagnames.push_back("RGBD");
			sprintf(buf, "%d", seq++);
			person.tags.push_back(buf);
			people.people.push_back(person);
		}

		if(people.people.size() > 0)
			peoplePub_->publish(people);
	}

	void publishPeopleMarkers(void)
	{
		// The teresa robot is compossed by 4 markers
		visualization_msgs::msg::MarkerArray people;
		visualization_msgs::msg::Marker marker;

		// Get marker color
		float r, g, b;
		r = 0.0;
		g = 1.0;
		b = 0.0;

		marker.header.frame_id = frame_id_;
		marker.header.stamp = this->now();
		marker.ns = this->get_name();
		for(int i=0; i<(int)people_pos_.size(); i++)
		{
			// Body
			marker.id = i;
			marker.type = visualization_msgs::msg::Marker::CYLINDER;
			marker.action = visualization_msgs::msg::Marker::ADD;
			marker.pose.position.x = people_pos_[i].point.x;
			marker.pose.position.y = people_pos_[i].point.y;
			marker.pose.position.z = 0.75;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 0.0;
			marker.scale.x = 0.4;
			marker.scale.y = 0.4;
			marker.scale.z = 1.5;
			marker.color.a = 1.0;
			marker.color.r = r;
			marker.color.g = g;
			marker.color.b = b;
			people.markers.push_back(marker);

			// Head
			marker.id = i+50;
			marker.type = visualization_msgs::msg::Marker::SPHERE;
			marker.action = visualization_msgs::msg::Marker::ADD;
			marker.pose.position.x = people_pos_[i].point.x;
			marker.pose.position.y = people_pos_[i].point.y;
			marker.pose.position.z = 1.5+0.2;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;
			marker.scale.x = 0.4;
			marker.scale.y = 0.4;
			marker.scale.z = 0.4;
			marker.color.a = 1.0;
			marker.color.r = r;
			marker.color.g = g;
			marker.color.b = b;
			people.markers.push_back(marker);
		}
		for(size_t i=people_pos_.size(); i<20; i++)
		{
			// Delete body marker
			marker.id = i;
			marker.type = visualization_msgs::msg::Marker::CYLINDER;
			marker.action = visualization_msgs::msg::Marker::DELETE;
			people.markers.push_back(marker);

			// Delete head marker
			marker.id = i+50;
			marker.type = visualization_msgs::msg::Marker::SPHERE;
			marker.action = visualization_msgs::msg::Marker::DELETE;
			people.markers.push_back(marker);
		}

		// publish marker:
		markers_pub_->publish(people);
	}
};


int main (int argc, char** argv)
{
	rclcpp::init(argc, argv);

	// RGBD People detector instance
	auto node = std::make_shared<RgbdPeopleDetector>();

	// Spin for ever
	rclcpp::spin(node);

	rclcpp::shutdown();
	return 0;
}
