//standards
#include <iostream>
#include <float.h>
//People detector
#include <PeopleDetector.h>
#include <people_msgs/msg/people.hpp>
#include "person_kf.hpp"

//ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std;

// A geometry point plus a scratch flag used to track processing state
// (ROS2's std_msgs/Header no longer carries a "seq" field, which the
// original ROS1 code reused for this purpose).
struct FlaggedPoint
{
	geometry_msgs::msg::PointStamped pt;
	int flag;
};

class PeopleDetectorNode : public rclcpp::Node
{
public:
	//Subscribers/Publishers
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
	rclcpp::Publisher<people_msgs::msg::People>::SharedPtr peoplePub_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
	int pose_seq_;
	//Parameters
	string input_scan_topic_;
	string hypotheses_filename_;
	int num_hypotheses_;
	double threshold_;
	FILE *f_hypotheses;
	int list_size_, min_beams_;
	double legs_distance_, track_distance_, max_distance_, track_timeout_, confidence_;
	string detection_frame_id_;
	//People Detector object
	PeopleDetector pd;
	// People tracking
	vector<PersonKF> persons;
	int personCount;
	// Laser transform
	std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
	std::shared_ptr<tf2_ros::TransformListener> tfListener_;


  /**
   * \brief Constructor
   */
	PeopleDetectorNode() : Node("tracker_node")
	{
		threshold_ = this->declare_parameter<double>("threshold", 0.15);
		hypotheses_filename_ = this->declare_parameter<std::string>("hypotheses_filename", "");
		num_hypotheses_ = this->declare_parameter<int>("num_hypotheses", 100);
		list_size_ = this->declare_parameter<int>("list_size", 500);
		if (hypotheses_filename_ == "")
		{
		  RCLCPP_ERROR(this->get_logger(), "Provide hypotheses file!!");
		  exit(2);
		}
		legs_distance_ = this->declare_parameter<double>("legs_distance", 0.75);
		track_distance_ = this->declare_parameter<double>("track_distance", 0.75);
		min_beams_ = this->declare_parameter<int>("min_beams", 3);
		max_distance_ = this->declare_parameter<double>("max_distance", 5.0);
		track_timeout_ = this->declare_parameter<double>("track_timeout", 2.0);
		confidence_ = this->declare_parameter<double>("confidence", 0.5);
		detection_frame_id_ = this->declare_parameter<std::string>("frame_id", "odom");

		tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
		tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

		scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
			"/scan", 1, std::bind(&PeopleDetectorNode::scan_cb, this, std::placeholders::_1));
		peoplePub_ = this->create_publisher<people_msgs::msg::People>("/people_detections", 10);
		markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/people_detections/markers", 1);

		personCount = 0;
		pose_seq_ = 0;
	}

  /**
   * \brief Destructor
   */
	~PeopleDetectorNode()
	{
	}

  /**
   * \brief read out file with trained hypotheses
   */
	void init ()
	{
		try
		{
			f_hypotheses = fopen(hypotheses_filename_.c_str(), "r");
			if ( f_hypotheses == NULL )
			{
				RCLCPP_ERROR(this->get_logger(), "ERROR opening hypotesis file %s", hypotheses_filename_.c_str());
				throw -1;
			}
		}
		catch (int e)
		{
		}

		// configure the people detector object
		pd.load(f_hypotheses, num_hypotheses_);

		// Close file
		fclose(f_hypotheses);
	}

  /**
   * \brief getting range messages from your system and classify them
   * \param msg input laser scan
   */
	void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
	{
		// This is a list of segments.
		// For more information have a look at ../common/dynamictable.h/hxx
		dyntab_segments *list_segments=NULL;
		int num_readings = msg->ranges.size();
		double *angles = new double[num_readings]; // array of angles
		double *ranges = new double[num_readings]; // array of measurements

		for (int i = 0; i < num_readings; i++)
		{
			ranges[i] = msg->ranges[i];
			angles[i] = msg->angle_min + i * msg->angle_increment;
		}
		list_segments = new dyntab_segments (list_size_);
		// segment the scan
		pd.segmentScan(threshold_, num_readings, angles, ranges, list_segments);

		// Classiy segments
		for (int i=0; i < list_segments->num(); i++)
		{
			Segment *s = list_segments->getElement(i);

			// discard segments with less than three points
			if ( s->num() < min_beams_ )
				s->type = -1;
			else
				pd.classify(s);
		}

		// Send out classification results and compute the averaged leg points
		vector<FlaggedPoint> legPoints;
		for (int i=0; i < list_segments->num(); i++)
		{
			// Get segment information
			Segment *s = list_segments->getElement(i);

			// Only process segment if it is a person
			if (s->type != 1)
				continue;

			// Extract mean range and angle from segment
			double r = (s->getElement(0)->range+s->getElement(s->num()-1)->range)/2.0;
			double a = (s->getElement(0)->angle+s->getElement(s->num()-1)->angle)/2.0;

			// Do not consider the leg if it is farther than max_distance
			if(r > max_distance_)
			  continue;

			// Create a 2D point in the laser frame
			geometry_msgs::msg::PointStamped laserPoint;
			laserPoint.header = msg->header;
			laserPoint.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
			laserPoint.point.x = r*cos(a);
			laserPoint.point.y = r*sin(a);
			laserPoint.point.z = 0.0;

			// Get the leg position into the map frame
			try
			{
				FlaggedPoint robotPoint;
				robotPoint.pt = tfBuffer_->transform(laserPoint, detection_frame_id_);
				robotPoint.flag = 0;
				legPoints.push_back(robotPoint);
			}
			catch(const tf2::TransformException& ex)
			{
				std::cout << "TF exception: " << ex.what() << std::endl;
			}
		}

		// Delete the list of segments
		list_segments->setAutoDelete(true);
		delete list_segments;
		list_segments = NULL;
		delete [] ranges;
		delete [] angles;

		// Process legs to search persons (pair of legs)
		double x1, y1, x2, y2, z, minDist, dist;
		vector<FlaggedPoint> personPoints;
		for(int i = 0; i < (int)legPoints.size(); i++)
		{
			// Do not consider legs already processed (flag = -1)
			if(legPoints[i].flag == -1)
				continue;

			// Mark this leg as already processed
			legPoints[i].flag = -1;

			// Get the closest leg
			int k = 0;
			x1 = legPoints[i].pt.point.x;
			y1 = legPoints[i].pt.point.y;
			z =  legPoints[i].pt.point.z;
			minDist = 1000000.0;
			for(int j = i+1; j < (int)legPoints.size(); j++)
			{
				if(legPoints[j].flag != -1)
				{
					x2 = legPoints[j].pt.point.x;
					y2 = legPoints[j].pt.point.y;
					dist = sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
					if(dist < minDist)
					{
						k = j;
						minDist = dist;
					}
				}
			}

			// Add a person in the middle of both legs or the single leg
			if(minDist < legs_distance_) {
				FlaggedPoint pPoint;
				pPoint.pt.header = legPoints[i].pt.header;
				pPoint.pt.point.x = (legPoints[i].pt.point.x+legPoints[k].pt.point.x)/2.0;
				pPoint.pt.point.y = (legPoints[i].pt.point.y+legPoints[k].pt.point.y)/2.0;
				pPoint.pt.point.z = (legPoints[i].pt.point.z+legPoints[k].pt.point.z)/2.0;
				pPoint.flag = legPoints[i].flag;
				legPoints[k].flag = -1;	// Do not considers this leg anymore
				personPoints.push_back(pPoint);
			}
			else {
				personPoints.push_back(legPoints[i]);
				personPoints.back().flag = -2;// Mark this person as "one-leg", not suitble for initialization
			}

		}

		// Update the position of the persons being tracked
		for(int i=0; i<(int)persons.size(); i++)
		{
			// Deactivate update flag
			persons[i].updated = false;

			// Search the closest point to every person tracked
			int k = 0;
			x1 = persons[i].x(0,0);
			y1 = persons[i].x(1,0);
			minDist = 1000000.0;
			for(int j=0; j<(int)personPoints.size(); j++)
			{
				if(personPoints[j].flag == -1 || personPoints[j].flag == -2)
				{
					x2 = personPoints[j].pt.point.x;
					y2 = personPoints[j].pt.point.y;
					dist = sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
					if(dist < minDist)
					{
						k = j;
						minDist = dist;
					}
				}
			}

			// Check if the distant is too high
			if(minDist < track_distance_)
			{
				// Predict the position of the person to this instant
				persons[i].predict((rclcpp::Time(msg->header.stamp)-persons[i].tStamp).seconds());

				// Update the filter
				persons[i].update(personPoints[k].pt.point.x, personPoints[k].pt.point.y, rclcpp::Time(msg->header.stamp));

				// Mark the personPoint as used
				personPoints[k].flag = 1;
			}
		}

		// Add new persons to the list
		for(int i=0; i<(int)personPoints.size(); i++)
		{
			if(personPoints[i].flag == -1) // Only add new persons with two legs detected
			{
				PersonKF person;
				person.init(personCount++, personPoints[i].pt.point.x, personPoints[i].pt.point.y, 0, 0, rclcpp::Time(msg->header.stamp));
				persons.push_back(person);

				personPoints[i].flag = 1;
			}
		}

		// Remove too old estimations without update
		vector<PersonKF> temp;
		for(int i=0; i<(int)persons.size(); i++)
		{
			if((this->now()-persons[i].tStamp).seconds() < track_timeout_)
				temp.push_back(persons[i]);
		}
		persons.clear();
		persons = temp;

		/* Publish estimated positions */
		publishPeopleDetection();

		/* Publish people markers */
		publishPeopleMarkers();
	}

	void publishPeopleDetection(void)
	{
		char buf[100];
		people_msgs::msg::People people;

		people.header.frame_id = detection_frame_id_;
		people.header.stamp = this->now();
		for(int i=0; i<(int)persons.size(); i++)
		{
			if(!persons[i].updated)
				continue;
			people_msgs::msg::Person person;
			person.name = "";
			person.position.x = persons[i].x(0,0);
			person.position.y = persons[i].x(1,0);
			person.position.z = 0;
			person.velocity.x = persons[i].x(2,0);
			person.velocity.y = persons[i].x(3,0);
			person.velocity.z = 0;
			person.reliability = 0.6;
			person.tagnames.push_back("LASER");
			sprintf(buf, "%d", persons[i].id);
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
		g = 0.0;
		b = 1.0;

		marker.header.frame_id = detection_frame_id_;
		marker.header.stamp = this->now();
		marker.ns = this->get_name();
		for(int i=0; i<(int)persons.size(); i++)
		{
			// Body
			marker.id = i;
			marker.type = visualization_msgs::msg::Marker::CYLINDER;
			marker.action = visualization_msgs::msg::Marker::ADD;
			marker.pose.position.x = persons[i].x(0,0);
			marker.pose.position.y = persons[i].x(1,0);
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
			marker.pose.position.x = persons[i].x(0,0);
			marker.pose.position.y = persons[i].x(1,0);
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
		for(int i=persons.size(); i<50; i++)
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
  auto pd_node = std::make_shared<PeopleDetectorNode>();
  pd_node->init();
  rclcpp::spin(pd_node);
  rclcpp::shutdown();
  return (0);
}
