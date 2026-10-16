#ifndef __ROLOC2DEKF_HPP__
#define __ROLOC2DEKF_HPP__

#include <Eigen/Core>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <range_msgs/msg/p2_p_range_with_pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vector>
#include <time.h>
#include <stdlib.h>
#include <people_msgs/msg/people.hpp>

class RoLocEKF : public rclcpp::Node
{
public:

	struct rangeData
	{
		uint64_t id;
		float x;
		float y;
		float z;
		std::vector<float> range;
		float variance;
		float getMeanRange(void)
		{
			if(range.size() == 0)
				return -1.0;
			float mean = 0;
			for(size_t i=0; i<range.size(); i++)
				mean += range[i];
			mean = mean/((float)range.size());
			return mean;
		}
	};

	// Class constructor
    RoLocEKF(void) : Node("roloc_node")
    {
		// Load parameters
		std::string id;
        m_rangeDev = this->declare_parameter<double>("range_dev", -1.0);
		m_updateRate = this->declare_parameter<double>("update_rate", 2.0);
		m_heightCorrection = this->declare_parameter<double>("height_correction", 0.0);
		m_frameId = this->declare_parameter<std::string>("frame_id", "map");
		m_predicDev = this->declare_parameter<double>("prediction_dev", 2.0);
        id = this->declare_parameter<std::string>("tag", "0");
        if(id.find("0x") == 0 || id.find("0X") == 0)
			sscanf(id.c_str(), "%lx", &m_tag);
		else
			sscanf(id.c_str(), "%lu", &m_tag);
		m_name = this->declare_parameter<std::string>("name", "Unknown");

		// Init filter
		m_init = false;

		// Subscribe to range data topic and initial pose
        m_rangeSub = this->create_subscription<range_msgs::msg::P2PRangeWithPose>(
			"/range", 10, std::bind(&RoLocEKF::rangeDataCallback, this, std::placeholders::_1));
        //m_visPub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/markers", 0);
        m_peoplePub = this->create_publisher<people_msgs::msg::People>("/people_detections", 10);

        // Launch updater timer
        srand(time(NULL));
		m_updateTimer = this->create_wall_timer(
			std::chrono::duration<double>(1.0/m_updateRate),
			std::bind(&RoLocEKF::updateTimer, this));
		m_marker_id = rand();
    }

    // Class destructor
    ~RoLocEKF(void)
    {
	}

	// Filter predition based on odometry
    bool filterPredict(float dt)
    {
		// State vector does not change
		m_x(0) = m_x(0);
		m_x(1) = m_x(1);

		// Add noise to covariance matrix
		Eigen::Matrix<float, 2, 2> Q;
		Q.setIdentity();
		Q(0,0) = m_predicDev*m_predicDev*dt*dt; Q(1,1) = m_predicDev*m_predicDev*dt*dt;
		m_P = m_P + Q;

		return false;
    }

    // Filter update based on range updates
    bool filterUpdate(void)
    {
		// Check that we have enought updates
		int nUpdates = m_data.size();
		//std::cout << "N measurements: " << nUpdates << std::endl;
		if(nUpdates == 0)
		{
			return false;
		}

		// Get current state vector values
		double x = m_x(0);
		double y = m_x(1);

		// Get update equations
		Eigen::MatrixXf H, R;
		Eigen::VectorXf z, hx;
		H.setZero(nUpdates,2);
		z.setZero(nUpdates);
		hx.setZero(nUpdates);
		R.setIdentity(nUpdates, nUpdates);
		for(size_t i = 0; i<m_data.size(); i++)
		{
			// Get anchor position
			float px, py, pz;
			px = m_data[i].x;
			py = m_data[i].y;
			pz = m_data[i].z;

			// Get 2D projection of the 3D meassurement cosidering the sensor height
			z(i) = m_data[i].getMeanRange();

			// Get update equation
			hx(i) = sqrt((x-px)*(x-px) + (y-py)*(y-py));

			// Compute measurement jacobian
			H(i,0) = -(2*px - 2*x)/(2*hx(i)); H(i,1) = -(2*py - 2*y)/(2*hx(i));

			// Get the update noise matrix
			if(m_rangeDev > 0.0)
				R(i,i) = m_rangeDev*m_rangeDev;
			else
				R(i,i) = m_data[i].variance;
		}
		//std::cout << "z:" << z << std::endl;
		//std::cout << "hx:" << hx << std::endl;

		// Compute innovation matrix
		Eigen::MatrixXf S = H*m_P*H.transpose()+R;

		// Compute kalman gain
		Eigen::MatrixXf K = m_P*H.transpose()*S.inverse();

		// Compute innovation vector
		Eigen::VectorXf in = z - hx;

		// Compute new state vector
		m_x = m_x + K*in;

		// Compute new covariance matrix
		Eigen::Matrix<float, 2, 2> I, aux;
		I.setIdentity();
		aux = (I-K*H)*m_P;
		m_P = aux;

		// Clean range measurement vector
		m_data.clear();

		//std::cout << "x:\n" << m_x.transpose() << std::endl;
		//std::cout << "P:\n" << m_P << std::endl;

		return true;
	}

	void publishPeopleDetection(void)
	{
		char buf[100];

		people_msgs::msg::People people;
		people_msgs::msg::Person person;

		people.header.frame_id = m_frameId;
		people.header.stamp = this->now();

		person.name = m_name;
		person.position.x = m_x(0);
		person.position.y = m_x(1);
		person.position.z = 0;
		person.velocity.x = 0;
		person.velocity.y = 0;
		person.velocity.z = 0;
		person.reliability = 1;
		person.tagnames.push_back("UWB");
		sprintf(buf, "%lu", m_tag);
		person.tags.push_back(buf);

		people.people.push_back(person);
		m_peoplePub->publish(people);
	}


private:

	// Periodically update the filter
	void updateTimer(void)
	{
		if(m_init)
		{
			// Predict state
			filterPredict(1.0/m_updateRate);

			// Update state
			if(filterUpdate())
				publishPeopleDetection();
		}
		else if(m_data.size() >2)
		{
			// Init filter
			m_init = true;
			initFilter(0,0);

			// Update state
			filterUpdate();

			// Publish estimation on visual markers
			//publishPersonMarker();
			publishPeopleDetection();
		}
	}

	// Range data callback
	void rangeDataCallback(const range_msgs::msg::P2PRangeWithPose::SharedPtr msg)
	{
		// Check if tag id matches
		if(m_tag != 0 && msg->source_id != m_tag)
		{
			return;
		}

		// Check if range frame_id is corect
		if(msg->header.frame_id != m_frameId)
		{
			std::cout << "[roloc_ekf] Incoming range frame_id does not match with roloc_ekf frame_id!!" << std::endl;
			std::cout << "\tDescarting range" << std::endl;
			return;
		}

		// Do we have already measurements from this anchor?
		int index = -1;
		for(size_t i = 0; i<m_data.size(); i++)
			if(msg->destination_id == m_data[i].id)
			{
				index = i;
				break;
			}

		// Save range meaurement
		if(index < 0)
		{
			rangeData anchor;
			anchor.id = msg->destination_id;
			anchor.x = msg->position.point.x;
			anchor.y = msg->position.point.y;
			anchor.z = msg->position.point.z;
			anchor.variance = msg->variance;
			anchor.range.push_back(msg->range);
			m_data.push_back(anchor);
		}
		else
			m_data[index].range.push_back(msg->range);
	}

	bool initFilter(double x, double y)
	{
		// Init covariance matrix
		m_P.setIdentity();
		m_P(0,0) = 10.0*10.0;
		m_P(1,1) = 10.0*10.0;

		// Init state vector
		m_x(0) = x;
		m_x(1) = y;

		return true;
	}

	// Parameters
	std::string m_frameId;
	double m_rangeDev;
	double m_heightCorrection;
	double m_updateRate;
	double m_predicDev;
	bool m_init;
	uint64_t m_tag;
	int m_marker_id;
	std::string m_name;

	// Range measurements
	std::vector<rangeData> m_data;

	// RoLoc Covariance matrix
    Eigen::Matrix<float, 2, 2> m_P;

    // RoLoc state vector x = [x,y]'
	Eigen::Matrix<float, 2, 1> m_x;

	// Subscriber data and timer
	rclcpp::Subscription<range_msgs::msg::P2PRangeWithPose>::SharedPtr m_rangeSub;
	rclcpp::TimerBase::SharedPtr m_updateTimer;
	rclcpp::Publisher<people_msgs::msg::People>::SharedPtr /*m_visPub, */m_peoplePub;
};

#endif
