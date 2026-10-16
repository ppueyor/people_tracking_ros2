#ifndef __LOCALINO_HPP__
#define __LOCALINO_HPP__

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <rclcpp/rclcpp.hpp>
#include <range_msgs/msg/p2_p_range.hpp>
#include <range_msgs/msg/p2_p_range_with_pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <sys/ioctl.h>
#include <net/if.h>
#include <arpa/inet.h>

struct Anchor
{
	uint64_t id;
	std::string frame_id;
	double x;
	double y;
	double z;
};

class Localino : public rclcpp::Node
{
public:

	// Class constructor
    Localino(void) : Node("localino_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
		// Load parameters
		port = this->get_parameter_or<int>("port", 10000);
		frame_id = this->get_parameter_or<std::string>("frame_id", "map");

		// Load anchor list
		loadAnchorList();
		std::cout << "Anchors configuration: " << std::endl;
		for(size_t i=0; i<a.size(); i++)
			std::cout << "\tAnchor " << a[i].id << ": (" << a[i].x << ", " << a[i].y << ", "<< a[i].z << ")" << std::endl;

		// Advertise publishers
		pubRange = this->create_publisher<range_msgs::msg::P2PRange>("/localino_node/range", 1);
		pubAnchorRange = this->create_publisher<range_msgs::msg::P2PRangeWithPose>("/localino_node/anchor/range", 1);
		pubVis = this->create_publisher<visualization_msgs::msg::MarkerArray>("/localino_node/anchor/markers", 1);

        // Launch marker array timer at 1Hz
		markerTimer = this->create_wall_timer(
			std::chrono::duration<double>(1.0),
			std::bind(&Localino::publishAnchorsMarkers, this));

		seq = 0;
    }

    // Class destructor
    ~Localino(void)
    {
	}

	bool connect(void)
	{
		// Create the socket
		if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 )
		{
			perror("[localino_node]: socket creation failed!");
			return false;
		}

		// Bind the server
		memset(&serv_addr, 0, sizeof(serv_addr));
		memset(&cli_addr, 0, sizeof(cli_addr));
		serv_addr.sin_family = AF_INET;
		serv_addr.sin_addr.s_addr = INADDR_ANY;
		serv_addr.sin_port = htons(port);
		if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
		{
			  perror("[localino_node]: error on socket binding!");
			  return false;
		}

		return true;
	}

	void process(void)
	{
		long long unsigned int anchorH, anchorL, tagH, tagL;
		socklen_t len;
		float range;
		int rssi, n;
		size_t i;
		char strA[32], strT[32];
		while(rclcpp::ok())
		{
			// Read one UDP package
			n = recvfrom(sockfd, (char *)buffer, 1024, MSG_WAITALL, ( struct sockaddr *) &cli_addr, &len);

			// Prepare message parsing
			buffer[n] = '\0';
			i = 0;
			while(buffer[i] != '\0')
			{
				if(buffer[i] == ',')
					buffer[i] = ' ';
				i++;
			}

			// Parse message
			if(strstr(buffer, "V") == NULL)
			{
				sscanf(buffer, "%s %s %f %d %d", strA, strT, &range, &seq, &rssi);
				parseNodeId(strA, anchorH, anchorL);
				parseNodeId(strT, tagH, tagL);
				//printf("New Range: %llx%llx %llx%llx %f %d %d\n", anchorH, anchorL, tagH, tagL, range-18.98, seq, rssi);
				printf("New Range: %llx %llx %f %d %d\n", anchorL, tagL, range-18.98, seq, rssi);

				// Fill-up range msg and publish
				rangeMsg.header.frame_id = frame_id;
				rangeMsg.header.stamp = this->now();
				rangeMsg.radiation_type = range_msgs::msg::P2PRange::RADIO;
				rangeMsg.source_id = tagL;
				rangeMsg.source_type = range_msgs::msg::P2PRange::TAG;
				rangeMsg.destination_id = anchorL;
				rangeMsg.destination_type = range_msgs::msg::P2PRange::BASE;
				rangeMsg.rssi = seq;
				rangeMsg.range = range-18.98;
				rangeMsg.variance = 0.2*0.2;
				pubRange->publish(rangeMsg);

				// Fill-up rangeWithPose msg and publish
				for(i=0; i<a.size(); i++)
					if(anchorL == a[i].id)
					{
						anchorRangeMsg.header.frame_id = frame_id;
						anchorRangeMsg.header.stamp = rangeMsg.header.stamp;
						anchorRangeMsg.radiation_type = range_msgs::msg::P2PRange::RADIO;
						anchorRangeMsg.source_id = tagL;
						anchorRangeMsg.source_type = range_msgs::msg::P2PRange::TAG;
						anchorRangeMsg.destination_id = anchorL;
						anchorRangeMsg.destination_type = range_msgs::msg::P2PRange::BASE;
						anchorRangeMsg.rssi = seq;
						anchorRangeMsg.range = range-18.98;
						anchorRangeMsg.variance = 0.2*0.2;
						anchorRangeMsg.position.header = anchorRangeMsg.header;
						anchorRangeMsg.position.point.x = a[i].x;
						anchorRangeMsg.position.point.y = a[i].y;
						anchorRangeMsg.position.point.z = a[i].z;
						pubAnchorRange->publish(anchorRangeMsg);
						break;
					}

				seq++;
			}

			rclcpp::spin_some(this->get_node_base_interface());
		}
	}

private:

	bool parseNodeId(char *str, long long unsigned int &idH, long long unsigned int &idL)
	{
		int len = strlen(str);
		if(len <= 16)
		{
			idH = 0;
			sscanf(str, "%llx", &idL);
			return true;
		}
		else if(len <= 32)
		{
			char strL[20], strH[20];
			int j = 0, i = 0;

			memset(strH, 0, 20);
			memset(strL, 0, 20);
			while(str[i] != '\0')
			{
				if(i<len-16)
				{
					strH[i] = str[i];
					j++;
				}
				else
					strL[i-j] = str[i];
				i++;
			}
			sscanf(strH, "%llx", &idH);
			sscanf(strL, "%llx", &idL);
			return true;
		}
		else
			return false;
	}

	void loadAnchorList(void)
	{
		// Clear anchors list
		a.clear();

		// Get the number of anchors to read
		int N = this->get_parameter_or<int>("anchor_list.N", 0);

		// Read each anchor and store in the list
		Anchor data;
		std::string id;
		for(int i=0; i<N; i++)
		{
			std::string s = "anchor_list.a" + std::to_string(i) + ".";
			if(this->get_parameter(s+"id", id) &&
			   this->get_parameter(s+"frame_id", data.frame_id) &&
			   this->get_parameter(s+"x", data.x) &&
			   this->get_parameter(s+"y", data.y) &&
			   this->get_parameter(s+"z", data.z))
			{
				if(id.find("0x") == 0 || id.find("0X") == 0)
					sscanf(id.c_str(), "%lx", &data.id);
				else
					sscanf(id.c_str(), "%lu", &data.id);
				a.push_back(data);
			}
		}
	}

	void publishAnchorsMarkers(void)
	{
		visualization_msgs::msg::MarkerArray map;
		visualization_msgs::msg::Marker marker;

		for(size_t i=0; i<a.size(); i++)
		{
			marker.header.frame_id = a[i].frame_id;
			marker.header.stamp = this->now();
			marker.ns = "localino_node";
			marker.id = 100+i;
			marker.type = visualization_msgs::msg::Marker::CUBE;
			marker.action = visualization_msgs::msg::Marker::ADD;
			marker.pose.position.x = a[i].x;
			marker.pose.position.y = a[i].y;
			marker.pose.position.z = a[i].z;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0;
			marker.pose.orientation.w = 0;
			marker.scale.x = 0.1;
			marker.scale.y = 0.1;
			marker.scale.z = 0.1;
			marker.color.a = 1.0;
			marker.color.r = 0;
			marker.color.g = 1;
			marker.color.b = 0;
			map.markers.push_back(marker);
		}

		pubVis->publish(map);
	}

	std::vector<Anchor> a;	// Anchor list
	int port;				// UDP Port
	std::string frame_id;	// Range frame id
	rclcpp::Publisher<range_msgs::msg::P2PRange>::SharedPtr pubRange;
	rclcpp::Publisher<range_msgs::msg::P2PRangeWithPose>::SharedPtr pubAnchorRange;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubVis;
	range_msgs::msg::P2PRange rangeMsg;
	range_msgs::msg::P2PRangeWithPose anchorRangeMsg;
	rclcpp::TimerBase::SharedPtr markerTimer;
	int sockfd;
	char buffer[1024];
	struct sockaddr_in serv_addr, cli_addr;
	int seq;
};

#endif
