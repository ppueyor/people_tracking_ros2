#include <rclcpp/rclcpp.hpp>
#include <people_msgs/msg/people.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cstdio>
#include <iostream>
#include <set>
#include "person_kf.hpp"

/**
 * @class PeopleTracker
 * @brief Fuse people detections coming from multiple sensor sources
 * (laser, RGBD, UWB) published on /people_detections into consolidated,
 * robust tracks.
 */
class PeopleTracker : public rclcpp::Node
{
public:
	// Ros stuff
	rclcpp::Subscription<people_msgs::msg::People>::SharedPtr peopleSub_;      /**< People detection subscriber*/
	rclcpp::Publisher<people_msgs::msg::People>::SharedPtr peoplePub_;         /**< Fused people publisher*/
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visPub_;   /**< Markers publisher*/

	// Node parameters
	std::string frame_id_;                             /**< Detection frame_id*/
	double track_distance_;                            /**< Maximum distance to consider detections */
	double track_timeout_;                             /**< Maximum time without detection to discard a track */
	int personCount_;                                  /**< For unique id */

	// Node objects
	std::vector<PersonKF> people_;                     /**< People being tracked */
	std::set<int> prevMarkerIds_;                      /**< Track ids whose markers were published last frame, for cleanup */

	/**
	 * \brief Constructor
	 */
	PeopleTracker() : Node("people_tracker_node")
	{
		// Topic subscription
		peopleSub_ = this->create_subscription<people_msgs::msg::People>(
			"/people_detections", 1, std::bind(&PeopleTracker::peopleCallback, this, std::placeholders::_1));

		// Read ros parameters
		frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
		track_distance_ = this->declare_parameter<double>("track_distance", 0.75);
		track_timeout_ = this->declare_parameter<double>("track_timeout", 2.0);

		// Launch publishers
		peoplePub_ = this->create_publisher<people_msgs::msg::People>("/people", 10);
		visPub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/people/markers", 1);

		// Setup variables
		personCount_ = 0;
	}

	/**
	 * \brief Destructor
	 */
	~PeopleTracker()
	{
	}

	/**
     * @brief People detection callback
     * @param People message
     */
    void peopleCallback(const people_msgs::msg::People::SharedPtr peopleMsg)
    {
		// Discard detections in frames other than frame_id_
		if(peopleMsg->header.frame_id != frame_id_)
		{
			std::cout << "[" << this->get_name() << "] Detection in unknown frame_id, discarting." << std::endl;
			return;
        }

		// Do we really have detections?
		if(peopleMsg->people.size() == 0)
			return;

		// Vector with flags to consider when a detection have been used
		std::vector<bool> used(peopleMsg->people.size(), false);

		// Define the variance of the measure
		double posDev;
		if(peopleMsg->people[0].tagnames[0] == "UWB")
		{
			posDev = 0.4;
			std::cout << "Using UWB deviation" << std::endl;
		}
		else if(peopleMsg->people[0].tagnames[0] == "RGBD")
		{
			posDev = 0.3;
			std::cout << "Using RGBD deviation" << std::endl;
		}
		else if(peopleMsg->people[0].tagnames[0] == "LASER")
		{
			posDev = 0.1;
			std::cout << "Using LASER deviation" << std::endl;
		}
		else
		{
			std::cout << "[" << this->get_name() << "] Unknown tagname detection: " << peopleMsg->people[0].tagnames[0] << std::endl;
			return;
		}

		// Update tracked people with detections accoring to name
		for(int i=0; i<(int)people_.size(); i++)
		{
			// Deactivate update flag
			people_[i].updated = false;

			// Search the closest detection to every person tracked
			for(int j=0; j<(int)peopleMsg->people.size(); j++)
			{
				if(!used[j] && peopleMsg->people[j].tagnames[0] == "UWB" &&
				   peopleMsg->people[j].name == people_[i].name)
				{
					std::cout << "Updating by UWB name" << std::endl;

					// Predict the position of the person to this instant
					people_[i].predict((this->now()-people_[i].tStamp).seconds());

					// Update the filter
					people_[i].update(peopleMsg->people[j].position.x, peopleMsg->people[j].position.y, posDev, this->now());

					// Mark the detection as used
					used[j] = true;

					std::cout << "Done" << std::endl;
				}
			}
		}

		// Update tracked people with detections accoding to min distance
		int k;
		double x1, y1, x2, y2, dist, minDist;
		for(int i=0; i<(int)people_.size(); i++)
		{
			if(people_[i].updated)
				continue;

			// Search the closest detection to every person tracked
			k = 0;
			x1 = people_[i].x(0,0);
			y1 = people_[i].x(1,0);
			minDist = 1000000.0;
			for(int j=0; j<(int)peopleMsg->people.size(); j++)
			{
				if(!used[j])
				{
					x2 = peopleMsg->people[j].position.x;
					y2 = peopleMsg->people[j].position.y;
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
				std::cout << "Updating person " << i << " with detection " << k << std::endl;

				// Predict the position of the person to this instant
				people_[i].predict((this->now()-people_[i].tStamp).seconds());

				// Update the filter
				people_[i].update(peopleMsg->people[k].position.x, peopleMsg->people[k].position.y, posDev, this->now());

				// Mark the detection as used
				used[k] = true;
				std::cout << "Done" << std::endl;
			}
		}

		// Add new persons to the list. Any unused detection (LASER, RGBD or UWB)
		// can bootstrap a new track; the originating detector is responsible for
		// filtering out its own false positives, and unconfirmed tracks are pruned
		// by the track_timeout below. This lets the laser (the most reliable and
		// widest-FOV sensor) create tracks, instead of depending on RGBD/UWB.
		for(int i=0; i<(int)peopleMsg->people.size(); i++)
		{
			if(!used[i])
			{
				std::cout << "New person added to the list: " << people_.size() << std::endl;
				PersonKF person;
				std::string name = (peopleMsg->people[i].tagnames[0] == "UWB") ? peopleMsg->people[i].name : "";
				person.init(personCount_++, peopleMsg->people[i].position.x, peopleMsg->people[i].position.y, 0, 0, this->now(), name);
				people_.push_back(person);

				used[i] = true;
			}
		}

		// Remove too old estimations without update
		std::vector<PersonKF> temp;
		for(int i=0; i<(int)people_.size(); i++)
		{
			if((this->now()-people_[i].tStamp).seconds() < track_timeout_)
				temp.push_back(people_[i]);
		}
		people_.clear();
		people_ = temp;

		// Publish people
		publishPeople();
		publishPersonMarker();
    }

	void publishPeople(void)
	{
		char buf[100];
		people_msgs::msg::People people;

		people.header.frame_id = frame_id_;
		people.header.stamp = this->now();
		// Publish every track that is still alive (stale ones were already pruned
		// above by track_timeout). Publishing only the tracks matched in the current
		// message would make the output flicker as laser/RGBD messages interleave.
		for(int i=0; i<(int)people_.size(); i++)
		{
			people_msgs::msg::Person person;
			person.name = "";
			person.position.x = people_[i].x(0,0);
			person.position.y = people_[i].x(1,0);
			person.position.z = 0;
			person.velocity.x = 0;
			person.velocity.y = 0;
			person.velocity.z = 0;
			person.reliability = 0.9;
			person.tagnames.push_back("FUSED");
			sprintf(buf, "%d", people_[i].id);
			person.tags.push_back(buf);
			people.people.push_back(person);
		}

		if(people.people.size() > 0)
			peoplePub_->publish(people);
	}

	void publishPersonMarker(void)
	{
		visualization_msgs::msg::MarkerArray peopleMarkers;
		visualization_msgs::msg::Marker marker;

		// Get marker color
		float r, g, b;
		r = 1.0;
		g = 1.0;
		b = 1.0;

		marker.header.frame_id = frame_id_;
		marker.header.stamp = this->now();
		marker.ns = this->get_name();

		// ADD a marker for every track that is still alive. All entries in
		// people_ are alive (stale ones were pruned by track_timeout before this
		// call), so we show them all steadily instead of only the ones matched in
		// the current message - the latter caused markers to flicker as laser/RGBD
		// messages interleaved.
		std::set<int> currentMarkerIds;
		for(int i=0; i<(int)people_.size(); i++)
		{
			currentMarkerIds.insert(people_[i].id);

			// Body
			marker.id = 10*people_[i].id;
			marker.type = visualization_msgs::msg::Marker::CYLINDER;
			marker.action = visualization_msgs::msg::Marker::ADD;
			marker.pose.position.x = people_[i].x(0,0);
			marker.pose.position.y = people_[i].x(1,0);
			marker.pose.position.z = 0.75;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;
			marker.scale.x = 0.4;
			marker.scale.y = 0.4;
			marker.scale.z = 1.5;
			marker.color.a = 1.0;
			marker.color.r = r;
			marker.color.g = g;
			marker.color.b = b;
			peopleMarkers.markers.push_back(marker);

			// Head
			marker.id = 10*people_[i].id+1;
			marker.type = visualization_msgs::msg::Marker::SPHERE;
			marker.action = visualization_msgs::msg::Marker::ADD;
			marker.pose.position.x = people_[i].x(0,0);
			marker.pose.position.y = people_[i].x(1,0);
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
			peopleMarkers.markers.push_back(marker);
		}

		// DELETE markers only for tracks that were shown last frame but are now
		// gone (timed out). This avoids re-deleting the same ids every frame.
		for(int id : prevMarkerIds_)
		{
			if(currentMarkerIds.count(id) == 0)
			{
				marker.id = 10*id;
				marker.type = visualization_msgs::msg::Marker::CYLINDER;
				marker.action = visualization_msgs::msg::Marker::DELETE;
				peopleMarkers.markers.push_back(marker);

				marker.id = 10*id+1;
				marker.type = visualization_msgs::msg::Marker::SPHERE;
				marker.action = visualization_msgs::msg::Marker::DELETE;
				peopleMarkers.markers.push_back(marker);
			}
		}
		prevMarkerIds_ = currentMarkerIds;

		// publish marker:
		if(!peopleMarkers.markers.empty())
			visPub_->publish(peopleMarkers);
	}
};


int main (int argc, char** argv)
{
	rclcpp::init(argc, argv);

	// People tracker instance
	auto node = std::make_shared<PeopleTracker>();

	// Spin for ever
	rclcpp::spin(node);

	rclcpp::shutdown();
	return 0;
}
