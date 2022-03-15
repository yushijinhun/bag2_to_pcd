#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sstream>
#include <tf2/buffer_core.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer_interface.h>

static std::string nsec_to_string(int64_t nsec) {
	std::stringstream ss;
	ss << nsec / 1000000000LL << "." << std::setw(9) << std::setfill('0')
	   << nsec % 1000000000LL;
	return ss.str();
}

int main(int argc, char *argv[]) {
	if (argc < 4) {
		std::cerr
		    << "Syntax is: " << argv[0]
		    << " <rosbag2_file> <topic> <output_directory> [<target_frame>]"
		    << std::endl;
		std::cerr << "Example: " << argv[0]
		          << " ./rosbag2 /laser_tilt_cloud ./pointclouds base_link"
		          << std::endl;
		return -1;
	}

	std::string rosbag2_file = argv[1];
	std::string topic = argv[2];
	std::filesystem::path output_directory = argv[3];
	std::optional<std::string> target_frame;
	if (argc > 4) {
		target_frame = argv[4];
	}

	rosbag2_cpp::Reader reader;
	reader.open(rosbag2_file);

	rosbag2_storage::StorageFilter filter;
	filter.topics = {"/tf", "/tf_static", topic};
	reader.set_filter(filter);

	if (!std::filesystem::exists(output_directory)) {
		if (!std::filesystem::create_directories(output_directory)) {
			std::cerr << "Error creating directory " << output_directory
			          << std::endl;
			return -1;
		}
		std::cerr << "Creating directory " << output_directory << std::endl;
	}

	std::cerr << "Saving recorded sensor_msgs::PointCloud2 messages on topic "
	          << argv[2] << " to " << output_directory << std::endl;

	rclcpp::Serialization<tf2_msgs::msg::TFMessage> serialization_tfmessage;
	rclcpp::Serialization<sensor_msgs::msg::PointCloud2>
	    serialization_pointcloud2;
	tf2::BufferCore tf2_buffer;

	while (reader.has_next()) {
		auto raw_msg = reader.read_next();
		rclcpp::SerializedMessage serialized_msg(*raw_msg->serialized_data);

		if (raw_msg->topic_name == "/tf" ||
		    raw_msg->topic_name == "/tf_static") {
			tf2_msgs::msg::TFMessage msg;
			serialization_tfmessage.deserialize_message(&serialized_msg, &msg);
			bool is_static = raw_msg->topic_name == "/tf_static";
			for (auto &transform : msg.transforms) {
				tf2_buffer.setTransform(transform, "unknown", is_static);
			}

		} else if (raw_msg->topic_name == topic) {
			sensor_msgs::msg::PointCloud2 msg;
			serialization_pointcloud2.deserialize_message(&serialized_msg,
			                                              &msg);
			rclcpp::Time timestamp(msg.header.stamp);

			if (target_frame.has_value()) {
				try {
					auto transform_stamped = tf2_buffer.lookupTransform(
					    *target_frame, msg.header.frame_id,
					    tf2_ros::fromRclcpp(msg.header.stamp));
					tf2::Transform tf_transform;
					tf2::convert(transform_stamped.transform, tf_transform);
					Eigen::Matrix4f eigen_transform;
					pcl_ros::transformAsMatrix(tf_transform, eigen_transform);
					pcl_ros::transformPointCloud(eigen_transform, msg, msg);
					msg.header.frame_id = *target_frame;
				} catch (tf2::TransformException &ex) {
					std::cerr << "Failed to transform point cloud at "
					          << nsec_to_string(timestamp.nanoseconds()) << ": "
					          << ex.what() << std::endl;
					continue;
				}
			}

			pcl::PCLPointCloud2 pointcloud;
			pcl_conversions::toPCL(msg, pointcloud);

			auto output_file =
			    output_directory /
			    (nsec_to_string(timestamp.nanoseconds()) + ".pcd");
			pcl::PCDWriter pcd_writer;
			pcd_writer.writeBinaryCompressed(output_file, pointcloud);
		}
	}
}
