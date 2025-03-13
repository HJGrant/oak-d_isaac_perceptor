#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include <fstream>
#include <sstream>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"

#include "nvblox_msgs/msg/voxel_block_layer.hpp"
#include "nvblox_msgs/msg/index3_d.hpp"
#include "nvblox_msgs/msg/voxel_block.hpp"


using namespace std::chrono_literals;

class PlaybackNode : public rclcpp::Node
{
  public:
    PlaybackNode(const std::string & bag_filename)
    : Node("playback_node")
    {
      publisher_ = this->create_publisher<nvblox_msgs::msg::VoxelBlockLayer>("/nvblox_node/color_layer", 10);
      timer_ = this->create_wall_timer(
          100ms, std::bind(&PlaybackNode::timer_callback, this));

      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = bag_filename;
      reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
      reader_->open(storage_options);
    }

    private:
        void timer_callback()
        {
            // Create the folder for saving the CSV files if it doesn't exist
            std::filesystem::create_directory("my_ws/extracted_data/bottle_0.050/");

            // Process all messages in the bag
            size_t message_count = 0;

            while (reader_->has_next()) {
                rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_->read_next();

                if (msg->topic_name != "/nvblox_node/color_layer") {
                    continue;  // Skip if the topic doesn't match
                }

                rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
                nvblox_msgs::msg::VoxelBlockLayer::SharedPtr ros_msg = std::make_shared<nvblox_msgs::msg::VoxelBlockLayer>();

                // Deserialize the message
                serialization_.deserialize_message(&serialized_msg, ros_msg.get());

                // Check if there are blocks and that they contain valid voxel data
                if (ros_msg->blocks.empty()) {
                    continue;  // Skip if there are no blocks
                }

                bool has_valid_voxels = false;
                for (const auto& block : ros_msg->blocks) {
                    if (!block.centers.empty() && !block.colors.empty()) {
                        has_valid_voxels = true;
                        break;  // Found valid voxel data, no need to check further
                    }
                }

                // Skip if no valid voxel data exists
                if (!has_valid_voxels) {
                    continue;
                }

                // Construct a unique filename for each CSV based on the message count
                std::stringstream csv_filename;
                csv_filename << "my_ws/extracted_data/bottle_0.050/voxels_" << message_count << ".csv";

                // Export data to CSV
                std::ofstream csv_file(csv_filename.str());
                if (csv_file.is_open()) {
                    // Write the header with additional information
                    csv_file << "BlockSize_m,VoxelSize_m,LayerType,Clear,IndexX,IndexY,IndexZ,CenterX,CenterY,CenterZ,ColorR,ColorG,ColorB,ColorA\n";

                    // Extract additional data from the message
                    float block_size_m = ros_msg->block_size_m;
                    float voxel_size_m = ros_msg->voxel_size_m;
                    int32_t layer_type = ros_msg->layer_type;
                    bool clear = ros_msg->clear;

                    // Loop through each block and write voxel data
                    std::vector<nvblox_msgs::msg::Index3D> indices = ros_msg->block_indices;
                    std::vector<nvblox_msgs::msg::VoxelBlock> blocks = ros_msg->blocks;

                    size_t num_indices = indices.size();

                    for (size_t i = 0; i < num_indices; i++) {
                        std::vector<std_msgs::msg::ColorRGBA> colors = blocks[i].colors;
                        std::vector<geometry_msgs::msg::Point32> centers = blocks[i].centers;

                        size_t num_voxels = centers.size();
                        for (size_t j = 0; j < num_voxels; j++) {
                            // Write voxel data (Block size, Voxel size, Layer type, Clear flag, and voxel data) to CSV
                            csv_file << block_size_m << ","
                                    << voxel_size_m << ","
                                    << layer_type << ","
                                    << clear << ","
                                    << indices[i].x << ","
                                    << indices[i].y << ","
                                    << indices[i].z << ","
                                    << centers[j].x << ","
                                    << centers[j].y << ","
                                    << centers[j].z << ","
                                    << colors[j].r << ","
                                    << colors[j].g << ","
                                    << colors[j].b << ","
                                    << colors[j].a << "\n";
                        }
                    }
                    csv_file.close();
                    message_count++;  // Increment the message count for the next file name
                }
            }
        }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nvblox_msgs::msg::VoxelBlockLayer>::SharedPtr publisher_;

    rclcpp::Serialization<nvblox_msgs::msg::VoxelBlockLayer> serialization_;
    std::unique_ptr<rosbag2_cpp::Reader> reader_;
};

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <bag>" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlaybackNode>(argv[1]));
  rclcpp::shutdown();

  return 0;
}