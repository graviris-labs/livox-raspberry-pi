#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <livox_lidar_api.h>

typedef pcl::PointXYZI LivoxPoint;
typedef pcl::PointCloud<LivoxPoint> LivoxPointCloud;

// Global variables
LivoxPointCloud::Ptr cloud(new LivoxPointCloud);
pcl::visualization::CloudViewer viewer("Livox Point Cloud Viewer");
bool new_data = false;

// Callback for point cloud data
void PointCloudCallback(const uint8_t handle, const LivoxLidarEthernetPacket *data, void *client_data) {
    if (data == nullptr) {
        return;
    }
    
    // Process the data packet to extract point cloud data
    LivoxLidarPointCloudMessage *point_cloud_msg = 
        (LivoxLidarPointCloudMessage *)data->data;
        
    // Clear previous cloud
    cloud->clear();
    
    // Extract points from the packet
    uint32_t point_num = point_cloud_msg->point_num;
    for (uint32_t i = 0; i < point_num; ++i) {
        LivoxLidarPoint *lidar_point = 
            (LivoxLidarPoint *)(point_cloud_msg->points + i * sizeof(LivoxLidarPoint));
        
        LivoxPoint pcl_point;
        pcl_point.x = lidar_point->x;
        pcl_point.y = lidar_point->y;
        pcl_point.z = lidar_point->z;
        pcl_point.intensity = lidar_point->reflectivity;
        
        cloud->push_back(pcl_point);
    }
    
    new_data = true;
}

int main(int argc, char** argv) {
    // Initialize Livox LiDAR SDK
    if (!livox_lidar_sdk_init("/opt/livox_config/livox_lidar_config.json")) {
        std::cerr << "Livox SDK init failed." << std::endl;
        return -1;
    }
    
    // Register callback function
    LivoxLidarCallback lidar_cb;
    lidar_cb.handle_other_msg = NULL;
    lidar_cb.handle_packet_callback = PointCloudCallback;
    
    // Start visualization loop
    while (!viewer.wasStopped()) {
        if (new_data) {
            viewer.showCloud(cloud);
            new_data = false;
        }
    }
    
    // Deinitialize Livox LiDAR SDK
    livox_lidar_sdk_deinit();
    
    return 0;
}