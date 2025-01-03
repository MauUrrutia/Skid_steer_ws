#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>


using std::placeholders::_1;


class PointCloudFilterNode : public rclcpp::Node {
public:
    PointCloudFilterNode(): Node("pcl_filter_node"){
        subscription = this->create_subscription
            <sensor_msgs::msg::PointCloud2>(
                "/camera/camera/depth/color/points", 10, 
                std::bind(&PointCloudFilterNode::pointcloudCallback, this, _1)); 
        publisher = this->create_publisher
            <sensor_msgs::msg::PointCloud2>(
                "/filtered/points", 10);
    };
private:
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        //Limpiar puntos vacios
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clean(new pcl::PointCloud<pcl::PointXYZ>());
        std::vector<int> index; // Indices de los puntos válidos
        pcl::removeNaNFromPointCloud(*cloud, *cloud_clean, index);


        //Voxeles
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f); //Tama;o del voxel
        voxel_filter.filter(*cloud_filtered);
        //Convertir el POintCloud a sensor_msg
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_filtered, output);
        output.header = msg->header;
        publisher->publish(output);
    };

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}