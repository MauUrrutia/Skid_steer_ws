#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <fstream>

using std::placeholders::_1;


class PointCloudFilterNode : public rclcpp::Node {
public:
    PointCloudFilterNode(): Node("pcl_filter_node"){
        subscription = this->create_subscription
            <sensor_msgs::msg::PointCloud2>(
                "/camera/realsense2_camera_node/depth/color/points", 10, 
                std::bind(&PointCloudFilterNode::pointcloudCallback, this, _1)); 
        publisher = this->create_publisher
            <sensor_msgs::msg::PointCloud2>(
                "/filtered/points", 10);
    };
private:
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(*msg, *cloud);

        //Limpiar puntos vacios
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clean(new pcl::PointCloud<pcl::PointXYZRGB>());
        std::vector<int> index; // Indices de los puntos válidos
        pcl::removeNaNFromPointCloud(*cloud, *cloud_clean, index);


        //Voxeles
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f); //Tama;o del voxel
        voxel_filter.filter(*cloud_filtered);

        //segmantacion
            //PAss throug filter (yo le digo filtro de corte LOL)
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_passthrough(new pcl::PointCloud<pcl::PointXYZRGB>());
        
        pcl::PassThrough<pcl::PointXYZRGB> passing_x;
        passing_x.setInputCloud(cloud_filtered);
        passing_x.setFilterFieldName("x");
        passing_x.setFilterLimits(-0.9, 0.9);
        passing_x.filter(*cloud_filtered_passthrough);

        pcl::PassThrough<pcl::PointXYZRGB> passing_y;
        passing_x.setInputCloud(cloud_filtered_passthrough);
        passing_x.setFilterFieldName("y");
        passing_x.setFilterLimits(-0.9, 0.9);
        passing_x.filter(*cloud_filtered_passthrough);

        //SEgmentacion de planos
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_segmented_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::SACSegmentation<pcl::PointXYZRGB> plane_segmentor;
        pcl::ExtractIndices<pcl::PointXYZRGB> indices_extractror;

        plane_segmentor.setInputCloud(cloud_filtered_passthrough);
        plane_segmentor.setModelType(pcl::SACMODEL_PLANE);
        plane_segmentor.setMethodType(pcl::SAC_RANSAC);
        plane_segmentor.setDistanceThreshold(0.005);
        plane_segmentor.segment(*inliers, *coefficients);

        indices_extractror.setInputCloud(cloud_filtered_passthrough);
        indices_extractror.setIndices(inliers);
        indices_extractror.setNegative(true);
        indices_extractror.filter(*plane_segmented_cloud);
        //Tipos de segmentacion //cilindro
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylindrical_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normals_estimator;
        pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> cylinder_segmentor;
        pcl::ExtractIndices<pcl::PointXYZRGB> cylinder_indices_extractor;


        normals_estimator.setSearchMethod(tree);
        normals_estimator.setInputCloud(plane_segmented_cloud);
        normals_estimator.setKSearch(30);
        normals_estimator.compute(*cloud_normals);
        
        cylinder_segmentor.setModelType(pcl::SACMODEL_CYLINDER);
        cylinder_segmentor.setMethodType(pcl::SAC_RANSAC);
        cylinder_segmentor.setDistanceThreshold(0.05);

        cylinder_segmentor.setInputCloud(plane_segmented_cloud);
        cylinder_segmentor.setInputNormals(cloud_normals);
        cylinder_segmentor.segment(*inliers, *coefficients);

        cylinder_indices_extractor.setInputCloud(plane_segmented_cloud);
        cylinder_indices_extractor.setIndices(inliers);
        cylinder_indices_extractor.setNegative(true);
        cylinder_indices_extractor.filter(*cylindrical_cloud);

        //Convertir el POintCloud a sensor_msg
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*plane_segmented_cloud, output);
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