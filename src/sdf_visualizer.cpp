#include "voxblox_ground_truth/sdf_visualizer.h"
#include <voxblox/core/tsdf_map.h>
#include "voxblox/mesh/mesh_integrator.h"
#include "voxblox/mesh/mesh_layer.h"
#include "voxblox_ros/mesh_vis.h"

namespace voxblox_ground_truth {
SdfVisualizer::SdfVisualizer(ros::NodeHandle* nh_private)
    : tsdf_slice_height_(0.0) {
  CHECK_NOTNULL(nh_private);

  // Get parameters.
  nh_private->param("tsdf_slice_height", tsdf_slice_height_,
                    tsdf_slice_height_);

  // Advertise the topics to visualize the SDF map in Rviz
  tsdf_map_pub_ = nh_private->advertise<pcl::PointCloud<pcl::PointXYZI>>(
      "tsdf_map", 1, true);
  mesh_pub_ = nh_private->advertise<voxblox_msgs::Mesh>("mesh", 1, true);
  tsdf_map_surface_pub_ =
      nh_private->advertise<pcl::PointCloud<pcl::PointXYZI>>("tsdf_map_surface",
                                                             1, true);
  tsdf_slice_pub_ = nh_private->advertise<pcl::PointCloud<pcl::PointXYZI>>(
      "tsdf_slice", 1, true);
  intersection_count_pub_ =
      nh_private->advertise<pcl::PointCloud<pcl::PointXYZI>>(
          "intersection_counts", 1, true);
}

void voxblox_ground_truth::SdfVisualizer::publishIntersectionVisuals(
    const voxblox::Layer<IntersectionVoxel>& intersection_layer) {
  LOG(INFO) << "Publishing intersection counts";
  pcl::PointCloud<pcl::PointXYZI> intersection_count_msg;
  intersection_count_msg.header.frame_id = "world";
  voxblox::createColorPointcloudFromLayer<IntersectionVoxel>(
      intersection_layer, &visualizeIntersectionCount, &intersection_count_msg);
  intersection_count_pub_.publish(intersection_count_msg);
}

std::shared_ptr<voxblox::MeshLayer> computeMesh(const voxblox::Layer<voxblox::TsdfVoxel>& tsdf_layer)
{
  voxblox::MeshIntegratorConfig mesh_config;
  mesh_config.min_weight = 0.1f;
  auto mesh_layer = std::make_shared<voxblox::MeshLayer>(tsdf_layer.block_size());
  auto mesh_integrator = std::make_shared<voxblox::MeshIntegrator<voxblox::TsdfVoxel>>(mesh_config, tsdf_layer, mesh_layer.get());

  bool only_mesh_updated_blocks = false;
  bool clear_updated_flag = false;
  mesh_integrator->generateMesh(only_mesh_updated_blocks, clear_updated_flag);
  return mesh_layer;
}

void publishMesh(const ros::Publisher &pub, const std::shared_ptr<voxblox::MeshLayer> &mesh, const std::string& frame_id)
{
  voxblox_msgs::Mesh mesh_msg;
  voxblox::generateVoxbloxMeshMsg(mesh, voxblox::ColorMode::kNormals, &mesh_msg);
  mesh_msg.header.frame_id = frame_id;
  pub.publish(mesh_msg);
}

void voxblox_ground_truth::SdfVisualizer::publishTsdfVisuals(
    const voxblox::Layer<voxblox::TsdfVoxel>& tsdf_layer) {
  LOG(INFO) << "Publishing TSDF";
  pcl::PointCloud<pcl::PointXYZI> tsdf_map_ptcloud_msg;
  tsdf_map_ptcloud_msg.header.frame_id = "world";
  voxblox::createDistancePointcloudFromTsdfLayer(tsdf_layer,
                                                 &tsdf_map_ptcloud_msg);
  tsdf_map_pub_.publish(tsdf_map_ptcloud_msg);

  LOG(INFO) << "Publishing TSDF surface";
  pcl::PointCloud<pcl::PointXYZI> tsdf_map_surface_ptcloud_msg;
  tsdf_map_surface_ptcloud_msg.header.frame_id = "world";
  voxblox::createSurfaceDistancePointcloudFromTsdfLayer(
      tsdf_layer, 0.1, &tsdf_map_surface_ptcloud_msg);
  tsdf_map_surface_pub_.publish(tsdf_map_surface_ptcloud_msg);


  LOG(INFO) << "Publishing TSDF slice";
  pcl::PointCloud<pcl::PointXYZI> tsdf_slice_ptcloud_msg;
  tsdf_slice_ptcloud_msg.header.frame_id = "world";
  voxblox::createDistancePointcloudFromTsdfLayerSlice(
      tsdf_layer, 2, tsdf_slice_height_, &tsdf_slice_ptcloud_msg);
  tsdf_slice_pub_.publish(tsdf_slice_ptcloud_msg);

  LOG(INFO) << "Publishing mesh";
  publishMesh(mesh_pub_, computeMesh(tsdf_layer), "world");
}
}  // namespace voxblox_ground_truth
