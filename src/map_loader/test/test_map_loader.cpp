#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <std_msgs/msg/string.hpp>
#include "map_loader/srv/load_map.hpp"
#include <thread>
#include <atomic>
#include <fstream>

std::atomic<bool> g_octomap_received{false};
std::atomic<bool> g_idx_received{false};

class TestMapLoader : public ::testing::Test {
protected:
  static void SetUpTestCase() {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestCase() {
    rclcpp::shutdown();
  }
};

TEST_F(TestMapLoader, LoadValidPCDEmitsOctomap) {
  // Create a synthetic PCD file with 5 known points
  std::string pcd_path = "/tmp/test_map_loader.pcd";
  {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.width = 5;
    cloud.height = 1;
    cloud.points.resize(5);
    // PCL 1.10 has no (x, y, z, intensity) constructor for PointXYZI,
    // so fill the fields explicitly to stay version-agnostic.
    auto set_point = [&cloud](std::size_t i, float x, float y, float z, float intensity) {
      cloud.points[i].x = x;
      cloud.points[i].y = y;
      cloud.points[i].z = z;
      cloud.points[i].intensity = intensity;
    };
    set_point(0, 0.f, 0.f, 0.f, 1.f);
    set_point(1, 1.f, 0.f, 0.f, 1.f);
    set_point(2, 0.f, 1.f, 0.f, 1.f);
    set_point(3, 0.f, 0.f, 1.f, 1.f);
    set_point(4, 2.f, 2.f, 0.5f, 1.f);
    pcl::io::savePCDFile(pcd_path, cloud);
  }

  // Spin node in background thread
  std::atomic<bool> node_ready{false};
  std::thread spinner([&node_ready]() {
    auto node = std::make_shared<rclcpp::Node>("test_map_loader_subscriber");
    auto octomap_sub = node->create_subscription<octomap_msgs::msg::Octomap>(
      "/map_loader/octomap", 10,
      [](const octomap_msgs::msg::Octomap::SharedPtr) {
        g_octomap_received = true;
      });
    auto idx_sub = node->create_subscription<std_msgs::msg::String>(
      "/map_loader/scan_context_index", 10,
      [](const std_msgs::msg::String::SharedPtr) {
        g_idx_received = true;
      });
    node_ready = true;
    rclcpp::spin(node);
  });

  // Wait for subscription setup
  while (!node_ready.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Call service
  auto service_node = rclcpp::Node::make_shared("test_service_caller");
  auto client = service_node->create_client<map_loader::srv::LoadMap>("map_loader/load_map");
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    spinner.detach();
    FAIL() << "Service not available";
  }

  auto req = std::make_shared<map_loader::srv::LoadMap::Request>();
  req->pcd_path = pcd_path;
  req->resolution = 0.1f;

  auto future = client->async_send_request(req);
  rclcpp::spin_until_future_complete(service_node, future, std::chrono::seconds(10));

  spinner.detach();

  ASSERT_TRUE(future.valid());
  auto res = future.get();
  EXPECT_TRUE(res->success);
  EXPECT_FALSE(res->octomap_path.empty());
  EXPECT_FALSE(res->scan_context_index_path.empty());
  EXPECT_TRUE(g_octomap_received);
  EXPECT_TRUE(g_idx_received);

  std::remove(pcd_path.c_str());
}
