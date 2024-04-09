#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "mesh_msgs/msg/mesh_geometry_stamped.hpp"

using namespace std::chrono_literals;


mesh_msgs::msg::MeshGeometry make_cube()
{
  mesh_msgs::msg::MeshGeometry cube;

  cube.vertices.resize(8);
  cube.faces.resize(2 * 6);

  cube.vertices[0].x = -0.5; cube.vertices[0].y = -0.5; cube.vertices[0].z = -0.5; 
  cube.vertices[1].x = -0.5; cube.vertices[1].y = -0.5; cube.vertices[1].z = 0.5;
  cube.vertices[2].x = -0.5; cube.vertices[2].y = 0.5; cube.vertices[2].z = 0.5;
  cube.vertices[3].x = -0.5; cube.vertices[3].y = 0.5; cube.vertices[3].z = -0.5;

  cube.faces[0].vertex_indices = {0, 1, 2};
  cube.faces[1].vertex_indices = {2, 3, 0};

  cube.vertices[4].x = 0.5; cube.vertices[4].y = 0.5; cube.vertices[4].z = -0.5;
  cube.vertices[5].x = 0.5; cube.vertices[5].y = 0.5; cube.vertices[5].z = 0.5;
  cube.vertices[6].x = 0.5; cube.vertices[6].y = -0.5; cube.vertices[6].z = 0.5;
  cube.vertices[7].x = 0.5; cube.vertices[7].y = -0.5; cube.vertices[7].z = -0.5;

  cube.faces[2].vertex_indices = {4, 5, 6};
  cube.faces[3].vertex_indices = {6, 7, 4};

  cube.faces[4].vertex_indices = {0, 7, 1};
  cube.faces[5].vertex_indices = {1, 7, 6};
  cube.faces[6].vertex_indices = {1, 6, 5};
  cube.faces[7].vertex_indices = {5, 2, 1};
  cube.faces[8].vertex_indices = {2, 5, 4};
  cube.faces[9].vertex_indices = {3, 2, 4};
  cube.faces[10].vertex_indices = {3, 4, 7};
  cube.faces[11].vertex_indices = {3, 7, 0};

  return cube;
}

mesh_msgs::msg::MeshGeometry move_mesh(
  const mesh_msgs::msg::MeshGeometry& mesh, 
  const geometry_msgs::msg::Point& vec)
{
  mesh_msgs::msg::MeshGeometry ret;

  ret.vertices = mesh.vertices;

  for(size_t i = 0; i < ret.vertices.size(); i++)
  {
    ret.vertices[i].x += vec.x;
    ret.vertices[i].y += vec.y;
    ret.vertices[i].z += vec.z;
  }

  ret.faces = mesh.faces;
  ret.vertex_normals = mesh.vertex_normals;

  return ret;
}



class MeshGeometryPublisher : public rclcpp::Node
{
public:
  MeshGeometryPublisher()
  : rclcpp::Node("mesh_publisher")
  {
    publisher_ = this->create_publisher<mesh_msgs::msg::MeshGeometryStamped>("mesh", 1);
    timer_ = this->create_wall_timer(
      50ms, std::bind(&MeshGeometryPublisher::timer_callback, this));
  
    cube_ = make_cube();
  }

private:

  void timer_callback()
  {
    if(!first_stamp_)
    {
      first_stamp_ = this->now();
    }

    double dt = (this->now() - *first_stamp_).seconds();

    mesh_msgs::msg::MeshGeometryStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.uuid = "bla";
    
    geometry_msgs::msg::Point translation;
    translation.x = 0.0;
    translation.y = 0.0;
    translation.z = cos(dt) * 5.0;

    msg.mesh_geometry = move_mesh(cube_, translation);

    RCLCPP_INFO(this->get_logger(), "Publishing mesh geometry");
    publisher_->publish(msg);
  }

  mesh_msgs::msg::MeshGeometry cube_;
  std::optional<rclcpp::Time> first_stamp_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<mesh_msgs::msg::MeshGeometryStamped>::SharedPtr publisher_;
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MeshGeometryPublisher>());
  rclcpp::shutdown();
  return 0;
}