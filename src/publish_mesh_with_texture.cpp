#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "mesh_msgs/msg/mesh_geometry_stamped.hpp"
#include "mesh_msgs/msg/mesh_materials_stamped.hpp"
#include "mesh_msgs/msg/mesh_texture.hpp"

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

mesh_msgs::msg::MeshMaterials make_cube_materials()
{
  mesh_msgs::msg::MeshMaterials mats;

  // one texture coordinate per vertex
  // mapping from 3D coordinates to 2D manifold
  mats.vertex_tex_coords.resize(8);
  mats.vertex_tex_coords[0].u = 0.0;
  mats.vertex_tex_coords[0].v = 0.0;
  mats.vertex_tex_coords[1].u = 0.1;
  mats.vertex_tex_coords[1].v = 0.1;
  mats.vertex_tex_coords[2].u = 0.2;
  mats.vertex_tex_coords[2].v = 0.2;
  mats.vertex_tex_coords[3].u = 0.3;
  mats.vertex_tex_coords[3].v = 0.3;
  mats.vertex_tex_coords[4].u = 0.4;
  mats.vertex_tex_coords[4].v = 0.4;
  mats.vertex_tex_coords[5].u = 0.5;
  mats.vertex_tex_coords[5].v = 0.5;
  mats.vertex_tex_coords[6].u = 0.6;
  mats.vertex_tex_coords[6].v = 0.6;
  mats.vertex_tex_coords[7].u = 0.7;
  mats.vertex_tex_coords[7].v = 0.7;

  mesh_msgs::msg::MeshMaterial mat;
  mat.color.r = 1.0;
  mat.color.g = 1.0;
  mat.color.b = 1.0;
  mat.color.a = 1.0;
  mat.has_texture = true;
  mat.texture_index = 5;

  mats.materials.resize(1);
  mats.materials[0] = mat;

  return mats;
}

mesh_msgs::msg::MeshTexture make_cube_texture()
{
  mesh_msgs::msg::MeshTexture tex;
  tex.texture_index = 0;
  // TODO: load image
  // 
  return tex;
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


class MeshPublisher : public rclcpp::Node
{
public:
  MeshPublisher()
  : rclcpp::Node("mesh_with_texture")
  {
    publisher_ = this->create_publisher<mesh_msgs::msg::MeshGeometryStamped>("mesh/geom", 1);
    publisher_mats_ = this->create_publisher<mesh_msgs::msg::MeshMaterialsStamped>("mesh/materials", 1);
    publisher_tex_ = this->create_publisher<mesh_msgs::msg::MeshTexture>("mesh/textures", 1);

    timer_ = this->create_wall_timer(
      30ms, std::bind(&MeshPublisher::timer_callback, this));
  
    geom_ = make_cube();
    mats_ = make_cube_materials();
  }

private:

  void timer_callback()
  {
    if(!first_stamp_)
    {
      first_stamp_ = this->now();
    }

    double dt = (this->now() - *first_stamp_).seconds();

    mesh_msgs::msg::MeshGeometryStamped msg_geom;
    msg_geom.header.stamp = this->now();
    msg_geom.header.frame_id = "map";
    msg_geom.uuid = "bla";

    geometry_msgs::msg::Point translation;
    translation.x = cos(dt * 2.0);
    translation.y = sin(dt);
    translation.z = cos(dt) * 2.0;

    msg_geom.mesh_geometry = move_mesh(geom_, translation);

    
    // 2. vertex colors
    mesh_msgs::msg::MeshMaterialsStamped msg_mats;  
    msg_mats.header.stamp = msg_geom.header.stamp;
    msg_mats.header.frame_id = msg_geom.header.frame_id;
    msg_mats.uuid = msg_geom.uuid;
    
    // TODO
    // msg_colors.mesh_vertex_colors = colors_;

    // publish geom and colors
    RCLCPP_INFO(this->get_logger(), "Publishing mesh geometry and textures");
    publisher_->publish(msg_geom);
    publisher_mats_->publish(msg_mats);
  }

  mesh_msgs::msg::MeshGeometry geom_;
  mesh_msgs::msg::MeshMaterials mats_;
  mesh_msgs::msg::MeshTexture tex_;

  std::optional<rclcpp::Time> first_stamp_;
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<mesh_msgs::msg::MeshGeometryStamped>::SharedPtr publisher_;
  rclcpp::Publisher<mesh_msgs::msg::MeshMaterialsStamped>::SharedPtr publisher_mats_;
  rclcpp::Publisher<mesh_msgs::msg::MeshTexture>::SharedPtr publisher_tex_;

};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MeshPublisher>());
  rclcpp::shutdown();
  return 0;
}