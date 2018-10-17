#ifndef RRTVISUAL_HPP
#define RRTVISUAL_HPP

#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <vector>

#define LINE_SCALE (0.01f)
#define LINE_ALPHA (0.1f)
#define POINT_SCALE (0.025f)
#define POINT_ALPHA (1.0f)
#define OBS_ALPHA (1.0f)
#define OBS_HEIGHT (1.0f)

class RRTVisual
{
public:
  RRTVisual(ros::NodeHandle &node, std::string postfix = std::string())
  {
    pfix = postfix;
    auto msg = std::string("/rrtvis_msgs") + postfix;
    pub = node.advertise<visualization_msgs::MarkerArray>(msg.c_str(), 10);
    // point.id = 0; lines.id = 1; obstacles.id = 2;
    point.ns = std::string("nodes")+postfix;
    lines.ns = std::string("lines")+postfix;
    obstacles.ns = std::string("obstacles")+postfix;
    point.header.frame_id = lines.header.frame_id = obstacles.header.frame_id = "/map";
    // point.action = visualization_msgs::Marker::ADD;
    // lines.action = visualization_msgs::Marker::ADD;
    point.type = visualization_msgs::Marker::POINTS;
    lines.type = visualization_msgs::Marker::LINE_LIST;
    obstacles.type = visualization_msgs::Marker::CYLINDER;
    lines.scale.x = lines.scale.y = lines.scale.z = LINE_SCALE;
    point.scale.x = point.scale.y = point.scale.z = POINT_SCALE;
    obstacles.scale.z = OBS_HEIGHT;
  }
private:
  template <size_t dim, size_t x_idx = 0, size_t y_idx = 1, size_t z_idx = 2, typename PointType>
  void add_point(visualization_msgs::Marker &pts, const PointType& point, double r = 1.0f, double g = 1.0f, double b = 1.0f, double alpha = 1.0f)
  {
    geometry_msgs::Point p;
    std_msgs::ColorRGBA c;
    c.r = r; c.b = b; c.g = g; c.a = alpha;
    p.x = point(x_idx); p.y = point(y_idx); p.z = (dim > 2? point(z_idx) : 0.0);
    pts.points.push_back(p);
    pts.colors.push_back(c);
  }

public:
  template <size_t dim, size_t x_idx = 0, size_t y_idx = 1, size_t z_idx = 2, typename Iterable>
  inline
  void add_points(const Iterable &nodes, double r = 1.0f, double g = 0.0f, double b = 0.0f, double alpha = 1.0f, const std::string ns = std::string(""))
  {
    visualization_msgs::Marker pts;
    std_msgs::ColorRGBA color;
    color.r = r; color.g = g; color.b = b; color.a = alpha;
    geometry_msgs::Point p;
    pts.header.frame_id = "/map";
    pts.header.seq = seq;
    pts.id = marker_array.markers.size();
    pts.header.stamp = ros::Time::now();
    pts.ns = std::string("points") + pfix + ns;
    pts.type = visualization_msgs::Marker::POINTS;
    pts.scale.x = pts.scale.y = pts.scale.z = POINT_SCALE;
    for(const auto& n : nodes) {
      add_point<dim,x_idx,y_idx,z_idx>(pts,n,r,g,b,alpha);
//      for(size_t i=0; i<dim; i++) {
//        switch (i) {
//        case x_idx: p.x = n(x_idx);
//          break;
//        case y_idx: p.y = n(y_idx);
//          break;
//        case z_idx: p.z = n(z_idx);
//          break;
//        default:
//          break;
//        }
//      }
//      pts.colors.push_back(color);
//      pts.points.push_back(p);
    }
    marker_array.markers.push_back(pts);
  }

  template <size_t x_idx = 0, size_t y_idx = 1, int z_idx = -1, typename Iterable>
  inline
  void add_circles(const Iterable &circles, double radius, double tickness, double z_offset = 0.0, double r = 1.0f, double g = 0.0f, double b = 0.0f, double alpha = 1.0f, const std::string &ns = std::string(""))
  {
    std_msgs::ColorRGBA color;
    color.r = r; color.g = g; color.b = b; color.a = alpha;
    auto now = ros::Time::now();
    auto nspace = std::string("circles") + pfix + ns;
    auto type = visualization_msgs::Marker::CYLINDER;
    for(const auto &c : circles) {
      visualization_msgs::Marker circle;
      circle.id = marker_array.markers.size();
      circle.header.frame_id = "/map";
      circle.header.seq = seq; circle.header.stamp = now;
      circle.ns = nspace; circle.type = type;
      circle.scale.x = circle.scale.y = radius; circle.scale.z = tickness;
      circle.pose.position.x = c(x_idx); circle.pose.position.y = c(y_idx);
      circle.pose.position.z = (z_idx >=0 ? c(z_idx) : z_offset);
      circle.color = color;
      marker_array.markers.push_back(circle);
    }
  }

  template <size_t dim, size_t x_idx = 0, size_t y_idx = 1, size_t z_idx = 2, typename Iterable>
  inline
  void add_trajectory(const Iterable &trajectory, double r = 1.0f, double g = 0.0f, double b = 0.0f, double alpha = 1.0f, const std::string &ns = std::string(""))
  {
    auto now = ros::Time::now();
    auto nspace = std::string("trajectory") + pfix + ns;
    auto type = visualization_msgs::Marker::LINE_STRIP;
    std_msgs::ColorRGBA color;
    color.r = r; color.g = g; color.b = b; color.a = alpha;
    visualization_msgs::Marker line;
    line.ns = nspace; line.color = color;
    line.id = marker_array.markers.size();
    line.header.stamp = now; line.type = type;
    line.header.frame_id = "/map"; line.header.seq = seq;
    line.scale.x = line.scale.y = line.scale.z = LINE_SCALE;
    for(const auto &t : trajectory) {
      geometry_msgs::Point pt;
      pt.x = t(x_idx); pt.y = t(y_idx); pt.z = (dim > 2 ? t(z_idx) : 0.0);
      line.points.push_back(pt);
      line.colors.push_back(color);
    }
    marker_array.markers.push_back(line);
  }

  template <size_t dim, size_t x_idx = 0, size_t y_idx = 1, size_t z_idx = 2, typename Iterable>
  inline
  void add_trajectories(const Iterable &trajectories, double r = 1.0f, double g = 0.0f, double b = 0.0f, double alpha = 1.0f, const std::string &ns = std::string(""))
  {
    visualization_msgs::Marker pts;
    std_msgs::ColorRGBA color;
    color.r = r; color.g = g; color.b = b; color.a = alpha;
    geometry_msgs::Point p;
    pts.header.frame_id = "/map";
    pts.header.seq = seq;
    pts.id = marker_array.markers.size();
    pts.header.stamp = ros::Time::now();
    pts.ns = std::string("points") + pfix + ns;
    pts.type = visualization_msgs::Marker::POINTS;
    pts.scale.x = pts.scale.y = pts.scale.z = POINT_SCALE;
    for(const auto& trajectory : trajectories) {
      add_trajectory<dim,x_idx,y_idx,z_idx>(trajectory,r,g,b,alpha,ns);
      add_point<dim,x_idx,y_idx,z_idx>(pts, trajectory.back(),r,g,b,1.0);
    }
    marker_array.markers.push_back(pts);
  }

  template <typename NodePosContainer, typename ParentIdxContainer>
  inline // NodePosType & ParentIndex should be container
  void set_nodes(const NodePosContainer &nodes_pos, const ParentIdxContainer &parent_map, int dim, size_t size)
  {
    // visualization_msgs::Marker point;
    // visualization_msgs::Marker lines;
    // point.header.frame_id = lines.header.frame_id = "/rrt_frame";
    point.header.stamp = lines.header.stamp = ros::Time::now();
    point.header.seq = lines.header.seq = seq;
    point.points.clear();
    lines.points.clear();
    point.colors.clear();
    lines.colors.clear();
    std_msgs::ColorRGBA pt_color, line_color;
    pt_color.a = POINT_ALPHA; pt_color.r = 1.0f;
    line_color.a = LINE_ALPHA; line_color.g = 1.0f;
    geometry_msgs::Point p;
    geometry_msgs::Point q;
    for(size_t i=0; i<size; i++) {
      auto n = nodes_pos.at(i);
      auto m = nodes_pos.at(std::max<int>(0,parent_map.at(i)));
      for(int i=0; i<dim; i++)
        switch (i) {
        case 0: p.x = n[0]; q.x = m[0]; break;
        case 1: p.y = n[1]; q.y = m[1]; break;
        case 2: p.z = n[2]; q.z = m[2]; break;
        default: break;
        }
      point.points.push_back(p);
      lines.points.push_back(q);
      lines.points.push_back(p);
      point.colors.push_back(pt_color);
      lines.colors.push_back(line_color);
      lines.colors.push_back(line_color);
    }
    auto n = nodes_pos.front();
    for(int i=0; i<dim; i++)
      switch (i) {
      case 0: p.x = n[0]; break;
      case 1: p.y = n[1]; break;
      case 2: p.z = n[2]; break;
      default: break;
      }
    pt_color.b = pt_color.g = pt_color.r = 1.0f;
    point.id = marker_array.markers.size();
    point.points.push_back(p);
    point.colors.push_back(pt_color);
    marker_array.markers.push_back(point);
    lines.id = marker_array.markers.size();
    marker_array.markers.push_back(lines);
  }
  template <typename Trajectory>
  inline
  void set_trajectory(const Trajectory &trajectory, int pos_dim)
  {
    std_msgs::ColorRGBA line_color;
    line_color.a = LINE_ALPHA; line_color.b = 1.0f;

    visualization_msgs::Marker line;
    line.scale.x = line.scale.y = line.scale.z = LINE_SCALE;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.id = marker_array.markers.size();
    line.header.stamp = ros::Time::now();
    line.header.frame_id = "/map";
    line.header.seq = seq;
    line.ns = std::string("trajectory")+pfix;
    for(const auto &t : trajectory) {
      geometry_msgs::Point p;
      for(int k=0; k<pos_dim; k++)
        switch (k) {
        case 0: p.x = t(0); break;
        case 1: p.y = t(1); break;
        case 2: p.z = t(2); break;
        default: break;
        }
      line.points.push_back(p);
    }
    marker_array.markers.push_back(line);
  }

  template <typename NodePosType, typename Trajectory, typename ParentIndex>
  inline
  void set_trajectories(const NodePosType &nodes_pos, const Trajectory &trajectories, const ParentIndex &parent_map, int pos_dim, size_t size)
  {
    point.points.clear();
    point.colors.clear();
    point.header.stamp = lines.header.stamp = ros::Time::now();
    point.header.seq = lines.header.seq = seq;
    std_msgs::ColorRGBA pt_color, line_color;
    pt_color.a = POINT_ALPHA; pt_color.r = 1.0f;
    line_color.a = LINE_ALPHA; line_color.g = 1.0f;
    geometry_msgs::Point p; geometry_msgs::Point q;
    point.scale.x = point.scale.y = point.scale.z = POINT_SCALE;
    for(size_t i=0; i<size; i++) {

      visualization_msgs::Marker line;
      line.scale.x = line.scale.y = line.scale.z = LINE_SCALE;
      line.type = visualization_msgs::Marker::LINE_STRIP;
      line.id = marker_array.markers.size();
      line.header.stamp = ros::Time::now();
      line.header.frame_id = "/map";
      line.header.seq = seq;
      line.ns = std::string("trajectory")+pfix;

      auto p_idx = std::max<int>(0,parent_map.at(i));
      auto n = nodes_pos.at(i);
      auto m = nodes_pos.at(p_idx);
      for(int k=0; k<pos_dim; k++)
        switch (k) {
        case 0: p.x = n[0]; q.x = m[0]; break;
        case 1: p.y = n[1]; q.y = m[1]; break;
        case 2: p.z = n[2]; q.z = m[2]; break;
        default: break;
        }
      point.points.push_back(p);
      point.colors.push_back(pt_color);

      auto path = trajectories.at(i).path();
      for(const auto& t : path) {
        geometry_msgs::Point l;
        for(int k=0; k<pos_dim; k++)
          switch (k) {
          case 0: l.x = t[0]; break;
          case 1: l.y = t[1]; break;
          case 2: l.z = t[2]; break;
          default: break;
          }
        line.points.push_back(l);
        line.colors.push_back(line_color);
      }

      marker_array.markers.push_back(line);
    }
    { // add start point
      geometry_msgs::Point p;
      auto n = nodes_pos.front();
      for(int i=0; i<pos_dim; i++)
        switch (i) {
        case 0: p.x = n[0]; break;
        case 1: p.y = n[1]; break;
        case 2: p.z = n[2]; break;
        default: break;
        }
      pt_color.r = pt_color.g = pt_color.b = 1.0f;
      point.points.push_back(p);
      point.colors.push_back(pt_color);
    }
    marker_array.markers.push_back(point);
  }

  inline
  void add_obstacles(double x, double y, double r)
  {
    obstacles.header.seq = seq;
    obstacles.header.stamp = ros::Time::now();
    obstacles.color.a = OBS_ALPHA;
    obstacles.scale.x = obstacles.scale.y = r*2;
    obstacles.pose.position.x = x;
    obstacles.pose.position.y = y;
    obstacles.pose.position.z = OBS_HEIGHT/2;
    obstacles.id = marker_array.markers.size();
    marker_array.markers.push_back(obstacles);
  }

  void publish()
  {
    seq++;
    pub.publish(marker_array);
  }

  void delete_all() {
    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(marker);
    pub.publish(arr);
  }
  void clear()
  {
    sphere_id = 2;
    marker_array.markers.clear();
  }
private:
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker point;
  visualization_msgs::Marker lines;
  visualization_msgs::Marker obstacles;
  ros::Publisher pub;
  std::string pfix;
  int seq = 0;
  int sphere_id = 2;
};

#endif // RRTVISUAL_HPP
