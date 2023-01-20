#ifndef NAV_TESTBENCH_UTILS_TF2_POLYGON_MSGS_HPP
#define NAV_TESTBENCH_UTILS_TF2_POLYGON_MSGS_HPP


#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <tf2/transform_datatypes.h>

namespace tf2
{

/** \brief Convert a tf2 Vector3 type to its equivalent geometry_msgs representation.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in A tf2 Vector3 object.
 * \return The Vector3 converted to a geometry_msgs message type.
 */
inline
geometry_msgs::msg::Point32& toMsg(const tf2::Vector3& in, geometry_msgs::msg::Point32& out)
{
  out.x = in.getX();
  out.y = in.getY();
  out.z = in.getZ();
  return out;
}

/** \brief Convert a Vector3 message to its equivalent tf2 representation.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h.
 * \param in A Vector3 message type.
 * \param out The Vector3 converted to a tf2 type.
 */
inline
void fromMsg(const geometry_msgs::msg::Point32& in, tf2::Vector3& out)
{
  out = tf2::Vector3(in.x, in.y, in.z);
}

/** \brief Apply a geometry_msgs TransformStamped to an geometry_msgs Point32 type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The point to transform, as a Point3 message.
 * \param t_out The transformed point, as a Point3 message.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
void doTransform(const geometry_msgs::msg::Point32& t_in, geometry_msgs::msg::Point32& t_out, const geometry_msgs::msg::TransformStamped& transform)
{
  tf2::Transform t;
  fromMsg(transform.transform, t);
  tf2::Vector3 v_in;
  fromMsg(t_in, v_in);
  tf2::Vector3 v_out = t * v_in;
  toMsg(v_out, t_out);
}

/********************/
/** PolygonStamped    **/
/********************/

// 
template <>
inline
void doTransform(const geometry_msgs::msg::PolygonStamped &p_in, geometry_msgs::msg::PolygonStamped &p_out, const geometry_msgs::msg::TransformStamped& transform)
{
  p_out = p_in;
  p_out.header.stamp = transform.header.stamp;
  p_out.header.frame_id = transform.header.frame_id;

  for(geometry_msgs::msg::Point32 &p : p_out.polygon.points)
  {
      doTransform(p, p, transform);
  }
}

// this method needs to be implemented by client library developers
template <>
inline
void doTransform(const geometry_msgs::msg::Polygon &p_in, geometry_msgs::msg::Polygon &p_out, const geometry_msgs::msg::TransformStamped& transform)
{
  p_out = p_in;

  for(geometry_msgs::msg::Point32 &p : p_out.points)
  {
      doTransform(p, p, transform);
  }
}

inline
geometry_msgs::msg::PolygonStamped toMsg(const geometry_msgs::msg::PolygonStamped &in)
{
  return in;
}

inline
void fromMsg(const geometry_msgs::msg::PolygonStamped &msg, geometry_msgs::msg::PolygonStamped &out)
{
  out = msg;
}

inline
geometry_msgs::msg::Polygon toMsg(const geometry_msgs::msg::Polygon &in)
{
  return in;
}

inline
void fromMsg(const geometry_msgs::msg::Polygon &msg, geometry_msgs::msg::Polygon &out)
{
  out = msg;
}

}

#endif // NAV_TESTBENCH_UTILS_TF2_POLYGON_MSGS_HPP