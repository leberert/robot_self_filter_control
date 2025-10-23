// ============================ self_see_filter.h (Updated) ============================
#ifndef FILTERS_SELF_SEE_H_
#define FILTERS_SELF_SEE_H_

#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <filters/filter_base.hpp>
#include <robot_self_filter/self_mask.h>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>  // For explicit constructor

namespace filters
{

class SelfFilterInterface
{
public:
  virtual ~SelfFilterInterface() = default;

  virtual void getLinkNames(std::vector<std::string> &frames) = 0;

  virtual bool fillPointCloud2(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud2,
                               const std::string &sensor_frame,
                               sensor_msgs::msg::PointCloud2 &out2,
                               int &input_size,
                               int &output_size) = 0;
};

template <typename PointT>
class SelfFilter : public FilterBase<pcl::PointCloud<PointT>>, public SelfFilterInterface
{
public:
  using PointCloud = pcl::PointCloud<PointT>;

  explicit SelfFilter(const rclcpp::Node::SharedPtr &node)
    : node_(node)
    , tf_buffer_(std::make_shared<tf2_ros::Buffer>(node_->get_clock()))
    , tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    node_->declare_parameter<double>("min_sensor_dist", 0.01, rcl_interfaces::msg::ParameterDescriptor());
    node_->declare_parameter<bool>("keep_organized", false, rcl_interfaces::msg::ParameterDescriptor());
    node_->declare_parameter<bool>("zero_for_removed_points", false, rcl_interfaces::msg::ParameterDescriptor());
    node_->declare_parameter<bool>("invert", false, rcl_interfaces::msg::ParameterDescriptor());

    node_->declare_parameter<std::vector<double>>("default_box_scale",
      {1.0, 1.0, 1.0}, rcl_interfaces::msg::ParameterDescriptor());
    node_->declare_parameter<std::vector<double>>("default_box_padding",
      {0.01, 0.01, 0.01}, rcl_interfaces::msg::ParameterDescriptor());
    node_->declare_parameter<std::vector<double>>("default_cylinder_scale",
      {1.0, 1.0}, rcl_interfaces::msg::ParameterDescriptor());
    node_->declare_parameter<std::vector<double>>("default_cylinder_padding",
      {0.01, 0.01}, rcl_interfaces::msg::ParameterDescriptor());
    node_->declare_parameter<double>("default_sphere_scale", 1.0, rcl_interfaces::msg::ParameterDescriptor());
    node_->declare_parameter<double>("default_sphere_padding", 0.01, rcl_interfaces::msg::ParameterDescriptor());

    node_->get_parameter("min_sensor_dist", min_sensor_dist_);
    node_->get_parameter("keep_organized", keep_organized_);
    node_->get_parameter("zero_for_removed_points", zero_for_removed_points_);
    node_->get_parameter("invert", invert_);

    node_->get_parameter("default_box_scale", default_box_scale_);
    node_->get_parameter("default_box_padding", default_box_pad_);
    node_->get_parameter("default_cylinder_scale", default_cyl_scale_);
    node_->get_parameter("default_cylinder_padding", default_cyl_pad_);
    node_->get_parameter("default_sphere_scale", default_sphere_scale_);
    node_->get_parameter("default_sphere_padding", default_sphere_pad_);

    node_->declare_parameter<std::vector<std::string>>(
      "self_see_links.names",
      std::vector<std::string>(),
      rcl_interfaces::msg::ParameterDescriptor()
    );
    std::vector<std::string> link_names;
    node_->get_parameter("self_see_links.names", link_names);

    std::vector<robot_self_filter::LinkInfo> links;
    for (auto &lname : link_names)
    {
      robot_self_filter::LinkInfo li;
      li.name = lname;
      li.scale   = default_sphere_scale_;
      li.padding = default_sphere_pad_;

      std::string box_scale_key   = "self_see_links." + lname + ".box_scale";
      std::string box_padding_key = "self_see_links." + lname + ".box_padding";
      std::string cyl_scale_key   = "self_see_links." + lname + ".cylinder_scale";
      std::string cyl_padding_key = "self_see_links." + lname + ".cylinder_padding";
      std::string padding_key     = "self_see_links." + lname + ".padding";
      std::string scale_key       = "self_see_links." + lname + ".scale";

      node_->declare_parameter<std::vector<double>>(box_scale_key,
        std::vector<double>(), rcl_interfaces::msg::ParameterDescriptor());
      node_->declare_parameter<std::vector<double>>(box_padding_key,
        std::vector<double>(), rcl_interfaces::msg::ParameterDescriptor());
      node_->get_parameter(box_scale_key, li.box_scale);
      node_->get_parameter(box_padding_key, li.box_padding);

      node_->declare_parameter<std::vector<double>>(cyl_scale_key,
        std::vector<double>(), rcl_interfaces::msg::ParameterDescriptor());
      node_->declare_parameter<std::vector<double>>(cyl_padding_key,
        std::vector<double>(), rcl_interfaces::msg::ParameterDescriptor());
      node_->get_parameter(cyl_scale_key, li.cylinder_scale);
      node_->get_parameter(cyl_padding_key, li.cylinder_padding);

      node_->declare_parameter<double>(padding_key, default_sphere_pad_, rcl_interfaces::msg::ParameterDescriptor());
      node_->declare_parameter<double>(scale_key, default_sphere_scale_, rcl_interfaces::msg::ParameterDescriptor());
      double link_pad = default_sphere_pad_;
      double link_scl = default_sphere_scale_;
      node_->get_parameter(padding_key, link_pad);
      node_->get_parameter(scale_key, link_scl);
      li.padding = link_pad;
      li.scale   = link_scl;

      links.push_back(li);
      link_names_.push_back(lname);
    }

    sm_ = std::make_shared<robot_self_filter::SelfMask<PointT>>(node_, *tf_buffer_, links);

    // Setup parameter callback for runtime changes
    param_callback_handle_ = node_->add_on_set_parameters_callback(
      std::bind(&SelfFilter::parametersCallback, this, std::placeholders::_1));
  }

  ~SelfFilter() override = default;

  bool configure() override
  {
    return true;
  }

  void getLinkNames(std::vector<std::string> &frames) override
  {
    if (sm_) sm_->getLinkNames(frames);
  }

  bool fillPointCloud2(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud2,
                       const std::string &sensor_frame,
                       sensor_msgs::msg::PointCloud2 &out2,
                       int &input_size,
                       int &output_size) override
  {
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*cloud2, *cloud);

    pcl::PointCloud<PointT> out;
    sensor_frame_ = sensor_frame;
    update(*cloud, out);

    pcl::toROSMsg(out, out2);
    out2.header = cloud2->header;

    input_size  = static_cast<int>(cloud->points.size());
    output_size = static_cast<int>(out.points.size());
    return true;
  }

  robot_self_filter::SelfMask<PointT>* getSelfMaskPtr() const
  {
    return sm_.get();
  }

protected:
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : parameters)
    {
      std::string param_name = param.get_name();

      // Handle default parameters
      if (param_name == "min_sensor_dist")
      {
        min_sensor_dist_ = param.as_double();
        RCLCPP_INFO(node_->get_logger(), "Updated min_sensor_dist to: %f", min_sensor_dist_);
      }
      else if (param_name == "keep_organized")
      {
        keep_organized_ = param.as_bool();
      }
      else if (param_name == "zero_for_removed_points")
      {
        zero_for_removed_points_ = param.as_bool();
      }
      else if (param_name == "invert")
      {
        invert_ = param.as_bool();
      }
      else if (param_name == "default_sphere_scale")
      {
        default_sphere_scale_ = param.as_double();
      }
      else if (param_name == "default_sphere_padding")
      {
        default_sphere_pad_ = param.as_double();
      }
      else if (param_name == "default_box_scale")
      {
        default_box_scale_ = param.as_double_array();
      }
      else if (param_name == "default_box_padding")
      {
        default_box_pad_ = param.as_double_array();
      }
      else if (param_name == "default_cylinder_scale")
      {
        default_cyl_scale_ = param.as_double_array();
      }
      else if (param_name == "default_cylinder_padding")
      {
        default_cyl_pad_ = param.as_double_array();
      }
      // Handle per-link parameters
      else if (param_name.find("self_see_links.") == 0)
      {
        // Extract link name from parameter name
        // Format: self_see_links.<link_name>.<parameter_type>
        size_t first_dot = param_name.find('.', 15); // after "self_see_links."
        if (first_dot != std::string::npos)
        {
          std::string link_name = param_name.substr(15, first_dot - 15);
          std::string param_type = param_name.substr(first_dot + 1);

          // Check if this is a known link
          if (std::find(link_names_.begin(), link_names_.end(), link_name) != link_names_.end())
          {
            if (param_type == "box_scale")
            {
              auto values = param.as_double_array();
              if (values.size() == 3)
              {
                sm_->updateLinkScale(link_name, values, robot_self_filter::ScaleType::BOX);
                RCLCPP_INFO(node_->get_logger(), "Updated %s box_scale to: [%f, %f, %f]",
                           link_name.c_str(), values[0], values[1], values[2]);
              }
            }
            else if (param_type == "box_padding")
            {
              auto values = param.as_double_array();
              if (values.size() == 3)
              {
                sm_->updateLinkPadding(link_name, values, robot_self_filter::ScaleType::BOX);
                RCLCPP_INFO(node_->get_logger(), "Updated %s box_padding to: [%f, %f, %f]",
                           link_name.c_str(), values[0], values[1], values[2]);
              }
            }
            else if (param_type == "cylinder_scale")
            {
              auto values = param.as_double_array();
              if (values.size() == 2)
              {
                sm_->updateLinkScale(link_name, values, robot_self_filter::ScaleType::CYLINDER);
                RCLCPP_INFO(node_->get_logger(), "Updated %s cylinder_scale to: [%f, %f]",
                           link_name.c_str(), values[0], values[1]);
              }
            }
            else if (param_type == "cylinder_padding")
            {
              auto values = param.as_double_array();
              if (values.size() == 2)
              {
                sm_->updateLinkPadding(link_name, values, robot_self_filter::ScaleType::CYLINDER);
                RCLCPP_INFO(node_->get_logger(), "Updated %s cylinder_padding to: [%f, %f]",
                           link_name.c_str(), values[0], values[1]);
              }
            }
            else if (param_type == "scale")
            {
              double value = param.as_double();
              sm_->updateLinkScale(link_name, {value}, robot_self_filter::ScaleType::SPHERE);
              RCLCPP_INFO(node_->get_logger(), "Updated %s scale to: %f", link_name.c_str(), value);
            }
            else if (param_type == "padding")
            {
              double value = param.as_double();
              sm_->updateLinkPadding(link_name, {value}, robot_self_filter::ScaleType::SPHERE);
              RCLCPP_INFO(node_->get_logger(), "Updated %s padding to: %f", link_name.c_str(), value);
            }
          }
        }
      }
    }

    return result;
  }
  bool update(const PointCloud &data_in, PointCloud &data_out) override
  {
    std::vector<int> keep(data_in.points.size(), 0);
    if (sensor_frame_.empty())
      sm_->maskContainment(data_in, keep);
    else
      sm_->maskIntersection(data_in, sensor_frame_, min_sensor_dist_, keep);
    fillResult(data_in, keep, data_out);
    return true;
  }

  void fillResult(const PointCloud &data_in, const std::vector<int> &keep, PointCloud &data_out)
  {
    data_out.header = data_in.header;
    data_out.points.clear();
    data_out.points.reserve(data_in.points.size());

    PointT blank_pt;
    if (zero_for_removed_points_)
    {
      blank_pt.x = 0.0f;
      blank_pt.y = 0.0f;
      blank_pt.z = 0.0f;
    }
    else
    {
      blank_pt.x = std::numeric_limits<float>::quiet_NaN();
      blank_pt.y = std::numeric_limits<float>::quiet_NaN();
      blank_pt.z = std::numeric_limits<float>::quiet_NaN();
    }

    for (size_t i = 0; i < data_in.points.size(); ++i)
    {
      bool outside = (keep[i] == robot_self_filter::OUTSIDE);
      if (outside && !invert_)
      {
        data_out.points.push_back(data_in.points[i]);
      }
      else if (!outside && invert_)
      {
        data_out.points.push_back(data_in.points[i]);
      }
      else if (keep_organized_)
      {
        data_out.points.push_back(blank_pt);
      }
    }

    if (keep_organized_)
    {
      data_out.width  = data_in.width;
      data_out.height = data_in.height;
    }
    else
    {
      data_out.width  = static_cast<uint32_t>(data_out.points.size());
      data_out.height = 1;
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<robot_self_filter::SelfMask<PointT>> sm_;

  bool keep_organized_ = false;
  bool zero_for_removed_points_ = false;
  bool invert_ = false;
  double min_sensor_dist_ = 0.01;

  std::vector<double> default_box_scale_;
  std::vector<double> default_box_pad_;
  std::vector<double> default_cyl_scale_;
  std::vector<double> default_cyl_pad_;
  double default_sphere_scale_ = 1.0;
  double default_sphere_pad_ = 0.01;

  std::string sensor_frame_;
  std::vector<std::string> link_names_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace filters

#endif  // FILTERS_SELF_SEE_H_
