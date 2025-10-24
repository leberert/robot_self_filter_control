// ============================ self_see_filter.h (Updated) ============================
#ifndef FILTERS_SELF_SEE_H_
#define FILTERS_SELF_SEE_H_

#include <algorithm>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <filters/filter_base.hpp>
#include <robot_self_filter/self_mask.h>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>  // For explicit constructor
#include <rcl_interfaces/msg/set_parameters_result.hpp>

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
  using ParameterMap = std::unordered_map<std::string, rclcpp::Parameter>;

  explicit SelfFilter(const rclcpp::Node::SharedPtr &node)
    : node_(node)
    , tf_buffer_(std::make_shared<tf2_ros::Buffer>(node_->get_clock()))
    , tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;

    node_->declare_parameter<double>("min_sensor_dist", 0.01, descriptor);
    node_->declare_parameter<bool>("keep_organized", false, descriptor);
    node_->declare_parameter<bool>("zero_for_removed_points", false, descriptor);
    node_->declare_parameter<bool>("invert", false, descriptor);
    node_->declare_parameter<bool>("shadow_filter_enabled", true, descriptor);

    node_->declare_parameter<std::vector<double>>("default_box_scale",
      {1.0, 1.0, 1.0}, descriptor);
    node_->declare_parameter<std::vector<double>>("default_box_padding",
      {0.01, 0.01, 0.01}, descriptor);
    node_->declare_parameter<std::vector<double>>("default_cylinder_scale",
      {1.0, 1.0}, descriptor);
    node_->declare_parameter<std::vector<double>>("default_cylinder_padding",
      {0.01, 0.01}, descriptor);
    node_->declare_parameter<double>("default_sphere_scale", 1.0, descriptor);
    node_->declare_parameter<double>("default_sphere_padding", 0.01, descriptor);

    node_->get_parameter("min_sensor_dist", min_sensor_dist_);
    node_->get_parameter("keep_organized", keep_organized_);
    node_->get_parameter("zero_for_removed_points", zero_for_removed_points_);
    node_->get_parameter("invert", invert_);
  node_->get_parameter("shadow_filter_enabled", shadow_filter_enabled_);

    node_->declare_parameter<std::vector<std::string>>(
      "self_see_links.names",
      std::vector<std::string>(),
      descriptor
    );
    node_->get_parameter("self_see_links.names", link_names_);

  refreshDefaultParameters();

    auto links = loadLinkInfos(link_names_);
    {
      std::lock_guard<std::mutex> lock(sm_mutex_);
      sm_ = std::make_shared<robot_self_filter::SelfMask<PointT>>(node_, *tf_buffer_, links, shadow_filter_enabled_);
    }

    parameter_callback_handle_ = node_->add_on_set_parameters_callback(
      std::bind(&SelfFilter::onParameterUpdate, this, std::placeholders::_1));
  }

  ~SelfFilter() override = default;

  bool configure() override
  {
    return true;
  }

  void getLinkNames(std::vector<std::string> &frames) override
  {
    std::lock_guard<std::mutex> lock(sm_mutex_);
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
    std::lock_guard<std::mutex> lock(sm_mutex_);
    return sm_.get();
  }

  template <typename Callable>
  void withSelfMask(Callable &&callable)
  {
    std::lock_guard<std::mutex> lock(sm_mutex_);
    callable(sm_.get());
  }

protected:
  bool update(const PointCloud &data_in, PointCloud &data_out) override
  {
    std::lock_guard<std::mutex> lock(sm_mutex_);
    if (!sm_)
    {
      data_out = data_in;
      return false;
    }

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

private:
  void refreshDefaultParameters(const ParameterMap &overrides = ParameterMap())
  {
    default_box_scale_ = getVectorParameter("default_box_scale", overrides, default_box_scale_);
    default_box_pad_   = getVectorParameter("default_box_padding", overrides, default_box_pad_);
    default_cyl_scale_ = getVectorParameter("default_cylinder_scale", overrides, default_cyl_scale_);
    default_cyl_pad_   = getVectorParameter("default_cylinder_padding", overrides, default_cyl_pad_);
    default_sphere_scale_ = getNumericParameter("default_sphere_scale", overrides, default_sphere_scale_);
    default_sphere_pad_   = getNumericParameter("default_sphere_padding", overrides, default_sphere_pad_);
  }

  robot_self_filter::LinkInfo loadLinkInfo(const std::string &lname,
                                           const ParameterMap &overrides = ParameterMap())
  {
    robot_self_filter::LinkInfo li;
    li.name   = lname;
    li.scale  = default_sphere_scale_;
    li.padding = default_sphere_pad_;

    const std::string prefix = "self_see_links." + lname + ".";

    rcl_interfaces::msg::ParameterDescriptor descriptor;

    const std::string box_scale_key   = prefix + "box_scale";
    const std::string box_padding_key = prefix + "box_padding";
    const std::string cyl_scale_key   = prefix + "cylinder_scale";
    const std::string cyl_padding_key = prefix + "cylinder_padding";
    const std::string padding_key     = prefix + "padding";
    const std::string scale_key       = prefix + "scale";

    if (!node_->has_parameter(box_scale_key))
    {
      node_->declare_parameter<std::vector<double>>(box_scale_key, std::vector<double>(), descriptor);
    }
    if (!node_->has_parameter(box_padding_key))
    {
      node_->declare_parameter<std::vector<double>>(box_padding_key, std::vector<double>(), descriptor);
    }
    li.box_scale   = getVectorParameter(box_scale_key, overrides, li.box_scale);
    li.box_padding = getVectorParameter(box_padding_key, overrides, li.box_padding);

    if (!node_->has_parameter(cyl_scale_key))
    {
      node_->declare_parameter<std::vector<double>>(cyl_scale_key, std::vector<double>(), descriptor);
    }
    if (!node_->has_parameter(cyl_padding_key))
    {
      node_->declare_parameter<std::vector<double>>(cyl_padding_key, std::vector<double>(), descriptor);
    }
    li.cylinder_scale   = getVectorParameter(cyl_scale_key, overrides, li.cylinder_scale);
    li.cylinder_padding = getVectorParameter(cyl_padding_key, overrides, li.cylinder_padding);

    if (!node_->has_parameter(padding_key))
    {
      node_->declare_parameter<double>(padding_key, default_sphere_pad_, descriptor);
    }
    li.padding = getNumericParameter(padding_key, overrides, li.padding);

    if (!node_->has_parameter(scale_key))
    {
      node_->declare_parameter<double>(scale_key, default_sphere_scale_, descriptor);
    }
    li.scale = getNumericParameter(scale_key, overrides, li.scale);

    return li;
  }

  std::vector<robot_self_filter::LinkInfo> loadLinkInfos(const std::vector<std::string> &names,
                                                        const ParameterMap &overrides = ParameterMap())
  {
    std::vector<robot_self_filter::LinkInfo> links;
    links.reserve(names.size());
    for (const auto &name : names)
    {
      links.push_back(loadLinkInfo(name, overrides));
    }
    return links;
  }

  std::vector<double> getVectorParameter(const std::string &name,
                                         const ParameterMap &overrides,
                                         const std::vector<double> &fallback)
  {
    auto it = overrides.find(name);
    if (it != overrides.end())
    {
      switch (it->second.get_type())
      {
        case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
          return it->second.as_double_array();
        case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
        {
          const auto &ints = it->second.as_integer_array();
          return std::vector<double>(ints.begin(), ints.end());
        }
        default:
          RCLCPP_WARN(node_->get_logger(),
                      "Unexpected parameter type for '%s'; keeping previous value", name.c_str());
          return fallback;
      }
    }

    std::vector<double> value = fallback;
    node_->get_parameter(name, value);
    return value;
  }

  double getNumericParameter(const std::string &name,
                             const ParameterMap &overrides,
                             double fallback)
  {
    auto it = overrides.find(name);
    if (it != overrides.end())
    {
      switch (it->second.get_type())
      {
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
          return it->second.as_double();
        case rclcpp::ParameterType::PARAMETER_INTEGER:
          return static_cast<double>(it->second.as_int());
        default:
          RCLCPP_WARN(node_->get_logger(),
                      "Unexpected parameter type for '%s'; keeping previous value", name.c_str());
          return fallback;
      }
    }

    double value = fallback;
    node_->get_parameter(name, value);
    return value;
  }

  bool getBoolParameter(const std::string &name,
                        const ParameterMap &overrides,
                        bool fallback)
  {
    auto it = overrides.find(name);
    if (it != overrides.end())
    {
      switch (it->second.get_type())
      {
        case rclcpp::ParameterType::PARAMETER_BOOL:
          return it->second.as_bool();
        case rclcpp::ParameterType::PARAMETER_INTEGER:
          return it->second.as_int() != 0;
        default:
          RCLCPP_WARN(node_->get_logger(),
                      "Unexpected parameter type for '%s'; keeping previous value", name.c_str());
          return fallback;
      }
    }

    bool value = fallback;
    node_->get_parameter(name, value);
    return value;
  }

  std::vector<std::string> getStringArrayParameter(const std::string &name,
                                                   const ParameterMap &overrides,
                                                   const std::vector<std::string> &fallback)
  {
    auto it = overrides.find(name);
    if (it != overrides.end())
    {
      if (it->second.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
      {
        return it->second.as_string_array();
      }

      RCLCPP_WARN(node_->get_logger(),
                  "Unexpected parameter type for '%s'; keeping previous value", name.c_str());
      return fallback;
    }

    std::vector<std::string> value = fallback;
    node_->get_parameter(name, value);
    return value;
  }

  bool isManagedLink(const std::string &name) const
  {
    return std::find(link_names_.begin(), link_names_.end(), name) != link_names_.end();
  }

  rcl_interfaces::msg::SetParametersResult onParameterUpdate(const std::vector<rclcpp::Parameter> &params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

  bool relevant = false;
    bool defaults_changed = false;
    bool names_changed = false;
  bool shadow_flag_changed = false;
    std::unordered_set<std::string> affected_links;

    ParameterMap overrides;
    overrides.reserve(params.size());

    for (const auto &param : params)
    {
      overrides[param.get_name()] = param;
      const std::string &name = param.get_name();
      if (name == "self_see_links.names")
      {
        names_changed = true;
        relevant = true;
        continue;
      }
      if (name == "shadow_filter_enabled")
      {
        shadow_flag_changed = true;
        relevant = true;
        continue;
      }
      if (name.rfind("self_see_links.", 0) == 0)
      {
        relevant = true;
        auto remainder = name.substr(std::string("self_see_links.").length());
        auto dot_pos = remainder.find('.');
        if (dot_pos != std::string::npos)
        {
          affected_links.insert(remainder.substr(0, dot_pos));
        }
        continue;
      }
      if (name.rfind("default_", 0) == 0)
      {
        defaults_changed = true;
        relevant = true;
      }
    }

    if (!relevant)
    {
      return result;
    }

    refreshDefaultParameters(overrides);

    bool previous_shadow_enabled = shadow_filter_enabled_;
    shadow_filter_enabled_ = getBoolParameter("shadow_filter_enabled", overrides, shadow_filter_enabled_);

    if (shadow_flag_changed && previous_shadow_enabled != shadow_filter_enabled_)
    {
      std::lock_guard<std::mutex> lock(sm_mutex_);
      if (sm_)
      {
        sm_->setShadowFilteringEnabled(shadow_filter_enabled_);
      }
    }

    if (names_changed)
    {
      std::vector<std::string> updated_names =
        getStringArrayParameter("self_see_links.names", overrides, link_names_);

  auto new_links = loadLinkInfos(updated_names, overrides);
  auto new_mask = std::make_shared<robot_self_filter::SelfMask<PointT>>(node_, *tf_buffer_, new_links, shadow_filter_enabled_);
      {
        std::lock_guard<std::mutex> lock(sm_mutex_);
        sm_ = new_mask;
        link_names_ = std::move(updated_names);
      }
      return result;
    }

    if (defaults_changed && affected_links.empty())
    {
      affected_links.insert(link_names_.begin(), link_names_.end());
    }

    if (affected_links.empty())
    {
      return result;
    }

    std::vector<robot_self_filter::LinkInfo> updates;
    updates.reserve(affected_links.size());
    for (const auto &name : affected_links)
    {
      if (!isManagedLink(name))
      {
        RCLCPP_WARN(node_->get_logger(), "Ignoring parameter update for unmanaged link '%s'", name.c_str());
        continue;
      }
      updates.push_back(loadLinkInfo(name, overrides));
    }

    if (!updates.empty())
    {
      std::lock_guard<std::mutex> lock(sm_mutex_);
      if (sm_)
      {
        sm_->setShadowFilteringEnabled(shadow_filter_enabled_);
        sm_->updateLinkParameters(updates);
      }
    }

    return result;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<robot_self_filter::SelfMask<PointT>> sm_;
  mutable std::mutex sm_mutex_;
  std::vector<std::string> link_names_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  bool keep_organized_ = false;
  bool zero_for_removed_points_ = false;
  bool invert_ = false;
  bool shadow_filter_enabled_ = true;
  double min_sensor_dist_ = 0.01;

  std::vector<double> default_box_scale_;
  std::vector<double> default_box_pad_;
  std::vector<double> default_cyl_scale_;
  std::vector<double> default_cyl_pad_;
  double default_sphere_scale_ = 1.0;
  double default_sphere_pad_ = 0.01;

  std::string sensor_frame_;
};

}  // namespace filters

#endif  // FILTERS_SELF_SEE_H_
