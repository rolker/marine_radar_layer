#include "marine_radar_layer/marine_radar_layer.h"
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/utils.h>

PLUGINLIB_EXPORT_CLASS(marine_radar_layer::MarineRadarLayer, costmap_2d::Layer)

namespace marine_radar_layer
{

MarineRadarLayer::MarineRadarLayer()
{

}

void MarineRadarLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = false;
  default_value_ = costmap_2d::NO_INFORMATION;
  matchSize();

  nh.param<float>("mark_threshold", m_mark_threshold, 8.0);
  nh.param<float>("clear_threshold", m_clear_threshold, 2.0);
  nh.param<float>("blanking_distance", m_blanking_distance, 4.0);
  nh.param<float>("maximum_intensity", m_maximum_intensity, 16.0);

  m_reconfigureServer = ReconfigureServerPtr(new ReconfigureServer(nh));
  m_reconfigureServer->setCallback(std::bind(&MarineRadarLayer::reconfigureCallback, this, std::placeholders::_1, std::placeholders::_2));

  m_global_frame_id = layered_costmap_->getGlobalFrameID();

  m_radar_subscriber = nh.subscribe("radar", 50, &MarineRadarLayer::radarSectorCallback, this);
}

void MarineRadarLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(), master->getOriginX(), master->getOriginY());
}

void MarineRadarLayer::reconfigureCallback(MarineRadarLayerConfig &config, uint32_t level)
{
  if(enabled_ != config.enabled)
  {
    enabled_ = config.enabled;
    current_ = false;
  }
}

void MarineRadarLayer::radarSectorCallback(const marine_sensor_msgs::RadarSectorConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(m_sector_buffer_mutex);
  m_sector_buffer.push_back(msg);
}

void MarineRadarLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  if (layered_costmap_->isRolling())
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);

  m_sector_buffer_mutex.lock();
  for(auto s: m_sector_buffer)
    if(!s->scanlines.empty())
    {
      m_sectors[s->scanlines.front().angle].sector = s;
      m_sectors[s->scanlines.front().angle].valid_position = false;
    }
  m_sector_buffer.clear();
  m_sector_buffer_mutex.unlock();

  // collect returns in map cell sized bins
  typedef std::pair<unsigned int, unsigned int> MapIndex;
  std::map<MapIndex, std::vector<uint8_t> > radar_returns;

  // find transformations if missing
  for(auto& s: m_sectors)
  {
    if(!s.second.valid_position)
    {
      geometry_msgs::PoseStamped in, out;
      in.header.stamp = s.second.sector->header.stamp;
      in.header.frame_id = s.second.sector->header.frame_id;
      in.pose.orientation.w = 1.0;

      if(tf_->canTransform(m_global_frame_id, in.header.frame_id, in.header.stamp, ros::Duration(1.0)))
      {
        tf_->transform(in, out, m_global_frame_id);
        s.second.x = out.pose.position.x;
        s.second.y = out.pose.position.y;
        s.second.yaw = tf2::getYaw(out.pose.orientation);
        s.second.valid_position = true;
        for(auto scanline: s.second.sector->scanlines)
        {
          double yaw = s.second.yaw + scanline.angle;
          double cos_yaw = cos(yaw);
          double sin_yaw = sin(yaw);
          double dr = scanline.range/double(scanline.intensities.size());
          for(int i = 0; i < scanline.intensities.size(); i++)
          {
            if(dr*i > m_blanking_distance)
            {
              unsigned int map_x, map_y;
              if(worldToMap(s.second.x+cos_yaw*dr*i, s.second.y+sin_yaw*dr*i, map_x, map_y))
                radar_returns[std::make_pair(map_x, map_y)].push_back(scanline.intensities[i]);
            }
          }
        }
      }
    }
  }

  current_ = !radar_returns.empty();
  
  if(radar_returns.empty())
    return;
  
  unsigned min_x_map, min_y_map, max_x_map, max_y_map;
  min_x_map = max_x_map = radar_returns.begin()->first.first;
  min_y_map = max_y_map = radar_returns.begin()->first.second;

  for(auto ret: radar_returns)
  {
    float sum = 0;
    for(auto intensity: ret.second)
      sum += intensity;
    float average_intensity = sum/float(ret.second.size());
    int index = getIndex(ret.first.first, ret.first.second);
    if(costmap_[index] == costmap_2d::NO_INFORMATION)
      costmap_[index] = average_intensity;
    else
    {
      costmap_[index] += average_intensity;
      costmap_[index] /= 2;
    }
    min_x_map = std::min(min_x_map, ret.first.first);
    max_x_map = std::max(max_x_map, ret.first.first);
    min_y_map = std::min(min_y_map, ret.first.second);
    max_y_map = std::max(max_y_map, ret.first.second);
  }

  ROS_INFO_STREAM("updateBounds in: " <<  *min_x << ", " << *min_y << " - " << *max_x << ", " << *max_y);
  
  mapToWorld(min_x_map, min_y_map, *min_x, *min_y);  
  mapToWorld(max_x_map, max_y_map, *max_x, *max_y);  

  ROS_INFO_STREAM("updateBounds out: " <<  *min_x << ", " << *min_y << " - " << *max_x << ", " << *max_y);
}

void MarineRadarLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;


  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = getIndex(i, j);
      if (costmap_[index] == costmap_2d::NO_INFORMATION)
        continue;
      if (costmap_[index] >= m_mark_threshold)
        master_grid.setCost(i, j, costmap_2d::LETHAL_OBSTACLE);
      else if(costmap_[index] <= m_clear_threshold)
        master_grid.setCost(i, j, costmap_2d::FREE_SPACE);
    }
  }
}

} // namespace marine_radar_layer

