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
    if(!s->intensities.empty())
    {
      m_sectors[s->angle_start].sector = s;
      m_sectors[s->angle_start].valid_position = false;
    }
  m_sector_buffer.clear();
  m_sector_buffer_mutex.unlock();

  double new_min_x = std::nan("");
  double new_max_x = std::nan("");
  double new_min_y = std::nan("");
  double new_max_y = std::nan("");

  // find transformations if missing
  for(auto& s: m_sectors)
  {
    if(!s.second.valid_position)
    {
      if(s.second.sector->range_max != m_last_range)
      {
        resetMap(0, 0, getSizeInCellsX(), getSizeInCellsY());
        m_last_range = s.second.sector->range_max;
        mapToWorld(0, 0, new_min_x, new_min_y);
        mapToWorld(getSizeInCellsX(), getSizeInCellsY(), new_max_x, new_max_y);
      }
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


        for(unsigned int i = 0; i < getSizeInCellsX(); i++)
          for(unsigned int j = 0; j < getSizeInCellsY(); j++)
          {
            double wx, wy;
            mapToWorld(i, j, wx, wy);
            float cost = s.second.getValue(wx, wy, m_blanking_distance);
            if(!std::isnan(cost))
            {
              setCost(i,j, cost*252);
              if(isnan(new_min_x))
              {
                new_min_x = wx;
                new_max_x = wx;
                new_min_y = wy;
                new_max_y = wy;
              }
              else
              {
                new_min_x = std::min(wx, new_min_x);
                new_max_x = std::max(wx, new_max_x);
                new_min_y = std::min(wy, new_min_y);
                new_max_y = std::max(wy, new_max_y);
              }
            }
          }
      }
    }
  }

  if(isnan(new_min_x))
  {
    current_ = false;
  }
  else
  {
    current_ = true;
    *min_x = new_min_x;
    *max_x = new_max_x;
    *min_y = new_min_y;
    *max_y = new_max_y;
  }
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
      master_grid.setCost(i,j, costmap_[index]);
    }
  }
}


float MarineRadarLayer::PositionedSector::getValue(double target_x, double target_y, double blanking_distance)
{
  if(valid_position && sector && !sector->intensities.empty())
  {
    double dx = target_x - x;
    double dy = target_y - y;
    double r = sqrt(dx*dx+dy*dy);
    if(r >= blanking_distance && r <= sector->range_max && r >= sector->range_min)
    {
      double theta = atan2(dy, dx);
      if (theta < 0.0)
        theta += 2.0*M_PI;
      theta -= yaw;
      if(theta < 0.0)
        theta += 2.0*M_PI;
      if (theta > 2.0*M_PI)
        theta -= 2.0*M_PI;
      double angle_proportion = -1.0;
      if(sector->angle_increment > 0.0)
      {
        double angle_max = sector->angle_start + sector->angle_increment*(sector->intensities.size()-1);
        angle_proportion = (theta-sector->angle_start)/(angle_max-sector->angle_start);
      }
      else
      {
        double angle_max = sector->angle_start + sector->angle_increment*(sector->intensities.size()-1);
        angle_proportion = -(theta-angle_max)/(sector->angle_start-angle_max);
      }
      if(angle_proportion >= 0.0 && angle_proportion <= 1.0)
      {
        int angle_index = (sector->intensities.size()-1)*angle_proportion;
        if(angle_index >= 0 && angle_index < sector->intensities.size())
        {
          double range_proportion = (r-sector->range_min)/(sector->range_max-sector->range_min);
          int range_index = (sector->intensities[angle_index].echoes.size()-1)*range_proportion;
          if(range_index >= 0 && range_index < sector->intensities[angle_index].echoes.size())
            return sector->intensities[angle_index].echoes[range_index];
        }
      }
    }
  }
  return std::nanf("");
}

} // namespace marine_radar_layer

