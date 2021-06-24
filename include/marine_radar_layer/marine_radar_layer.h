#ifndef MARINE_RADAR_LAYER_H
#define MARINE_RADAR_LAYER_H

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <marine_radar_layer/MarineRadarLayerConfig.h>
#include <dynamic_reconfigure/server.h>
#include <marine_msgs/RadarSectorStamped.h>

namespace marine_radar_layer
{

class MarineRadarLayer: public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  MarineRadarLayer();

  virtual void onInitialize() override;
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y) override;
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)  override;

  bool isDiscretized();

  virtual void matchSize() override;

private:
  void reconfigureCallback(MarineRadarLayerConfig &config, uint32_t level);

  typedef dynamic_reconfigure::Server<MarineRadarLayerConfig> ReconfigureServer;
  typedef std::shared_ptr<ReconfigureServer> ReconfigureServerPtr;
  ReconfigureServerPtr m_reconfigureServer;

  void radarSectorCallback(const marine_msgs::RadarSectorStampedConstPtr &msg);

  std::list<marine_msgs::RadarSectorStampedConstPtr> m_sector_buffer;
  std::mutex m_sector_buffer_mutex;

  struct PositionedSector
  {
    bool valid_position = false;
    double yaw;
    double x;
    double y;
    marine_msgs::RadarSectorStampedConstPtr sector;
  };

  std::map<double, PositionedSector> m_sectors;

  std::string m_global_frame_id;

  ros::Subscriber m_radar_subscriber;

  float m_clear_threshold;
  float m_mark_threshold;
  float m_blanking_distance;
  float m_maximum_intensity;
};

} // namespace marine_radar_layer

#endif
