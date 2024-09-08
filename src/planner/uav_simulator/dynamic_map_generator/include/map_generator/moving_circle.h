/**
 * @file moving_circle.h
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2022-07-07
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __MOVING_CIRCLE_H__
#define __MOVING_CIRCLE_H__

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Eigen>
#include <random>

namespace dynamic_map_objects {
class MovingCircle {
 private:
  std::uniform_real_distribution<double> _rand_x, _rand_y, _rand_z, _rand_r, _rand_theta, _rand_v,
      _rand_omega;
  double _x_l, _x_h, _y_l, _y_h, _r_l, _r_h, _r_2, _v_h, _omega_h;
  double _resolution;

 public:
  pcl::PointCloud<pcl::PointXYZ> _cloud;

  double             x;       // x coordinate of the circle center
  double             y;       // y coordinate of the circle center
  double             z;       // y coordinate of the circle center
  double             r1, r2;  // outer radius
  double             dr;      // thickness of the circle
  double             vx;
  double             vy;
  double             omega;  // angular velocity
  double             theta;  // angle of the circle center
  Eigen::Quaterniond q;
  MovingCircle(double                      x_l,
               double                      x_h,
               double                      y_l,
               double                      y_h,
               double                      z_l,
               double                      z_h,
               double                      r_l,
               double                      r_h,
               double                      dr,
               double                      v_h,
               std::default_random_engine &eng,
               double                      resolution)
      : _rand_x(x_l, x_h)
      , _rand_y(y_l, y_h)
      , _rand_z(z_l, z_h)
      , _rand_r(r_l, r_h)
      , _rand_theta(0, 2 * M_PI)
      , _rand_v(-v_h, v_h)
      , _x_l(x_l)
      , _x_h(x_h)
      , _y_l(y_l)
      , _y_h(y_h)
      , _r_2(dr)
      , _v_h(v_h)
      , _resolution(resolution) {
    x        = _rand_x(eng);
    y        = _rand_y(eng);
    z        = _rand_z(eng);
    vx       = _rand_v(eng);
    vy       = _rand_v(eng);
    theta    = _rand_theta(eng);
    double r = _rand_r(eng);
    // random point in grid scale
    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;
    Eigen::Matrix3d rotate;
    rotate << cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1;
    q = Eigen::Quaterniond(rotate);
    q = q * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY());

    r1 = r + _r_2 / 2;
    r2 = r - _r_2 / 2;

    // draw a circle centered at (x,y,z)
    pcl::PointXYZ   pt;
    Eigen::Vector3d cpt;
    for (double angle = 0.0; angle < 2 * M_PI; angle += _resolution / 2) {
      cpt(0) = 0.0;
      cpt(1) = r1 * cos(angle);
      cpt(2) = r2 * sin(angle);
      Eigen::Vector3d cpt_if;
      for (int ifx = -0; ifx <= 0; ++ifx)
        for (int ify = -0; ify <= 0; ++ify)
          for (int ifz = -0; ifz <= 0; ++ifz) {
            cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution, ifz * _resolution);
            cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
            pt.x   = cpt_if(0);
            pt.y   = cpt_if(1);
            pt.z   = cpt_if(2);
            _cloud.push_back(pt);
          }
    }
    _cloud.width    = _cloud.points.size();
    _cloud.height   = 1;
    _cloud.is_dense = true;
  };

  ~MovingCircle() {}

  void update() {
    x += vx;
    y += vy;
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << vx, vy, 0;
    pcl::transformPointCloud(_cloud, _cloud, transform);

    if (x < _x_l || x > _x_h) {
      vx = -vx;
    }

    if (y < _y_l || y > _y_h) {
      vy = -vy;
    }
  }
};
}  // namespace dynamic_map_objects
#endif  // __MOVING_CIRCLE_H__
