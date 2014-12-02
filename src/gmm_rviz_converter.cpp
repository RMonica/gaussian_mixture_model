/*
 * Copyright (c) 2014, Riccardo Monica
 *   RIMLab, Department of Information Engineering, University of Parma, Italy
 *   http://www.rimlab.ce.unipr.it/
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other materials provided with
 * the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// custom
#include "gmm_rviz_converter.h"
#include <gaussian_mixture_model/GaussianMixture.h>
#include <gaussian_mixture_model/Gaussian.h>

// ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Eigen
#include <Eigen/Dense>

// STL
#include <sstream>

class GMMRvizConverter
{
  public:
  typedef unsigned int uint;

  GMMRvizConverter(ros::NodeHandle & nh): m_nh(nh)
  {
    std::string temp_string;
    double temp_double;
    int temp_int;

    m_nh.param<std::string>(PARAM_NAME_FRAME_ID,m_frame_id,PARAM_DEFAULT_FRAME_ID);

    m_nh.param<std::string>(PARAM_NAME_COORD_MASK,temp_string,PARAM_DEFAULT_COORD_MASK);
    // scope only
    {
      std::istringstream is(temp_string);
      while (is >> temp_string)
      {
        if (temp_string == PARAM_CMD_CONST)
          m_conversion_mask.push_back(IParamCMD::Ptr(new TParamCMDConst(is)));
        else if (temp_string == PARAM_CMD_INDEX)
          m_conversion_mask.push_back(IParamCMD::Ptr(new TParamCMDIndex(is)));
        else
          ROS_ERROR("gmm_rviz_converter: invalid parameter command: \"%s\".",temp_string.c_str());
      }

      if (m_conversion_mask.size() != NUM_OUTPUT_COORDINATES)
      {
        ROS_FATAL("gmm_rviz_converter: the coordinate_mask must contain 3 coordinates!");
        exit(21);
      }
    }

    m_nh.param<std::string>(PARAM_NAME_INPUT_TOPIC,temp_string,PARAM_DEFAULT_INPUT_TOPIC);
    m_sub = m_nh.subscribe(temp_string,1,&GMMRvizConverter::onGMM,this);

    m_nh.param<std::string>(PARAM_NAME_OUTPUT_TOPIC,temp_string,PARAM_DEFAULT_OUTPUT_TOPIC);
    m_pub = m_nh.advertise<visualization_msgs::MarkerArray>(temp_string,1);

    m_nh.param<double>(PARAM_NAME_SCALE,temp_double,PARAM_DEFAULT_SCALE);
    m_scale = temp_double;

    m_nh.param<int>(PARAM_NAME_MAX_MARKERS,temp_int,PARAM_DEFAULT_MAX_MARKERS);
    m_max_markers = temp_int > 0 ? temp_int : PARAM_DEFAULT_MAX_MARKERS;

    m_nh.param<std::string>(PARAM_NAME_RVIZ_NAMESPACE,m_rviz_namespace,PARAM_DEFAULT_RVIZ_NAMESPACE);

    m_nh.param<bool>(PARAM_NAME_NORMALIZE,m_normalize,PARAM_DEFAULT_NORMALIZE);
  }

  void onGMM(const gaussian_mixture_model::GaussianMixture & mix)
  {
    visualization_msgs::MarkerArray msg;
    ROS_INFO("gmm_rviz_converter: Received message.");

    uint i;

    for (i = 0; i < mix.gaussians.size(); i++)
    {
      visualization_msgs::Marker marker;

      marker.header.frame_id = m_frame_id;
      marker.header.stamp = ros::Time::now();
      marker.ns = m_rviz_namespace;
      marker.id = i;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.lifetime = ros::Duration();

      Eigen::Vector3f coords;
      for (uint ir = 0; ir < NUM_OUTPUT_COORDINATES; ir++)
        coords[ir] = m_conversion_mask[ir]->GetMean(m_conversion_mask,mix.gaussians[i]);
      marker.pose.position.x = coords.x();
      marker.pose.position.y = coords.y();
      marker.pose.position.z = coords.z();

      Eigen::Matrix3f covmat;
      for (uint ir = 0; ir < NUM_OUTPUT_COORDINATES; ir++)
        for (uint ic = 0; ic < NUM_OUTPUT_COORDINATES; ic++)
          covmat(ir,ic) = m_conversion_mask[ir]->GetCov(m_conversion_mask,mix.gaussians[i],ic);

      Eigen::EigenSolver<Eigen::Matrix3f> evsolver(covmat);

      Eigen::Matrix3f eigenvectors = evsolver.eigenvectors().real();
      if (eigenvectors.determinant() < 0.0)
        eigenvectors.col(0) = - eigenvectors.col(0);
      Eigen::Matrix3f rotation = eigenvectors;
      Eigen::Quaternionf quat = Eigen::Quaternionf(Eigen::AngleAxisf(rotation));

      marker.pose.orientation.x = quat.x();
      marker.pose.orientation.y = quat.y();
      marker.pose.orientation.z = quat.z();
      marker.pose.orientation.w = quat.w();

      Eigen::Vector3f eigenvalues = evsolver.eigenvalues().real();
      Eigen::Vector3f scale = Eigen::Vector3f(eigenvalues.array().abs().sqrt());
      if (m_normalize)
        scale.normalize();
      marker.scale.x = mix.weights[i] * scale.x() * m_scale;
      marker.scale.y = mix.weights[i] * scale.y() * m_scale;
      marker.scale.z = mix.weights[i] * scale.z() * m_scale;

      marker.color.a = 1.0;
      rainbow(float(i) / float(mix.gaussians.size()),marker.color.r,marker.color.g,marker.color.b);

      msg.markers.push_back(marker);
    }

    // this a waste of resources, but we need to delete old markers in some way
    // someone should add a "clear all" command to rviz
    // (using expiration time is not an option, here)
    for ( ; i < m_max_markers; i++)
    {
      visualization_msgs::Marker marker;
      marker.id = i;
      marker.action = visualization_msgs::Marker::DELETE;
      marker.lifetime = ros::Duration();
      marker.header.frame_id = m_frame_id;
      marker.header.stamp = ros::Time::now();
      marker.ns = m_rviz_namespace;
      msg.markers.push_back(marker);
    }

    m_pub.publish(msg);
    ROS_INFO("gmm_rviz_converter: Sent message.");
  }

  class IParamCMD
  {
    public:
    typedef boost::shared_ptr<IParamCMD> Ptr;
    typedef std::vector<Ptr> PtrVector;

    virtual ~IParamCMD() {}
    virtual double GetMean(const PtrVector & vec,const gaussian_mixture_model::Gaussian & g) = 0;
    virtual double GetCov(const PtrVector & vec,const gaussian_mixture_model::Gaussian & g,uint other_index) = 0;
    virtual bool IsConstant(const PtrVector & vec,const gaussian_mixture_model::Gaussian & g) = 0;
    virtual uint GetIndex(const PtrVector & vec,const gaussian_mixture_model::Gaussian & g) = 0;
  };

  class TParamCMDConst: public IParamCMD
  {
    public:
    TParamCMDConst(std::istream & s)
    {
      s >> m_const;
      if (!s)
        ROS_ERROR("gmm_rviz_converter: %s: real constant expected.",PARAM_CMD_CONST);
    }

    double GetMean(const PtrVector & /*vec*/,const gaussian_mixture_model::Gaussian & /*g*/) {return m_const; }
    double GetCov(const PtrVector & /*vec*/,const gaussian_mixture_model::Gaussian & /*g*/,uint /*other_index*/) {return 0.0; }
    bool IsConstant(const PtrVector & /*vec*/,const gaussian_mixture_model::Gaussian & /*g*/) {return true; }
    uint GetIndex(const PtrVector & /*vec*/,const gaussian_mixture_model::Gaussian & /*g*/) {return 0; }

    private:
    double m_const;
  };

  class TParamCMDIndex: public IParamCMD
  {
    public:
    TParamCMDIndex(std::istream & s)
    {
      m_index = 0;

      int temp_index;
      s >> temp_index;
      if (!s || temp_index < 0)
        ROS_ERROR("gmm_rviz_converter: %s: non-negative integer expected.",PARAM_CMD_INDEX);
      else
        m_index = temp_index;
    }

    double GetMean(const PtrVector & /*vec*/,const gaussian_mixture_model::Gaussian & g)
    {
      if (m_index >= g.means.size())
      {
        ROS_ERROR("gmm_rviz_converter: %s: index out of range: %u.",PARAM_CMD_INDEX,m_index);
        return 0.0;
      }

      return g.means[m_index];
    }

    double GetCov(const PtrVector & vec,const gaussian_mixture_model::Gaussian & g,uint other_index)
    {
      uint dim = g.means.size();

      if (m_index >= dim)
      {
        ROS_ERROR("gmm_rviz_converter: %s: index out of range: %u, max is %u.",PARAM_CMD_INDEX,m_index,dim);
        return 0.0;
      }

      // if one is constant, cov is always 0
      if (vec[other_index]->IsConstant(vec,g))
        return 0.0;

      uint other_real_index = vec[other_index]->GetIndex(vec,g);
      return g.covariances[m_index * dim + other_real_index];
    }

    bool IsConstant(const PtrVector & /*vec*/,const gaussian_mixture_model::Gaussian & /*g*/) {return false; }
    uint GetIndex(const PtrVector & /*vec*/,const gaussian_mixture_model::Gaussian & /*g*/) {return m_index; }

    private:
    uint m_index;
  };

  static void rainbow(float val,float & r,float & g,float & b)
  {
    if (val < 0.33)
    {
      r = 1.0 - (val * 3.0);
      g = val * 3.0;
      b = 0.0;
    }
    else if (val < 0.66)
    {
      r = 0.0;
      g = 1.0 - ((val - 0.33) * 3.0);
      b = (val - 0.33) * 3.0;
    }
    else
    {
      r = (val - 0.66) * 3.0;
      g = 0.0;
      b = 1.0 - ((val - 0.66) * 3.0);
    }

    // avoid invalid values caused by approximations
    if (g > 1.0) g = 1.0; if (g < 0.0) g = 0.0;
    if (r > 1.0) r = 1.0; if (r < 0.0) r = 0.0;
    if (b > 1.0) b = 1.0; if (b < 0.0) b = 0.0;
  }

  private:
  ros::NodeHandle & m_nh;

  std::string m_frame_id;
  IParamCMD::PtrVector m_conversion_mask;

  ros::Subscriber m_sub;
  ros::Publisher m_pub;

  float m_scale;
  bool m_normalize;

  uint m_max_markers;

  std::string m_rviz_namespace;
};

int main(int argc,char ** argv)
{
  ros::init(argc,argv,"gmm_rviz_converter");

  ros::NodeHandle nh("~");
  GMMRvizConverter cnv(nh);

  ros::spin();

  return 0;
}
