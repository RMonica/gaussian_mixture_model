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
#include "gmm_node.h"
#include <gaussian_mixture_model/gmm.h>
#include <gaussian_mixture_model/GaussianMixture.h>
#include <gaussian_mixture_model/Gaussian.h>

// ROS
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

// Boost
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

// STL
#include <string>
#include <deque>

// Eigen
#include <Eigen/Dense>

class GMMNode
{
  public:
  typedef unsigned int uint;

  GMMNode(ros::NodeHandle & nh): m_nh(nh), m_shutting_down_thread(&GMMNode::shutdownWaitingThread,this)
  {
    int temp_int;
    std::string temp_string;
    double temp_double;

    m_nh.param<int>(PARAM_NAME_GAUSSIAN_COUNT_MAX,temp_int,PARAM_DEFAULT_GAUSSIAN_COUNT_MAX);
    m_gaussian_count_max = (temp_int > 0) ? temp_int : 1;

    m_nh.param<int>(PARAM_NAME_GAUSSIAN_COUNT_MIN,temp_int,PARAM_DEFAULT_GAUSSIAN_COUNT_MIN);
    m_gaussian_count_min = (temp_int > 0) ? temp_int : 1;

    m_nh.param<std::string>(PARAM_NAME_DATA_INPUT_TOPIC,temp_string,PARAM_DEFAULT_DATA_INPUT_TOPIC);
    m_data_subscriber = m_nh.subscribe(temp_string,10,&GMMNode::onData,this);

    m_nh.param<double>(PARAM_NAME_BIC_TERM_THRESHOLD,temp_double,PARAM_DEFAULT_BIC_TERM_THRESHOLD);
    m_bic_termination_threshold = temp_double;

    m_nh.param<int>(PARAM_NAME_TIME_COLUMN_ID,temp_int,PARAM_DEFAULT_TIME_COLUMN_ID);
    m_time_column_id = temp_int >= 0 ? temp_int : PARAM_DEFAULT_TIME_COLUMN_ID;

    m_nh.param<std::string>(PARAM_NAME_MIX_OUTPUT_TOPIC,temp_string,PARAM_DEFAULT_MIX_OUTPUT_TOPIC);
    m_mix_publisher = m_nh.advertise<gaussian_mixture_model::GaussianMixture>(temp_string,2);
  }

  class TerminationHandler: public GMMExpectationMaximization::ITerminationHandler
  {
    public:
    bool isTerminated() {return ros::isShuttingDown(); }
  };

  ~GMMNode()
  {
    ros::shutdown();
    m_shutting_down_thread.join();
  }

  void Run()
  {
    while (!ros::isShuttingDown())
    {
      std_msgs::Float32MultiArrayConstPtr rosdata;

      // scope only
      {
        boost::mutex::scoped_lock lock(m_queue_mutex);
        while (m_queue.empty() && !ros::isShuttingDown())
          m_queue_cond.wait(lock);

        if (ros::isShuttingDown())
          return;

        rosdata = m_queue.front();
        m_queue.pop_front();
      }

      ROS_INFO("gmm: got a message.");

      if (rosdata->layout.dim.size() != 2)
      {
        ROS_ERROR("gmm: input array must contain two dimensions!!");
        continue;
      }

      uint dim = rosdata->layout.dim[1].size;
      uint ndata = rosdata->layout.dim[0].size;
      uint size = rosdata->data.size();

      if (dim * ndata != size)
      {
        ROS_ERROR("gmm: input array contains %u elements, but that is not the product of individual dimensions %u * %u",
          size,dim,ndata);
      }

      // convert input data
      Eigen::MatrixXf eigendata(ndata,dim);
      for (uint i = 0; i < ndata; i++)
        for (uint h = 0; h < dim; h++)
          eigendata(i,h) = rosdata->data[dim * i + h];

      // execute and find the best (lower) bic
      uint best_num_gauss = 0;
      float best_bic = 0.0;
      GMMExpectationMaximization::Ptr best_g;

      if (ros::isShuttingDown())
          return;

      for (uint num_gauss = m_gaussian_count_min; num_gauss <= m_gaussian_count_max; num_gauss++)
      {
        ROS_INFO("gmm: attempting gaussian count: %u",num_gauss);
        GMMExpectationMaximization::Ptr gmix(new GMMExpectationMaximization);
        gmix->setTerminationHandler(GMMExpectationMaximization::ITerminationHandler::Ptr(new TerminationHandler));

        gmix->execute(num_gauss,m_time_column_id,eigendata);

        if (ros::isShuttingDown())
          return;

        float bic = gmix->getBIC(eigendata);
        ROS_INFO("gmm: bic is: %f",bic);
        if (best_num_gauss == 0 || bic < best_bic)
        {
          best_bic = bic;
          best_num_gauss = num_gauss;
          best_g = gmix;
        }

        if (ros::isShuttingDown())
          return;

        // the bic is rising: exit
        if (bic - best_bic > m_bic_termination_threshold)
          break;
      }

      ROS_INFO("gmm: chosen gaussian count %u",best_num_gauss);

      // create the message and send
      const std::vector<Eigen::VectorXf> & means = best_g->getMeans();
      const std::vector<float> & weights = best_g->getWeights();
      const std::vector<Eigen::MatrixXf> & covariances = best_g->getCovariances();

      gaussian_mixture_model::GaussianMixturePtr mix_msg = gaussian_mixture_model::GaussianMixturePtr(new gaussian_mixture_model::GaussianMixture);
      mix_msg->bic = best_bic;
      mix_msg->gaussians.resize(best_num_gauss);
      mix_msg->weights.resize(best_num_gauss);
      for (uint s = 0; s < best_num_gauss; s++)
      {
        mix_msg->weights[s] = weights[s];
        gaussian_mixture_model::Gaussian & g = mix_msg->gaussians[s];
        g.means.resize(dim);
        g.covariances.resize(dim * dim);

        for (uint i = 0; i < dim; i++)
          g.means[i] = means[s][i];
        for (uint i = 0; i < dim; i++)
          for (uint h = 0; h < dim; h++)
            g.covariances[i * dim + h] = covariances[s](i,h);
      }

      m_mix_publisher.publish(mix_msg);
      ROS_INFO("gmm: message sent.");
    }
  }

  void onData(std_msgs::Float32MultiArrayConstPtr data)
  {
    boost::mutex::scoped_lock lock(m_queue_mutex);
    m_queue.push_back(data);
    m_queue_cond.notify_one();
  }

  void shutdownWaitingThread()
  {
    ros::waitForShutdown();

    // scope only
    {
      boost::mutex::scoped_lock lock(m_queue_mutex);
      m_queue_cond.notify_all();
    }
  }

  private:
  ros::NodeHandle & m_nh;

  boost::mutex m_queue_mutex;
  boost::condition_variable m_queue_cond;
  std::deque<std_msgs::Float32MultiArrayConstPtr> m_queue;

  uint m_gaussian_count_max;
  uint m_gaussian_count_min;

  uint m_time_column_id;

  float m_bic_termination_threshold;

  ros::Subscriber m_data_subscriber;
  ros::Publisher m_mix_publisher;

  // this thread will simply wait for shutdown
  // and unlock all the conditions variables
  boost::thread m_shutting_down_thread;
};

int main(int argc,char ** argv)
{
  ros::init(argc,argv,"gmm_node");

  ros::NodeHandle nh("~");
  GMMNode node(nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  node.Run();

  return 0;
}
