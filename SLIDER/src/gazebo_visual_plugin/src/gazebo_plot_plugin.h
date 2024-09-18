#ifndef GAZEBO_PLOT_PLUGIN_H_
#define GAZEBO_PLOT_PLUGIN_H_

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/MessageTypes.hh"

#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/ros.h>
#include <iostream>

#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Point.h>
// if you want some positions of the model use this....
#include <gazebo_msgs/ModelStates.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

using namespace std;
namespace gazebo
{
  namespace rendering
  {
    class SomeVisualPlugin : public VisualPlugin
    {
      public:
        /// \brief Constructor
        SomeVisualPlugin();

        /// \brief Destructor
        virtual ~SomeVisualPlugin();

        /// \brief Load the visual force plugin tags
        /// \param node XML config node
        void Load( VisualPtr _parent, sdf::ElementPtr _sdf );


      protected: 
        /// \brief Update the visual plugin
        virtual void UpdateChild();


      private:
        /// \brief pointer to ros node
        ros::NodeHandle* rosnode_;

        /// \brief store model name
        std::string model_name_;

        /// \brief topic name
        std::string topic_name_;

        // /// \brief The visual pointer used to visualize the force.
        VisualPtr visual_;

        // /// \brief The scene pointer.
        ScenePtr scene_;

        /// \brief For example a line to visualize the force
        DynamicLines *line1;
        DynamicLines *line2;
        DynamicLines *line3;
        DynamicLines *line4;
        double step1x;
        double step1y;

        double step2x;
        double step2y;

        double step3x;
        double step3y;

        double step4x;
        double step4y;
        int count = 0;
        int max_count = 10;
        const double LOOP_RATE = 50.0;
        

        /// \brief for setting ROS name space
        std::string visual_namespace_;

        /// \brief for setting ROS name space
        std::string visited_visual_namespace_;

        /// \Subscribe to some force
        ros::Subscriber force_sub_;

        /// \brief Visualize the force
        void VisualizeForceOnLink(const std_msgs::Float64MultiArray::ConstPtr& msg);

        void draw_rectangle1(double x, double y, double width, double length);
        void draw_rectangle2(double x, double y, double width, double length);
        void draw_rectangle3(double x, double y, double width, double length);
        void draw_rectangle4(double x, double y, double width, double length);

        // Pointer to the update event connection
        event::ConnectionPtr update_connection_;
    };
  }
}

#endif