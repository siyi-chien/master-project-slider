#include "gazebo_plot_plugin.h"

namespace gazebo
{
  namespace rendering
  {

    ////////////////////////////////////////////////////////////////////////////////
    // Constructor
    SomeVisualPlugin::SomeVisualPlugin():line1(NULL),line2(NULL),line3(NULL),line4(NULL)
    {
      cout<<"I am here0!!!!"<<endl;
      cout<<"I am here0!!!!"<<endl;
      cout<<"I am here0!!!!"<<endl;
      cout<<"I am here0!!!!"<<endl;
      cout<<"I am here0!!!!"<<endl;
    } 

    ////////////////////////////////////////////////////////////////////////////////
    // Destructor
    SomeVisualPlugin::~SomeVisualPlugin()
    {
      // Finalize the visualizer
      this->rosnode_->shutdown();
      delete this->rosnode_;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Load the plugin

    void SomeVisualPlugin::draw_rectangle1(double x, double y, double width, double length)
    {
      double xy_ratio = 20;
      double x_min=(x-length/2)/xy_ratio;
      double x_max=(x+length/2)/xy_ratio;
      double y_min=(y+width/2)/xy_ratio;
      double y_max=(y-width/2)/xy_ratio;

      this->line1->Clear();
        this->line1->AddPoint(ignition::math::Vector3d(x_min,y_min,0));
        this->line1->AddPoint(ignition::math::Vector3d(x_min,y_max,0));
        this->line1->AddPoint(ignition::math::Vector3d(x_max,y_max,0));
        this->line1->AddPoint(ignition::math::Vector3d(x_max,y_min,0));
        this->line1->AddPoint(ignition::math::Vector3d(x_min,y_min,0));
      
      

      this->line1->setMaterial("Gazebo/Purple");
      this->line1->setVisible(true);
      this->line1->setVisibilityFlags(GZ_VISIBILITY_GUI);

      // 

    }

    void SomeVisualPlugin::draw_rectangle2(double x, double y, double width, double length)
    {
      double xy_ratio = 20;
      double x_min=(x-length/2)/xy_ratio;
      double x_max=(x+length/2)/xy_ratio;
      double y_min=(y+width/2)/xy_ratio;
      double y_max=(y-width/2)/xy_ratio;

      this->line2->Clear();
      this->line2->AddPoint(ignition::math::Vector3d(x_min,y_min,0));
      this->line2->AddPoint(ignition::math::Vector3d(x_min,y_max,0));
      this->line2->AddPoint(ignition::math::Vector3d(x_max,y_max,0));
      this->line2->AddPoint(ignition::math::Vector3d(x_max,y_min,0));
      this->line2->AddPoint(ignition::math::Vector3d(x_min,y_min,0));

      this->line2->setMaterial("Gazebo/Purple");
      this->line2->setVisibilityFlags(GZ_VISIBILITY_GUI);
      this->visual_->SetVisible(true);
      
      // set the Material of the line, in this case to purple
    }

    void SomeVisualPlugin::draw_rectangle3(double x, double y, double width, double length)
    {
      double xy_ratio = 20;
      double x_min=(x-length/2)/xy_ratio;
      double x_max=(x+length/2)/xy_ratio;
      double y_min=(y+width/2)/xy_ratio;
      double y_max=(y-width/2)/xy_ratio;

      this->line3->Clear();
      this->line3->AddPoint(ignition::math::Vector3d(x_min,y_min,0));
      this->line3->AddPoint(ignition::math::Vector3d(x_min,y_max,0));
      this->line3->AddPoint(ignition::math::Vector3d(x_max,y_max,0));
      this->line3->AddPoint(ignition::math::Vector3d(x_max,y_min,0));
      this->line3->AddPoint(ignition::math::Vector3d(x_min,y_min,0));

      this->line3->setMaterial("Gazebo/Purple");
      this->line3->setVisibilityFlags(GZ_VISIBILITY_GUI);
      this->visual_->SetVisible(true);
      
      // set the Material of the line, in this case to purple
    }

    void SomeVisualPlugin::draw_rectangle4(double x, double y, double width, double length)
    {
      double xy_ratio = 20;
      double x_min=(x-length/2)/xy_ratio;
      double x_max=(x+length/2)/xy_ratio;
      double y_min=(y+width/2)/xy_ratio;
      double y_max=(y-width/2)/xy_ratio;

      this->line4->Clear();
      this->line4->AddPoint(ignition::math::Vector3d(x_min,y_min,0));
      this->line4->AddPoint(ignition::math::Vector3d(x_min,y_max,0));
      this->line4->AddPoint(ignition::math::Vector3d(x_max,y_max,0));
      this->line4->AddPoint(ignition::math::Vector3d(x_max,y_min,0));
      this->line4->AddPoint(ignition::math::Vector3d(x_min,y_min,0));

      this->line4->setMaterial("Gazebo/Purple");
      this->line4->setVisibilityFlags(GZ_VISIBILITY_GUI);
      this->visual_->SetVisible(true);
      
      // set the Material of the line, in this case to purple
    }








    
    void SomeVisualPlugin::Load( VisualPtr _parent, sdf::ElementPtr _sdf )
    {
      cout<<"I am here1!!!!"<<endl;
      cout<<"I am here1!!!!"<<endl;
      cout<<"I am here1!!!!"<<endl;
      cout<<"I am here1!!!!"<<endl;
      cout<<"I am here1!!!!"<<endl;
      this->visual_ = _parent;

      this->visual_namespace_ = "visual/";
      this->visited_visual_namespace_ = "visual";

      // start ros node
      if (!ros::isInitialized())
      {
        int argc = 0;
        char** argv = NULL;
        ros::init(argc,argv,"gazebo_visual",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
      
        cout<<"I am here error0!!!"<<endl;
        cout<<"I am here error0!!!"<<endl;
        cout<<"I am here error0!!!"<<endl;
        cout<<"I am here error0!!!"<<endl;
      }

      this->rosnode_ = new ros::NodeHandle(this->visited_visual_namespace_);

      this->force_sub_ = this->rosnode_->subscribe<std_msgs::Float64MultiArray>("/slider_gazebo/zmp_foothold",1,&SomeVisualPlugin::VisualizeForceOnLink,this);

      this->update_connection_ = event::Events::ConnectRender(
          boost::bind(&SomeVisualPlugin::UpdateChild, this));

      this->line1 = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP);
      this->line2 = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP);
      this->line3 = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP);
      this->line4 = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP);

      // this->draw_rectangle1(step1x,step1y,0.14,0.22);
      // this->draw_rectangle2(step2x,step2y,0.14,0.22);
        
      // this->draw_rectangle3(step3x,step3y,0.14,0.22);
      // this->draw_rectangle4(step4x,step4y,0.14,0.22);
        
    }

    //////////////////////////////////////////////////////////////////////////////////
    // Update the visualizer
    void SomeVisualPlugin::UpdateChild()
    {
      
      ros::spinOnce();
    }

    //////////////////////////////////////////////////////////////////////////////////
    // VisualizeForceOnLink

    void SomeVisualPlugin::VisualizeForceOnLink(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
      step1x=msg->data[2];
      step1y=msg->data[3];

      step2x=msg->data[9];
      step2y=msg->data[10];

      step3x=msg->data[11];
      step3y=msg->data[12];

      step4x=msg->data[13];
      step4y=msg->data[14];

      count++;

      // if(count==(int)(max_count/2))
      // {
      //   this->visual_->DeleteDynamicLine(this->line1);
      //   this->line1 = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP);
      //   this->draw_rectangle1(step1x,step1y,0.14,0.22);
      // }


      if(count>max_count)
      {
        count = 0;
        
        
        // this->visual_->DeleteDynamicLine(this->line1);
        // this->visual_->DeleteDynamicLine(this->line2);
        // this->visual_->DeleteDynamicLine(this->line3);
        // this->visual_->DeleteDynamicLine(this->line4);
        // //this->visual_->Update();

        // this->line1 = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP);
        // this->line2 = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP);
        // this->line3 = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP);
        // this->line4 = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP);



        this->draw_rectangle1(step1x,step1y,0.14,0.22);
        this->draw_rectangle2(step2x,step2y,0.14,0.22);
        
        this->draw_rectangle3(step3x,step3y,0.14,0.22);
        this->draw_rectangle4(step4x,step4y,0.14,0.22);
        
      }

      
      // set the Material of the line, in this case to purple
    }

    // Register this plugin within the simulator
    GZ_REGISTER_VISUAL_PLUGIN(SomeVisualPlugin)
  }
  
}