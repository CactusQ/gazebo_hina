#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <ros/ros.h>


#include "modify_terrain.h"
#include "shared_constants.h"

using namespace gazebo;

class SoilTerrainModel : public ModelPlugin
{
    public:
        void Load(physics::ModelPtr parent, sdf::ElementPtr /*sdf*/)
        {
            this->model = parent;
            this->onUpdateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&SoilTerrainModel::onUpdate, this));


            node = transport::NodePtr(new transport::Node());     
            node->Init(model->GetWorld()->Name());
            transport::run();
            heightUpdatesPub = node->Advertise<msgs::Vector3d>("/gazebo/gazebo_hina/new_height");

            pluginLoadTime = common::Time::GetWallTime();
            contactManager = model->GetWorld()->Physics()->GetContactManager();
            contactManager->Init(model->GetWorld());
            contactManager->SetNeverDropContacts(false);

            auto collision = model->GetLink("terrain-link")->GetCollision("collision");
            heightMapShape = boost::dynamic_pointer_cast<physics::HeightmapShape>(collision->GetShape());
            gzlog << "SoilTerrainModel plugin: heightmap shape ["
                << heightMapShape->VertexCount().X() << ", " << heightMapShape->VertexCount().Y() << "]" << std::endl;
            gzlog << "SoilTerrainModel: successfully loaded!" << std::endl;

            /* Update Visual
            msgs::Visual visualMsg;
            visualPub = this->node->Advertise<msgs::Visual>("~/visual", 10);

            // Set the visual's name. This should be unique.
            visualMsg.set_name("terrain-visual");

            // Set the visual's parent. This visual will be attached to the parent
            visualMsg.set_parent_name(_parent->GetScopedName());

            // Create a cylinder
            msgs::Geometry *geomMsg = visualMsg.mutable_geometry();
            geomMsg->set_type(msgs::Geometry::CYLINDER);
            geomMsg->mutable_cylinder()->set_radius(1);
            geomMsg->mutable_cylinder()->set_length(.1);

            // Set the material to be bright red
            visualMsg.mutable_material()->mutable_script()->set_name(
                "Gazebo/RedGlow");

            // Set the pose of the visual relative to its parent
            msgs::Set(visualMsg.mutable_pose(),
                ignition::math::Pose3d(0, 0, 0.6, 0, 0, 0));

            // Don't cast shadows
            visualMsg.set_cast_shadows(false);

            visPub->Publish(visualMsg);
            */
        }

        void onUpdate()
        {
            if (gazebo::common::Time::GetWallTime().sec - pluginLoadTime.sec < SharedConstants::WARM_UP_PERIOD_IN_SECONDS)
                return;

            gazebo::msgs::PointCloud heightUpdatesMsg;
            std::cout << "Contacts: "  <<  contactManager->GetContacts().size() << std::endl;

            int positions = 0;
            for (auto &contact : contactManager->GetContacts())
            { 
                if (contact->collision1->GetModel() == model || contact->collision2->GetModel() == model)  
                    positions += contact->count;     
            }
            std::cout << "Positions: "  <<  positions << std::endl;
            for (auto &contact : contactManager->GetContacts())
            {          
                if (contact->collision1->GetModel() == model || contact->collision2->GetModel() == model)
                {
                    physics::Collision *terrainCollision = contact->collision1->GetModel() == model ? contact->collision1 : contact->collision2;
                    physics::Collision *wheelCollision = contact->collision1->GetModel() == model ? contact->collision2 : contact->collision1;

                    double worldX  = wheelCollision->WorldPose().Pos().X();
                    double worldY  = wheelCollision->WorldPose().Pos().Y();

                    // coordinate transform from regular Word (x,y) to the HeightmapShape (index_x,index_y)
                    // source: https://answers.gazebosim.org//question/17167/how-to-get-the-terrain-elevation-z-at-specific-xy-location/
                    auto size = this->heightMapShape->Size(); 
                    auto vc = this->heightMapShape->VertexCount();

                    int indexX = (((worldX + size.X()/2)/size.X())*vc.X()-1);
                    int indexY = (((-worldY + size.Y()/2)/size.Y())*vc.Y()-1);

                    double newHeight =  this->heightMapShape->GetHeight(indexX, indexY) - 0.0005; // TODO CHANGE
                    this->heightMapShape->SetHeight(indexX, indexY, newHeight); 
                    //std::cout << "(" << worldX << " " << worldY << ") / (" << indexX << " " << indexY  << ") = " << newHeight << std::endl;

                    gazebo::msgs::Vector3d newHeightMsg;
                    newHeightMsg.set_x(indexX);
                    newHeightMsg.set_y(indexY);
                    newHeightMsg.set_z(newHeight);                
                    //heightUpdatesMsg.add_points(newHeightMsg);
                    heightUpdatesPub->Publish(newHeightMsg);

                    // Get child link
                    // link->add force
                }
            }
            if (heightUpdatesMsg.points_size() != 0)
            {
            }
        }

    private:
        ros::Publisher 
        transport::NodePtr node;
        physics::HeightmapShapePtr heightMapShape;
        transport::PublisherPtr heightUpdatesPub;
        transport::PublisherPtr visualPub;

        physics::ModelPtr model;
        gazebo::physics::ContactManager *contactManager;

        event::ConnectionPtr onUpdateConnection;
        common::Time pluginLoadTime;
    };

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SoilTerrainModel)