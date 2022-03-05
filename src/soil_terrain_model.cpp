#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include "modify_terrain.h"
#include "shared_constants.h"

using namespace gazebo;

class SoilTerrainModel : public ModelPlugin
{
    public: void Load(physics::ModelPtr model, sdf::ElementPtr /*sdf*/)
    {
        gzlog << "SoilTerrainModel: successfully loaded!" << std::endl;

        this->model_ = model;

        this->on_update_connection_ = event::Events::ConnectPostRender(
            std::bind(&SoilTerrainModel::onUpdate, this));

        hole_drilled_ = false;
        plugin_load_time_ = gazebo::common::Time::GetWallTime();
        contactManager = model->GetWorld()->Physics()->GetContactManager();
        contactManager->Init(model->GetWorld());
        contactManager->SetNeverDropContacts(true);

        heightmap = getHeightmap();
        if (heightmap == nullptr)
        {
            gzerr << "SoilTerrainModel: Couldn't acquire heightmap!" << std::endl;
            return;
        }

        heightmap_shape = getHeightmapShape();
        if (heightmap_shape == nullptr)
        {
            gzerr << "SoilTerrainModel: Couldn't acquire heightmap shape!" << std::endl;
            return;
        }
    }

    rendering::Heightmap* getHeightmap()
    {
        auto scene = rendering::get_scene();
        if (!scene)
        {
            gzerr << "SoilTerrainModel: Couldn't acquire scene!" << std::endl;
            return nullptr;
        }

        auto heightmap = scene->GetHeightmap();
        if (heightmap == nullptr)
        {
            gzerr << "SoilTerrainModel: scene has no heightmap!" << std::endl;
            return nullptr;
        }
        return heightmap;
    }

    physics::HeightmapShapePtr getHeightmapShape()
    {
        if (model_ == nullptr)
        {
            gzerr << "SoilTerrainModel plugin: Couldn't acquire heightmap model!" << std::endl;
            return nullptr;
        }

        auto collision = model_->GetLink("terrain-link")->GetCollision("collision");
        if (collision == nullptr)
        {
            gzerr << "SoilTerrainModel plugin: Couldn't acquire heightmap model collision!" << std::endl;
            return nullptr;
        }
        
        auto shape = boost::dynamic_pointer_cast<physics::HeightmapShape>(collision->GetShape());
        if (shape == nullptr)
        {
            gzerr << "SoilTerrainModel plugin: Couldn't acquire heightmap model collision!" << std::endl;
            return nullptr;
        }            

        gzlog << "SoilTerrainModel plugin: heightmap shape ["
            << shape->VertexCount().X() << ", " << shape->VertexCount().Y() << "]" << std::endl;

        return shape;
    }

    static inline float getHeightInWorldCoords(const physics::HeightmapShapePtr& heightmap_shape, int x, int y)
    {
        auto value = heightmap_shape->GetHeight(x, heightmap_shape->VertexCount().Y() - y - 1);
        value += heightmap_shape->Pos().Z();
        return value;
    }

    static inline void setHeightFromWorldCoords(const physics::HeightmapShapePtr& heightmap_shape, int x, int y, float value)
    {
        value -= heightmap_shape->Pos().Z();
        heightmap_shape->SetHeight(x, heightmap_shape->VertexCount().Y() - y - 1, value);
    }

    void drillTerrainAt(double x, double y)
    {
        auto position_xy = Ogre::Vector3(x, y, 0);
        Ogre::Vector3 heightmap_position;
        auto terrain = heightmap->OgreTerrain()->getTerrain(0, 0);

        if (!terrain)
        {
            gzerr << "DynamicTerrain: Heightmap has no associated terrain object!" << std::endl;
            return;
        }

        terrain->getTerrainPosition(position_xy, &heightmap_position);
        /*

        ModifyTerrain::modify(heightmap, position_xy, 0.003, 0.002, 1.0, "lower",
            [&heightmap_shape](int x, int y) { return getHeightInWorldCoords(heightmap_shape, x, y); },
            [&heightmap_shape](int x, int y, float value) { setHeightFromWorldCoords(heightmap_shape, x, y, value); }
        );
        */

        hole_drilled_ = true;
        gzlog << "SoilTerrainModel: A hole has been drilled at ("
            << position_xy.x << ", " << position_xy.y << ")" << std::endl;
    }

    void onUpdate()
    {
        if (gazebo::common::Time::GetWallTime().sec - plugin_load_time_.sec < SharedConstants::WARM_UP_PERIOD_IN_SECONDS)
            return;

        for (auto &contact : contactManager->GetContacts())
        {          
          if (contact->collision1->GetModel() == model_ || contact->collision2->GetModel() == model_)
          {
            physics::Collision *terrainCollision = contact->collision1->GetModel() == model_ ? contact->collision1 : contact->collision2;
            physics::Collision *wheelCollision = contact->collision1->GetModel() == model_ ? contact->collision2 : contact->collision1;

            int x  = model_->WorldPose().Pos().X();
            int y  = model_->WorldPose().Pos().Y();
            drillTerrainAt(x, y);
            std::cout << x << " / " << y << std::endl;
          }
          else
          {
            std::cout << " No Update " << std::endl;
          }
        }
    }

private:
    rendering::Heightmap* heightmap;
    physics::HeightmapShapePtr heightmap_shape;

    physics::ModelPtr model_;
    gazebo::physics::ContactManager *contactManager;

    event::ConnectionPtr on_update_connection_;
    bool hole_drilled_;
    common::Time plugin_load_time_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SoilTerrainModel)