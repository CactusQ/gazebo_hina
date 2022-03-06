#include <gazebo/common/common.hh>
#include "modify_terrain.h"
#include "shared_constants.h"
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/rendering/Conversions.hh>

#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Heightmap.hh>
#include <OgreVector3.h>

using namespace gazebo;

class SoilTerrainVisual : public VisualPlugin
{
public:
    void Load(rendering::VisualPtr /*visual*/, sdf::ElementPtr /*sdf*/)
    {
        gzlog << "SoilTerrainVisual: successfully loaded!" << std::endl;

        this->on_update_connection_ = event::Events::ConnectPostRender(
            std::bind(&SoilTerrainVisual::onUpdate, this));

        node = transport::NodePtr(new transport::Node());   
        node->Init();
        transport::run();
        heightUpdatesSub = node->Subscribe("/gazebo/gazebo_hina/new_height", &SoilTerrainVisual::newHeightCallback, this);
        pluginLoadTime = gazebo::common::Time::GetWallTime();
    }

private:

    void newHeightCallback(ConstVector3dPtr &msg)
    {
        if (!sceneRendered) return;
        while (!scene)
        {
            scene = rendering::get_scene();
        }
        gzerr << "GOT SCENE" << std::endl;

        while (heightmap == nullptr)
        {
            heightmap = scene->GetHeightmap();
        }
        gzerr << "GOT HM" << std::endl;

        auto terrain = heightmap->OgreTerrain()->getTerrain(0, 0);

        gzerr << "GOT Terrain" << std::endl;

        //gzerr << "Get H: " << msg->x() << " " << msg->y() << " "  << terrain->getHeightAtPoint(msg->x(), msg->y()) << std::endl;

        auto position_xy = Ogre::Vector3(msg->x(),msg->y(), 0);
        ModifyTerrain::modify(heightmap, position_xy, 0.0003, 0.0002, 1.0, "lower",
                    [&terrain](long x, long y) { return terrain->getHeightAtPoint(x, y); },
                    [&terrain](long x, long y, float value) { terrain->setHeightAtPoint(x, y, value); }
        );

        //terrain->setHeightAtPoint(msg->x(), msg->y(), msg->z());
        gzerr << "Set H: " << msg->x() << " " << msg->y() << " "  << msg->z() << std::endl;

        terrain->updateGeometry();
        gzerr << "updateGeometry Geom" << std::endl;

        terrain->updateDerivedData(false,
            Ogre::Terrain::DERIVED_DATA_NORMALS | Ogre::Terrain::DERIVED_DATA_LIGHTMAP);
        gzerr << "updateGeometry DerivedData" << std::endl;

    }


    void onUpdate()
    {   
        if (gazebo::common::Time::GetWallTime().sec - pluginLoadTime.sec < SharedConstants::WARM_UP_PERIOD_IN_SECONDS)
            return;

        sceneRendered = true;
    }

private:
    gazebo::rendering::ScenePtr scene;
    gazebo::rendering::Heightmap* heightmap;
    transport::NodePtr node;
    transport::SubscriberPtr heightUpdatesSub;
    event::ConnectionPtr on_update_connection_;
    bool sceneRendered = false;
    common::Time pluginLoadTime;
};

GZ_REGISTER_VISUAL_PLUGIN(SoilTerrainVisual)