#include "modify_terrain.h"
#include <gazebo/rendering/Conversions.hh>

using namespace std;
using namespace Ogre;
using namespace gazebo;
using namespace rendering;

void ModifyTerrain::modify(Heightmap* heightmap,
        Vector3 terrain_position,
        double outside_radius, double inside_radius, double weight,
        const string& op,
        function<float (long, long)> get_height_value,
        function<void (long, long, float)> set_height_value)
{
    auto terrain = heightmap->OgreTerrain()->getTerrain(0, 0);

    if (!terrain)
    {
        gzerr << "DynamicTerrain: Heightmap has no associated terrain object!" << endl;
        return;
    }

    Vector3 heightmap_position;
    terrain->getTerrainPosition(terrain_position, &heightmap_position);

    auto size = static_cast<int>(terrain->getSize());
    gzerr << "================================+" << std::endl;

    int x_start = terrain_position.x;
    int y_start = terrain_position.y;
    int padding = 6;

    for (int x = x_start-padding/2; x < x_start+padding; x++){
        for (int y = x_start-padding/2; y < x_start+padding; y++){
        
            auto added_height = 0.001;
            auto new_height = get_height_value(x, y);

            gzerr << "new height: " << x << " " << y << " " << new_height << std::endl;

            new_height += added_height;
            set_height_value(x, y, new_height);
        }
    }
}
