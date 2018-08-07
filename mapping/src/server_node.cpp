#include "map_server/map_reps/map_reps_ros.h"
#include "map_server/map_loader.h"



#define USAGE "\nUSAGE: map_server <map.yaml>\n" \
              "  map.yaml: map description file\n" \
              "DEPRECATED USAGE: map_server <map> <resolution>\n" \
              "  map: image file to load\n"\
              "  resolution: map resolution [meters/pixel]"



class MapServer: public MapServerROS
{
    public:
        MapServer(const std::string& fname, const std::string& map_type)
        {        
            ROS_INFO("Loading map from image \"%s\"", fname.c_str());
            ROS_INFO("Loading map representation: \"%s\"", map_type.c_str());

            // Here we try to load the map given user filename and desired map type
            try
            {
                MapLoader *m = new MapLoader(); 
                ROS_INFO("Create Map Object");
                BaseMapLoader* MyMap = m->createMap(map_type); 
                ROS_INFO("Load map info");
                MyMap->loadMapInfoFromFile(fname);
                std::string temp = MyMap->mapfname;
                ROS_INFO("Load Map: %s", temp.c_str());
                MyMap->loadMapFromFile(MyMap->mapfname);
                ROS_INFO("Set up Service");
                setResponse(MyMap->map);                            
                ROS_INFO("Set up Publisher");                    
                publishMap(MyMap->map);
                ROS_INFO("Published");

            }
            catch (std::runtime_error e)
            {
                ROS_ERROR("Cannot create map");
                exit(-1); 
            }

        // To make sure get a consistent time in simulation
        ros::Time::waitForValid();

        /* TODO: figure out this header stuff 
        
        map_resp_.map.info.map_load_time = ros::Time::now();
        map_resp_.map.header.frame_id = frame_id;
        map_resp_.map.header.stamp = ros::Time::now();
        
        */
        }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_server", ros::init_options::AnonymousName);
  if(argc != 3 && argc != 2)
  {
    ROS_ERROR("%s", USAGE);
    exit(-1);
  }

  std::string fname(argv[1]);
  std::string map_type = (argc ==2) ? "occupancy" : std::string(argv[2]);

  try
  {
    MapServer ms(fname, map_type);
    ros::spin();
  }
  catch(std::runtime_error& e)
  {
    ROS_ERROR("map_server exception: %s", e.what());
    return -1;
  }

  return 0;
}