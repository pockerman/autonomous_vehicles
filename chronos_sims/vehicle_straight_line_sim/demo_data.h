#ifndef DEMO_DATA_H
#define DEMO_DATA_H

#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"


using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::vehicle;

namespace demo_data
{

const std::string BASIC_DATA_PATH("/home/david/MyProjects/cubic_engine/chrono_lib/chrono/data/vehicle/paths/");

// Rigid terrain dimensions
const double TERRAIN_HEIGHT = 0;
const double TERRAIN_LENGTH = 300.0;  // size in X direction
const double TERRAIN_WIDTH = 300.0;   // size in Y direction
const std::string TERRAIN_PATH("/home/david/MyProjects/cubic_engine/chrono_lib/chrono/data/vehicle/terrain/textures/tile4.jpg");

// Input file names for the path-follower driver model
const std::string PATH_FILE(BASIC_DATA_PATH + "straight.txt");

// Initial vehicle location and orientation
const ChVector<> INIT_LOC(-125, -125, 0.5);
const ChQuaternion<> INIT_ROT(1, 0, 0, 0);

// Desired vehicle speed (m/s)
double target_speed = 20;

// Point on chassis tracked by the chase camera
const ChVector<> TRACK_POINT(0.0, 0.0, 1.75);

// Simulation step size
const double DELTA_T = 2e-3;
const double TIRE_DELTA_T = 1e-3;

// Simulation end time
const double TIME_END = 100;

// Render FPS
const double FPS = 60;

// Debug logging
const bool DEBUG_OUTPUT = false;
const double DEBUG_FPS = 10;

// Output directories
const std::string OUT_DIR = GetChronoOutputPath() + "STEERING_CONTROLLER";
const std::string POV_DIR = OUT_DIR + "/POVRAY";

// POV-Ray output
const bool POVRAY_OUTPUT = false;

// Vehicle state output (forced to true if povray output enabled)
bool state_output = false;
const int FILTER_WINDOW_SIZE = 20;


// Custom Irrlicht event receiver for selecting current driver model.
class ChDriverSelector : public irr::IEventReceiver {
  public:

    ChDriverSelector(const ChVehicle& vehicle, ChPathFollowerDriverXT* driver_follower, ChIrrGuiDriver* driver_gui);

    ChDriver* GetDriver() { return m_driver; }
    bool UsingGUI() const { return m_using_gui; }

    virtual bool OnEvent(const irr::SEvent& event);

  private:

    bool m_using_gui;
    const ChVehicle& m_vehicle;

    ChPathFollowerDriverXT* m_driver_follower;
    ChIrrGuiDriver* m_driver_gui;
    ChDriver* m_driver;
};
}

#endif // DEMO_DATA_H
