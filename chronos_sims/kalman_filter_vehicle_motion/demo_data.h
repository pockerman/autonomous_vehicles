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
//#include "chrono_models/vehicle/hmmwv/HMMWV.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::vehicle;
//using namespace chrono::vehicle::hmmwv;

namespace demo_data
{

    const std::string BASIC_DATA_PATH("/home/david/MyProjects/cubic_engine/chrono_lib/chrono/data/vehicle/paths/");

    // Rigid terrain dimensions
    const double TERRAIN_HEIGHT = 0;
    const double TERRAIN_LENGTH = 300.0;  // size in X direction
    const double TERRAIN_WIDTH = 300.0;   // size in Y direction
    const std::string TERRAIN_PATH("/home/david/MyProjects/cubic_engine/chrono_lib/chrono/data/vehicle/terrain/textures/tile4.jpg");

    // The extended steering controller only works inside the path limits
    // =============================================================================
    // Problem parameters

    // Contact method type
    const ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::SMC;


    // Steering type (PITMAN_ARM or PITMAN_ARM_SHAFTS)
    // Note: Compliant steering requires higher PID gains.
    const SteeringType steering_type = SteeringType::PITMAN_ARM;

    // Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
    const VisualizationType chassis_vis_type = VisualizationType::PRIMITIVES;
    const VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
    const VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
    const VisualizationType wheel_vis_type = VisualizationType::MESH;
    const VisualizationType tire_vis_type = VisualizationType::NONE;

    // Input file names for the path-follower driver model
    const std::string PATH_FILE(BASIC_DATA_PATH + "straight.txt");



    // Initial vehicle location and orientation
    const ChVector<> INIT_LOC(-125, -125, 0.5);
    const ChQuaternion<> INIT_ROT(1, 0, 0, 0);

    // Desired vehicle speed (m/s)
    double target_speed = 20;

    // Point on chassis tracked by the chase camera
    const ChVector<> trackPoint(0.0, 0.0, 1.75);

    // Simulation step size
    const double DELTA_T = 2e-3;
    const double TIRE_DELTA_T = 1e-3;

    // Simulation end time
    const double TIME_END = 100;

    // Render FPS
    const double FPS = 60;

    // Debug logging
    const bool debug_output = false;
    const double debug_fps = 10;

    // Output directories
    const std::string out_dir = GetChronoOutputPath() + "STEERING_CONTROLLER";
    const std::string pov_dir = out_dir + "/POVRAY";

    // POV-Ray output
    const bool povray_output = false;

    // Vehicle state output (forced to true if povray output enabled)
    bool state_output = false;
    const int filter_window_size = 20;

    // =============================================================================

    // Custom Irrlicht event receiver for selecting current driver model.
    class ChDriverSelector : public irr::IEventReceiver {
        public:

    #ifdef USE_PID
        ChDriverSelector(const ChVehicle& vehicle, ChPathFollowerDriver* driver_follower, ChIrrGuiDriver* driver_gui);
    #else
        ChDriverSelector(const ChVehicle& vehicle, ChPathFollowerDriverXT* driver_follower, ChIrrGuiDriver* driver_gui);
    #endif

        ChDriver* GetDriver() { return m_driver; }
        bool UsingGUI() const { return m_using_gui; }

        virtual bool OnEvent(const irr::SEvent& event);

      private:

        bool m_using_gui;
        const ChVehicle& m_vehicle;
    #ifdef USE_PID
        ChPathFollowerDriver* m_driver_follower;
    #else
        ChPathFollowerDriverXT* m_driver_follower;
    #endif
        ChIrrGuiDriver* m_driver_gui;
        ChDriver* m_driver;
    };

    // =============================================================================
}

#endif // DEMO_DATA_H
