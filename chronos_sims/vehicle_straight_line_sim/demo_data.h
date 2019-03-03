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
#include "chrono_models/vehicle/hmmwv/HMMWV.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

namespace demo_data
{

    const std::string BASIC_DATA_PATH("/home/david/MyProjects/cubic_engine/chrono_lib/chrono/data/vehicle/paths/");

    // Rigid terrain dimensions
    const double terrainHeight = 0;
    const double terrainLength = 300.0;  // size in X direction
    const double terrainWidth = 300.0;   // size in Y direction

    // The extended steering controller only works inside the path limits
    // =============================================================================
    // Problem parameters

    // Contact method type
    const ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::SMC;

    // Type of tire model (RIGID, LUGRE, FIALA, PACEJKA, or TMEASY)
    const TireModelType tire_model = TireModelType::RIGID;

    // Type of powertrain model (SHAFTS or SIMPLE)
    const PowertrainModelType powertrain_model = PowertrainModelType::SHAFTS;

    // Drive type (FWD, RWD, or AWD)
    const DrivelineType drive_type = DrivelineType::RWD;

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
    const std::string path_file(BASIC_DATA_PATH + "straight.txt");


    // Initial vehicle location and orientation
    const ChVector<> initLoc(-125, -125, 0.5);
    const ChQuaternion<> initRot(1, 0, 0, 0);

    // Desired vehicle speed (m/s)
    double target_speed = 12;

    // Point on chassis tracked by the chase camera
    const ChVector<> trackPoint(0.0, 0.0, 1.75);

    // Simulation step size
    const double step_size = 2e-3;
    const double tire_step_size = 1e-3;

    // Simulation end time
    const double t_end = 100;

    // Render FPS
    double fps = 60;

    // Debug logging
    bool debug_output = false;
    double debug_fps = 10;

    // Output directories
    const std::string out_dir = GetChronoOutputPath() + "STEERING_CONTROLLER";
    const std::string pov_dir = out_dir + "/POVRAY";

    // POV-Ray output
    bool povray_output = false;

    // Vehicle state output (forced to true if povray output enabled)
    bool state_output = false;
    int filter_window_size = 20;

    // =============================================================================

    // Custom Irrlicht event receiver for selecting current driver model.
    class ChDriverSelector : public irr::IEventReceiver {
        public:

    #ifdef USE_PID
        ChDriverSelector(const ChVehicle& vehicle, ChPathFollowerDriver* driver_follower, ChIrrGuiDriver* driver_gui)
            : m_vehicle(vehicle),
              m_driver_follower(driver_follower),
              m_driver_gui(driver_gui),
              m_driver(m_driver_follower),
              m_using_gui(false) {}
    #else
        ChDriverSelector(const ChVehicle& vehicle, ChPathFollowerDriverXT* driver_follower, ChIrrGuiDriver* driver_gui)
            : m_vehicle(vehicle),
              m_driver_follower(driver_follower),
              m_driver_gui(driver_gui),
              m_driver(m_driver_follower),
              m_using_gui(false) {}
    #endif
        ChDriver* GetDriver() { return m_driver; }
        bool UsingGUI() const { return m_using_gui; }

        virtual bool OnEvent(const irr::SEvent& event) {
            // Only interpret keyboard inputs.
            if (event.EventType != irr::EET_KEY_INPUT_EVENT)
                return false;

            // Disregard key pressed
            if (event.KeyInput.PressedDown)
                return false;

            switch (event.KeyInput.Key) {
                case irr::KEY_COMMA:
                    if (m_using_gui) {
                        m_driver = m_driver_follower;
                        m_using_gui = false;
                    }
                    return true;
                case irr::KEY_PERIOD:
                    if (!m_using_gui) {
                        m_driver_gui->SetThrottle(m_driver_follower->GetThrottle());
                        m_driver_gui->SetSteering(m_driver_follower->GetSteering());
                        m_driver_gui->SetBraking(m_driver_follower->GetBraking());
                        m_driver = m_driver_gui;
                        m_using_gui = true;
                    }
                    return true;
                case irr::KEY_HOME:
                    if (!m_using_gui && !m_driver_follower->GetSteeringController().IsDataCollectionEnabled()) {
                        std::cout << "Data collection started at t = " << m_vehicle.GetChTime() << std::endl;
                        m_driver_follower->GetSteeringController().StartDataCollection();
                    }
                    return true;
                case irr::KEY_END:
                    if (!m_using_gui && m_driver_follower->GetSteeringController().IsDataCollectionEnabled()) {
                        std::cout << "Data collection stopped at t = " << m_vehicle.GetChTime() << std::endl;
                        m_driver_follower->GetSteeringController().StopDataCollection();
                    }
                    return true;
                case irr::KEY_INSERT:
                    if (!m_using_gui && m_driver_follower->GetSteeringController().IsDataAvailable()) {
                        char filename[100];
                        sprintf(filename, "controller_%.2f.out", m_vehicle.GetChTime());
                        std::cout << "Data written to file " << filename << std::endl;
                        m_driver_follower->GetSteeringController().WriteOutputFile(std::string(filename));
                    }
                    return true;
                default:
                    break;
            }

            return false;
        }

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
