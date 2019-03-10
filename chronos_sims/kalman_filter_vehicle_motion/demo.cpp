// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Demonstration of a steering path-follower PID controller.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "demo_data.h"
#include "chrono_models/vehicle/sedan/Sedan.h"

using namespace chrono::vehicle::sedan;

namespace demo_data
{

#ifdef USE_PID
    ChDriverSelector::ChDriverSelector(const ChVehicle& vehicle, ChPathFollowerDriver* driver_follower, ChIrrGuiDriver* driver_gui)
        :
          m_vehicle(vehicle),
          m_driver_follower(driver_follower),
          m_driver_gui(driver_gui),
          m_driver(m_driver_follower),
          m_using_gui(false)
    {}
#else
    ChDriverSelector::ChDriverSelector(const ChVehicle& vehicle, ChPathFollowerDriverXT* driver_follower, ChIrrGuiDriver* driver_gui)
        :
           m_using_gui(false),
           m_vehicle(vehicle),
           m_driver_follower(driver_follower),
           m_driver_gui(driver_gui),
           m_driver(m_driver_follower)
    {}
#endif


    bool ChDriverSelector::OnEvent(const irr::SEvent& event) {

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
}


using namespace demo_data;

int main(/*int argc, char* argv[]*/) {

    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    GetLog() <<" Data path is: "<<vehicle::GetDataPath()<<"\n";

    // ------------------------------
    // Create the vehicle and terrain
    // ------------------------------

    // Create the vehicle, set parameters, and initialize
    Sedan vehicle;
    vehicle.SetContactMethod(contact_method);
    vehicle.SetChassisFixed(false);
    vehicle.SetInitPosition(ChCoordsys<>(INIT_LOC, INIT_ROT));
    vehicle.SetTireType(TireModelType::RIGID);
    vehicle.SetTireStepSize(TIRE_DELTA_T);
    vehicle.SetVehicleStepSize(DELTA_T);
    vehicle.Initialize();

    vehicle.SetChassisVisualizationType(chassis_vis_type);
    vehicle.SetSuspensionVisualizationType(suspension_vis_type);
    vehicle.SetSteeringVisualizationType(steering_vis_type);
    vehicle.SetWheelVisualizationType(wheel_vis_type);
    vehicle.SetTireVisualizationType(tire_vis_type);

    // Create the terrain
    RigidTerrain terrain(vehicle.GetSystem());
    auto patch = terrain.AddPatch(ChCoordsys<>(ChVector<>(0, 0, TERRAIN_HEIGHT - 5), QUNIT),
                                  ChVector<>(TERRAIN_LENGTH, TERRAIN_WIDTH, 10));

    patch->SetContactFrictionCoefficient(0.8f);
    patch->SetContactRestitutionCoefficient(0.01f);
    patch->SetContactMaterialProperties(2e7f, 0.3f);
    patch->SetColor(ChColor(1, 1, 1));


    std::string texture_file = vehicle::GetDataFile(TERRAIN_PATH);
    GetLog()<<" Texture file is located at "<<texture_file<<"\n";
    patch->SetTexture(TERRAIN_PATH, 200, 200 );

    terrain.Initialize();

    // Create the Bezier path from data file
    GetLog()<<" Bezeir path is at: "<<PATH_FILE<<"\n";
    auto path = ChBezierCurve::read(PATH_FILE);
    GetLog()<<"Set up path file"<<"\n";

    // Create the vehicle Irrlicht application

    ChVehicleIrrApp app(&vehicle.GetVehicle(), &vehicle.GetPowertrain(), L"Sraight Line Vehicle Motion Demo", irr::core::dimension2d<irr::u32>(800, 640));
    app.SetHUDLocation(500, 20);
    app.SetSkyBox();
    app.AddTypicalLogo();
    app.AddTypicalLights(irr::core::vector3df(-150.f, -150.f, 200.f), irr::core::vector3df(-150.f, 150.f, 200.f), 100, 100);
    app.AddTypicalLights(irr::core::vector3df(150.f, -150.f, 200.f), irr::core::vector3df(150.0f, 150.f, 200.f), 100, 100);
    app.EnableGrid(false);
    app.SetChaseCamera(trackPoint, 6.0, 0.5);

    app.SetTimestep(DELTA_T);

    // Visualization of controller points (sentinel & target)
    irr::scene::IMeshSceneNode* ballS = app.GetSceneManager()->addSphereSceneNode(0.1f);
    irr::scene::IMeshSceneNode* ballT = app.GetSceneManager()->addSphereSceneNode(0.1f);
    ballS->getMaterial(0).EmissiveColor = irr::video::SColor(0, 255, 0, 0);
    ballT->getMaterial(0).EmissiveColor = irr::video::SColor(0, 0, 255, 0);

    // -------------------------
    // Create the driver systems
    // -------------------------

    // Create a GUI driver for interactive inputs
    ChIrrGuiDriver driver_gui(app);
    driver_gui.Initialize();

    ChPathFollowerDriverXT driver_follower( vehicle.GetVehicle(), path, "my_path", target_speed, vehicle.GetVehicle().GetMaxSteeringAngle());
    driver_follower.GetSteeringController().SetLookAheadDistance(5);
    driver_follower.GetSteeringController().SetGains(0.4, 1, 1, 1);
    driver_follower.GetSpeedController().SetGains(0.4, 0, 0);
    driver_follower.Initialize();

    // Create and register a custom Irrlicht event receiver to allow selecting the
    // current driver model.
    ChDriverSelector selector(vehicle.GetVehicle(), &driver_follower, &driver_gui);


    app.SetUserEventReceiver(&selector);

    // Finalize construction of visualization assets
    app.AssetBindAll();
    app.AssetUpdateAll();

    // -----------------
    // Initialize output
    // -----------------

    state_output = state_output || povray_output;

    if (state_output) {
        if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }

    if (povray_output) {
        if (ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
        driver_follower.ExportPathPovray(out_dir);
    }

    utils::CSV_writer csv("\t");
    csv.stream().setf(std::ios::scientific | std::ios::showpos);
    csv.stream().precision(6);

    utils::ChRunningAverage fwd_acc_GC_filter(filter_window_size);
    utils::ChRunningAverage lat_acc_GC_filter(filter_window_size);

    utils::ChRunningAverage fwd_acc_driver_filter(filter_window_size);
    utils::ChRunningAverage lat_acc_driver_filter(filter_window_size);

    // ---------------
    // Simulation loop
    // ---------------

    // Driver location in vehicle local frame
    ChVector<> driver_pos = vehicle.GetChassis()->GetLocalDriverCoordsys().pos;

    // Number of simulation steps between miscellaneous events
    double render_step_size = 1 / FPS;
    int render_steps = static_cast<int>(std::ceil(render_step_size / DELTA_T));
    double debug_step_size = 1 / debug_fps;
    int debug_steps = static_cast<int>(std::ceil(debug_step_size / DELTA_T));

    // Initialize simulation frame counter and simulation time
    ChRealtimeStepTimer realtime_timer;
    int sim_frame = 0;
    int render_frame = 0;

    while (app.GetDevice()->run()) {

        // Extract system state
        double time = vehicle.GetSystem()->GetChTime();
        ChVector<> acc_CG = vehicle.GetVehicle().GetChassisBody()->GetPos_dtdt();
        ChVector<> acc_driver = vehicle.GetVehicle().GetVehicleAcceleration(driver_pos);

        double fwd_acc_CG = fwd_acc_GC_filter.Add(acc_CG.x());
        double lat_acc_CG = lat_acc_GC_filter.Add(acc_CG.y());
        double fwd_acc_driver = fwd_acc_driver_filter.Add(acc_driver.x());
        double lat_acc_driver = lat_acc_driver_filter.Add(acc_driver.y());

        // End simulation
        if (time >= TIME_END)
            break;

        // Collect output data from modules (for inter-module communication)
        double throttle_input = selector.GetDriver()->GetThrottle();
        double steering_input = selector.GetDriver()->GetSteering();
        double braking_input  = selector.GetDriver()->GetBraking();

        // Update sentinel and target location markers for the path-follower controller.
        // Note that we do this whether or not we are currently using the path-follower driver.
        const ChVector<>& pS = driver_follower.GetSteeringController().GetSentinelLocation();
        ballS->setPosition(irr::core::vector3df(static_cast<irr::f32>(pS.x()),
                                                static_cast<irr::f32>(pS.y()),
                                                static_cast<irr::f32>(pS.z())));

        const ChVector<>& pT = driver_follower.GetSteeringController().GetTargetLocation();
        ballT->setPosition(irr::core::vector3df(static_cast<irr::f32>(pT.x()),
                                                static_cast<irr::f32>(pT.y()),
                                                static_cast<irr::f32>(pT.z())));

        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();

        // Output POV-Ray data
        if (sim_frame % render_steps == 0) {
            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteShapesPovray(vehicle.GetSystem(), filename);
            }

            if (state_output) {
                csv << time << steering_input << throttle_input << braking_input;
                csv << vehicle.GetVehicle().GetVehicleSpeed();
                csv << acc_CG.x() << fwd_acc_CG << acc_CG.y() << lat_acc_CG;
                csv << acc_driver.x() << fwd_acc_driver << acc_driver.y() << lat_acc_driver;
                csv << std::endl;
            }

            render_frame++;
        }

        // Debug logging
        //if (debug_output && sim_frame % debug_steps == 0) {

            ChVector<> pos = vehicle.GetVehicle().GetVehiclePos();
            double speed = vehicle.GetVehicle().GetVehicleSpeed();

            GetLog() << "Driver Acceleration:  " << acc_driver.x() << "  " << acc_driver.y() << "  " << acc_driver.z()
                     << "\n";
            GetLog() << "CG Acceleration:      " << acc_CG.x() << "  " << acc_CG.y() << "  " << acc_CG.z() << "\n";
            GetLog() << "\n";
            GetLog()<<"Vehicle Position: "<<pos.x()<<" "<<pos.y()<<" "<<pos.z()<<"\n";
            GetLog() << "\n";
            GetLog()<<"Vehicle Speed: "<<speed<<"\n";
        //}

        // Update modules (process inputs from other modules)
        driver_follower.Synchronize(time);
        driver_gui.Synchronize(time);
        terrain.Synchronize(time);
        vehicle.Synchronize(time, steering_input, braking_input, throttle_input, terrain);
        std::string msg = selector.UsingGUI() ? "GUI driver" : "Follower driver";
        app.Synchronize(msg, steering_input, throttle_input, braking_input);

        // Advance simulation for one timestep for all modules
        double step = realtime_timer.SuggestSimulationStep(DELTA_T);

        driver_follower.Advance(step);
        driver_gui.Advance(step);
        terrain.Advance(step);
        vehicle.Advance(step);
        app.Advance(step);

        // Increment simulation frame number
        sim_frame++;

        app.EndScene();
    }

    if (state_output)
        csv.write_to_file(out_dir + "/state.out");

    return 0;
}
