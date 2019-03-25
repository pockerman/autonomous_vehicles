#include "chrono/core/ChFileutils.h"
#include "chrono_vehicle/utils/ChVehicleIrrApp.h"
#include "demo_data.h"

#include <iostream>
#include <utility>

namespace demo_data
{

Vehicle::Vehicle()
    :
   system_(nullptr),
   vehicle_(nullptr),
   powertrain_(nullptr),
   contact_method_(chrono::ChMaterialSurface::SMC),
   tire_type_(chrono::vehicle::TireModelType::RIGID),
   init_pos_(),
   is_fixed_(false),
   apply_drag_(true),
   init_omega_(4, 0.0),
   tires_(),
   properties_()
{
    properties_.emplace(std::pair<std::string, double>("tire_mass",0.0));
    properties_.emplace(std::pair<std::string, double>("air_density",0.0));
    properties_.emplace(std::pair<std::string, double>("area",0.0));
    properties_.emplace(std::pair<std::string, double>("Cd",0.0));
    properties_.emplace(std::pair<std::string, double>("vehicle_step_size",0.0));
    properties_.emplace(std::pair<std::string, double>("tire_step_size",0.0));
    properties_.emplace(std::pair<std::string, double>("init_fwd_vel",0.0));
}

void
Vehicle::initialize(){

    // Create and initialize the Sedan vehicle
    using namespace chrono::vehicle;
    typedef chrono::vehicle::sedan::Sedan_Vehicle v_tp;
    vehicle_ = system_ ? std::make_shared<v_tp>(system_, is_fixed_, ChassisCollisionType::NONE)
                            : std::make_shared<v_tp>(is_fixed_, contact_method_, ChassisCollisionType::NONE);

    vehicle_->SetInitWheelAngVel(init_omega_);
    vehicle_->Initialize(init_pos_, properties_["init_fwd_vel"]);


    // Create and initialize the powertrain system
    //powertrain_ = new Sedan_SimpleMapPowertrain("Powertrain");
    //powertrain_->Initialize(GetChassisBody(), m_vehicle->GetDriveshaft());

    //bool use_mesh = (m_tireType == TireModelType::RIGID_MESH);
    //Sedan_RigidTire* tire_FL = new Sedan_RigidTire("FL", false);
    //Sedan_RigidTire* tire_FR = new Sedan_RigidTire("FR", false);
    //Sedan_RigidTire* tire_RL = new Sedan_RigidTire("RL", use_mesh);
    //Sedan_RigidTire* tire_RR = new Sedan_RigidTire("RR", use_mesh);

    //setup the tires
    tires_[0] = nullptr; //tire_FL;
    tires_[1] = nullptr; //tire_FR;
    tires_[2] = nullptr; //tire_RL;
    tires_[3] = nullptr; //tire_RR;
}


SimApp::SimApp(const std::string& name)
    :
      name_(name)
{}

}

int main()
{
    chrono::GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    chrono::GetLog() <<" Starting Simulation" <<"\n";

    demo_data::Vehicle vehicle;
    vehicle.initialize();

    //create the application
    // Create the vehicle Irrlicht application
    chrono::vehicle::ChVehicleIrrApp app(vehicle.get_vehicle().get(),
                                         vehicle.get_powertrain().get(),
                        L"Longitudinal Vehicle Model",
                        irr::core::dimension2d<irr::u32>(800, 640));

    app.SetHUDLocation(500, 20);
    app.SetSkyBox();
    app.AddTypicalLogo();
    app.AddTypicalLights(irr::core::vector3df(-150.f, -150.f, 200.f), irr::core::vector3df(-150.f, 150.f, 200.f), 100, 100);
    app.AddTypicalLights(irr::core::vector3df(150.f, -150.f, 200.f), irr::core::vector3df(150.0f, 150.f, 200.f), 100, 100);
    app.EnableGrid(false);
    app.SetChaseCamera(TRACK_POINT, 6.0, 0.5);


    chrono::GetLog() <<" End Simulation" <<"\n";
    return 0;
}
