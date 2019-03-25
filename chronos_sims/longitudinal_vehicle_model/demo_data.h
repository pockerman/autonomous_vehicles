#ifndef DEMO_DATA_H
#define DEMO_DATA_H

#include "chrono_vehicle/wheeled_vehicle/tire/ChPacejkaTire.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/sedan/Sedan_Vehicle.h"
#include "chrono_models/vehicle/sedan/Sedan_SimpleMapPowertrain.h"
#include "chrono_models/vehicle/sedan/Sedan_RigidTire.h"
#include "chrono_models/vehicle/sedan/Sedan_TMeasyTire.h"

#include <map>
#include <memory>
#include <vector>
#include <cmath>
#include <string>

namespace demo_data
{

// gravitational constant
const double G = 9.81;
double gravity(double mass, double alpha){

    return mass*G*std::sin((alpha));
}

/// The class that will model our vehicle
class Vehicle
{
public:

    //the type of the vehicle
    typedef chrono::vehicle::sedan::Sedan_Vehicle vehicle_t;

    /// \brief The type to the powertrain
    typedef chrono::vehicle::ChPowertrain powertrain_t;

    /// constructor
    Vehicle();

    /// initialize the vehicle
    void initialize();

    /// \brief Retruns the underlying pointer to the vehicle
    std::shared_ptr<vehicle_t> get_vehicle()const{return vehicle_;}

    /// \brief Retruns the underlying pointer to the powertrain
    std::shared_ptr<powertrain_t> get_powertrain()const{return powertrain_;}

private:

    /// pointers to vehicle, system, and powertrain
    std::shared_ptr<chrono::ChSystem> system_;
    std::shared_ptr<vehicle_t> vehicle_;
    std::shared_ptr<powertrain_t> powertrain_;
    chrono::ChMaterialSurface::ContactMethod contact_method_;
    chrono::vehicle::TireModelType tire_type_;
    chrono::ChCoordsys<> init_pos_;

    bool is_fixed_;
    bool apply_drag_;

    std::vector<double> init_omega_;

    // our car has 4 tires
    std::array<chrono::vehicle::ChTire*, 4> tires_;

    // some properties
    std::map<std::string, double> properties_;

};

//this class handles our simulation application
class SimApp
{

public:

    SimApp(const std::string& name);

private:

    const std::string name_;

};

}

#endif // DEMO_DATA_H
