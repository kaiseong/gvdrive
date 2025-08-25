#include "GVdrive.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>

int main(int argc, char* argv[])
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <interface> <calib|control>\n";
        return -1;
    }

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("gv_package");
    std::string config_path = package_share_directory + "/config/motors.yaml";

    GVTerrain::robot::GVdrive gvdrive(argv[1], config_path);
    std::string mode = argv[2];

    if (mode == "calib"){
        gvdrive.setMode(GVTerrain::robot::UserMode::CALIBRATION);
    }
    else if (mode == "control"){ 
        gvdrive.setMode(GVTerrain::robot::UserMode::CONTROL);
    }
    else {
        std::cerr << "Unknown mode: " << mode << "\n";
        return -1;
    }

    if (!gvdrive.scan() || !gvdrive.con()) {
        std::cerr << "GVdrive scan/con failed\n";
        return -1;
    }

    /*********** user code start *************/
    uint16_t cnt=0;
    while (gvdrive.runEtherCatLoop) {
        if(cnt > 2000)
            break;
        // Torque Control
        gvdrive.setTargetTorque("TEST", 100);  // unit: Nm
        cnt++;

    }
    /*********** user code end *************/


    std::this_thread::sleep_for(std::chrono::seconds(1));
    gvdrive.shutdown();

    return 0;
}