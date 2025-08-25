#include "GVdrive.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>

int main(int argc, char* argv[])
{
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <interface> <calib|control> <noboard|raihub>\n";
        return -1;
    }

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("gv_package");
    std::string config_path = package_share_directory + "/config/motors.yaml";

    GVTerrain::robot::GVdrive gvdrive(argv[1], config_path);
    std::string control_mode = argv[2];
    std::string connect_mode = argv[3];

    // control mode set
    if (control_mode=="calib"){
        gvdrive.setControlMode(GVTerrain::robot::UserMode::CALIBRATION);
    }
    else if(control_mode=="control"){
        gvdrive.setControlMode(GVTerrain::robot::UserMode::CONTROL);
    }
    else{
        std::cerr << "Unknown control mode: " << control_mode << "\n";
        return -1;
    }

    // connect_mode
    if (connect_mode=="noboard"){
        gvdrive.setConnectMode(GVTerrain::robot::ConnectMode::NOBOARD);
    }
    else if (connect_mode=="raihub"){
        gvdrive.setConnectMode(GVTerrain::robot::ConnectMode::RAIHUB);
    }
    else{
        std::cerr << "Unknown connect mode: " << connect_mode << "\n";
        return -1;
    }

    if (!gvdrive.scan() || !gvdrive.con()) {
        std::cerr << "GVdrive scan/con failed\n";
        return -1;
    }

    /*********** user code start *************/
    uint16_t cnt=0;
    while (true) {
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