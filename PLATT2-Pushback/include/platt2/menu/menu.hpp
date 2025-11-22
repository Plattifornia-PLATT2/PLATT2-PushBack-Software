#ifndef MENU_HPP
#define MENU_HPP

#include "platt2/EAllianceConfig.hpp"
#include "platt2/EAutonConfig.hpp"
#include "platt2/ERobotConfig.hpp"
#include "platt2/EDriverConfig.hpp"
#include "liblvgl/lvgl.h"
#include "platt2/robot/Robot.hpp"
#include <stdio.h>

namespace platt2{
namespace menu{
    struct FullConfig{
    robot::AutonConfig autonConfig;
    robot::RobotConfig robotConfig;
    robot::DriverProfile driverProfile;
};

class RobotConfigMenu {
public:
    RobotConfigMenu() = default;
    // build UI and load config
    void build();

    // getters for other code (opcontrol/auton)
    int getRobotConfig() const { return robot_config; }    // 0 Pink, 1 Purple
    int getDriverProfile() const { return driver_profile; } // 0 Jon, 1 Quinn
    int getAutonMode() const { return auton_mode; }         // 0 Comp, 1 Skills
    
    bool menuInUse();

    FullConfig getFullConfig();

private:
    // stored values
    robot::RobotConfig robot_config = robot::NO_ROBOT;
    robot::DriverProfile driver_profile = robot::NO_DRIVER;
    robot::AutonConfig auton_mode = robot::NO_AUTON;

    // LVGL objects
    lv_obj_t* btn_pink   = nullptr;
    lv_obj_t* btn_purple = nullptr;
    lv_obj_t* btn_jon    = nullptr;
    lv_obj_t* btn_quinn  = nullptr;
    lv_obj_t* btn_comp   = nullptr;
    lv_obj_t* btn_skills = nullptr;
    lv_obj_t* btn_save   = nullptr;

    // helpers
    void saveConfig();
    void loadConfig();
    void setButtonChecked(lv_obj_t* btn, bool checked, lv_color_t base_color);
    void updateButtonStates();
    void applyColorTheme(lv_obj_t* btn, lv_color_t color);
    void loadStartMenu();

    // static event callbacks (forward to instance via user_data)
    static void cbRobotConfig(lv_event_t* e);
    static void cbDriverProfile(lv_event_t* e);
    static void cbAutonMode(lv_event_t* e);
    static void cbSave(lv_event_t* e);

    bool isInUse = true;

};

}}
#endif