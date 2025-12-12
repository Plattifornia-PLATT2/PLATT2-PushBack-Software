#include "platt2/menu/menu.hpp"
#include "liblvgl/lv_conf_internal.h"
#include "liblvgl/misc/lv_color.h"
#include "liblvgl/misc/lv_types.h"
#include "liblvgl/widgets/buttonmatrix/lv_buttonmatrix.h"
#include "platt2/EAutonConfig.hpp"
#include "platt2/ERobotConfig.hpp"

namespace platt2{
namespace menu{
// File location on PROS brain / USD (adjust if needed)
static constexpr const char* CONFIG_PATH = "/usd/config.txt";

// ---------- Color constants (hex) ----------
static const lv_color_t COLOR_PINK    = lv_color_hex(0xF02F63);
static const lv_color_t COLOR_PURPLE  = lv_color_hex(0x6647A5);
static const lv_color_t COLOR_JON     = lv_color_hex(0xF02F63);
static const lv_color_t COLOR_QUINN   = lv_color_hex(0x6647A5);
static const lv_color_t COLOR_COMP    = lv_color_hex(0xF02F63);
static const lv_color_t COLOR_SKILLS  = lv_color_hex(0x6647A5);
static const lv_color_t COLOR_SAVE    = lv_color_hex(0x4ABAC2);
static const lv_color_t BORDER_SELECTED = lv_color_white();
static const lv_color_t BORDER_DEFAULT  = lv_color_hex(0x555555);
static const lv_color_t COLOR_RED_ALLIANCE = lv_color_hex(0xD22630);
static const lv_color_t COLOR_BLUE_ALLIANCE = lv_color_hex(0x0077C8);

// ---------- Helper: set button style ----------
void RobotConfigMenu::applyColorTheme(lv_obj_t* btn, lv_color_t color) {
    // Main background
    lv_obj_set_style_bg_color(btn, color, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(btn, 12, LV_PART_MAIN);

    // Border default
    lv_obj_set_style_border_width(btn, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(btn, lv_color_black(), LV_PART_MAIN);

    // When checked, make background slightly lighter and border thicker
    lv_obj_set_style_bg_color(btn, lv_color_mix(color, lv_color_white(),245), LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_border_width(btn, 4, LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_border_color(btn, BORDER_SELECTED, LV_PART_MAIN | LV_STATE_CHECKED);
}

// ---------- Save / Load ----------
void RobotConfigMenu::saveConfig() {
    isInUse = false;
}

bool RobotConfigMenu::menuInUse() {
    return isInUse;
}

FullConfig RobotConfigMenu::getFullConfig() {
    return FullConfig{
        .autonConfig = static_cast<robot::AutonConfig>(auton_mode),
        .robotConfig = static_cast<robot::RobotConfig>(robot_config),
        .driverProfile = static_cast<robot::DriverProfile>(driver_profile)
    };
}

void RobotConfigMenu::loadConfig() {

}

void RobotConfigMenu::setButtonChecked(lv_obj_t* btn, bool checked, lv_color_t base_color) {
    if (checked) {
        // lighter background, thicker border
        lv_obj_set_style_bg_color(btn, lv_color_mix(base_color, lv_color_white(), 200), LV_PART_MAIN);
        lv_obj_set_style_border_width(btn, 4, LV_PART_MAIN);
        lv_obj_set_style_border_color(btn, BORDER_SELECTED, LV_PART_MAIN);
    } else {
        // base background, thin border
        lv_obj_set_style_bg_color(btn, base_color, LV_PART_MAIN);
        lv_obj_set_style_border_width(btn, 2, LV_PART_MAIN);
        lv_obj_set_style_border_color(btn, BORDER_DEFAULT, LV_PART_MAIN);
    }
}

// ---------- Update visual selected states ----------
void RobotConfigMenu::updateButtonStates() {
    // Robot color
    setButtonChecked(btn_pink,   robot_config == robot::PINK, COLOR_PINK);
    setButtonChecked(btn_purple, robot_config == robot::PURPLE, COLOR_PURPLE);

    // Driver profile
    setButtonChecked(btn_jon,    driver_profile == robot::JON, COLOR_JON);
    setButtonChecked(btn_quinn,  driver_profile == robot::QUINN, COLOR_QUINN);

    // Auton
    setButtonChecked(btn_comp,   auton_mode == robot::COMP_1, COLOR_COMP);
    setButtonChecked(btn_skills, auton_mode == robot::SKILLS_1, COLOR_SKILLS);

    setButtonChecked(btn_red,   alliance_color == robot::RED, COLOR_RED_ALLIANCE);
    setButtonChecked(btn_blue, alliance_color == robot::BLUE, COLOR_BLUE_ALLIANCE);
}

// ---------- Static callbacks ----------
void RobotConfigMenu::cbRobotConfig(lv_event_t* e) {
    // user_data holds the instance pointer
    RobotConfigMenu* self = static_cast<RobotConfigMenu*>(lv_event_get_user_data(e));
    lv_obj_t* target = static_cast<lv_obj_t*>(lv_event_get_target(e));
    if (target == self->btn_pink)   self->robot_config = robot::PINK;
    else                             self->robot_config = robot::PURPLE;
    self->updateButtonStates();
}

void RobotConfigMenu::cbDriverProfile(lv_event_t* e) {
    RobotConfigMenu* self = static_cast<RobotConfigMenu*>(lv_event_get_user_data(e));
    lv_obj_t* target = static_cast<lv_obj_t*>(lv_event_get_target(e));
    if (target == self->btn_jon)    self->driver_profile = robot::JON;
    else                             self->driver_profile = robot::QUINN;
    self->updateButtonStates();
}

void RobotConfigMenu::cbAutonMode(lv_event_t* e) {
    RobotConfigMenu* self = static_cast<RobotConfigMenu*>(lv_event_get_user_data(e));
    lv_obj_t* target = static_cast<lv_obj_t*>(lv_event_get_target(e));
    if (target == self->btn_comp)   self->auton_mode = robot::COMP_1;
    else                             self->auton_mode = robot::SKILLS_1;
    self->updateButtonStates();
}

void RobotConfigMenu::cbAlliance(lv_event_t* e) {
    RobotConfigMenu* self = static_cast<RobotConfigMenu*>(lv_event_get_user_data(e));
    lv_obj_t* target = static_cast<lv_obj_t*>(lv_event_get_target(e));
    if (target == self->btn_red)   self->alliance_color = robot::RED;
    else                             self->alliance_color = robot::BLUE;
    self->updateButtonStates();
}

void RobotConfigMenu::cbSave(lv_event_t* e) {
    RobotConfigMenu* self = static_cast<RobotConfigMenu*>(lv_event_get_user_data(e));
    self->updateButtonStates();
    self->saveConfig();
}

// ---------- Build the menu ----------
void RobotConfigMenu::build() {
    // Create/load screen
    lv_obj_t* scr = lv_screen_active();
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    // Use column flex on the screen to stack rows
    lv_obj_set_flex_flow(scr, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(scr, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(scr, 8, LV_PART_MAIN);

    // Helper to create a row container with horizontal flex
    auto makeRow = [&](int height) -> lv_obj_t* {
        lv_obj_t* row = lv_obj_create(scr);
        lv_obj_set_size(row, 480, height);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        return row;
    };

    // Title label (top)
    lv_obj_t* title = lv_label_create(scr);
    lv_label_set_text(title, "67 41 SIGMA OHIO RIZZ");
    lv_obj_set_style_text_font(title, LV_THEME_DEFAULT_FONT_NORMAL, 0);
    lv_obj_set_align(title, LV_ALIGN_TOP_MID);

    // --- Robot color row ---
    lv_obj_t* row1 = makeRow(32);
    lv_obj_remove_flag(row1, LV_OBJ_FLAG_SCROLLABLE);
    btn_pink = lv_button_create(row1);
    lv_obj_set_size(btn_pink, 220, 32);
    applyColorTheme(btn_pink, COLOR_PINK);
    lv_obj_add_event_cb(btn_pink, cbRobotConfig, LV_EVENT_CLICKED, this);
    lv_obj_t* lbl_pink = lv_label_create(btn_pink);
    lv_label_set_text(lbl_pink, "Pink");

    btn_purple = lv_button_create(row1);
    lv_obj_set_size(btn_purple, 220, 32);
    applyColorTheme(btn_purple, COLOR_PURPLE);
    lv_obj_add_event_cb(btn_purple, cbRobotConfig, LV_EVENT_CLICKED, this);
    lv_obj_t* lbl_purple = lv_label_create(btn_purple);
    lv_label_set_text(lbl_purple, "Purple");

    btn_red = lv_button_create(row1);
    lv_obj_set_size(btn_red, 220, 32);
    applyColorTheme(btn_red, COLOR_RED_ALLIANCE);
    lv_obj_add_event_cb(btn_red, cbAlliance, LV_EVENT_CLICKED, this);
    lv_obj_t* lbl_red = lv_label_create(btn_red);
    lv_label_set_text(lbl_red, "Red");

    // --- Driver profile row ---
    lv_obj_t* row2 = makeRow(48);
    lv_obj_remove_flag(row2, LV_OBJ_FLAG_SCROLLABLE);
    btn_jon = lv_button_create(row2);
    lv_obj_set_size(btn_jon, 220, 32);
    applyColorTheme(btn_jon, COLOR_JON);
    lv_obj_add_event_cb(btn_jon, cbDriverProfile, LV_EVENT_CLICKED, this);
    lv_obj_t* lbl_jon = lv_label_create(btn_jon);
    lv_label_set_text(lbl_jon, "Jon");

    btn_quinn = lv_button_create(row2);
    lv_obj_set_size(btn_quinn, 220, 32);
    applyColorTheme(btn_quinn, COLOR_QUINN);
    lv_obj_add_event_cb(btn_quinn, cbDriverProfile, LV_EVENT_CLICKED, this);
    lv_obj_t* lbl_quinn = lv_label_create(btn_quinn);
    lv_label_set_text(lbl_quinn, "Quinn");

    btn_blue = lv_button_create(row1);
    lv_obj_set_size(btn_blue, 220, 32);
    applyColorTheme(btn_blue, COLOR_BLUE_ALLIANCE);
    lv_obj_add_event_cb(btn_blue, cbAlliance, LV_EVENT_CLICKED, this);
    lv_obj_t* lbl_blue = lv_label_create(btn_blue);
    lv_label_set_text(lbl_blue, "Blue");

    // --- Auton row ---
    lv_obj_t* row3 = makeRow(48);
    lv_obj_remove_flag(row3, LV_OBJ_FLAG_SCROLLABLE);
    btn_comp = lv_button_create(row3);
    lv_obj_set_size(btn_comp, 220, 48);
    applyColorTheme(btn_comp, COLOR_COMP);
    lv_obj_add_event_cb(btn_comp, cbAutonMode, LV_EVENT_CLICKED, this);
    lv_obj_t* lbl_comp = lv_label_create(btn_comp);
    lv_label_set_text(lbl_comp, "Comp");

    btn_skills = lv_button_create(row3);
    lv_obj_set_size(btn_skills, 220, 48);
    applyColorTheme(btn_skills, COLOR_SKILLS);
    lv_obj_add_event_cb(btn_skills, cbAutonMode, LV_EVENT_CLICKED, this);
    lv_obj_t* lbl_skills = lv_label_create(btn_skills);
    lv_label_set_text(lbl_skills, "Skills");

    // --- Save / Load button ---
    btn_save = lv_button_create(scr);
    lv_obj_set_size(btn_save, 360, 48);
    applyColorTheme(btn_save, COLOR_SAVE);
    lv_obj_add_event_cb(btn_save, cbSave, LV_EVENT_CLICKED, this);
    lv_obj_t* lbl_save = lv_label_create(btn_save);
    lv_label_set_text(lbl_save, "Save / Load Config");

    // load persisted values (if present) and reflect them visually
    loadConfig();
    updateButtonStates();
}

}}