#pragma once

#include <string>
#include <vector>
#include <utility>

#include <dlfcn.h>

#include "RobotBase.h"
#include "RadarObj.h"

class Arena
{
public:
    explicit Arena(std::string config_path);

    void run();

private:
    std::string m_config_path;

    int m_height = 20;
    int m_width = 20;
    int m_max_rounds = 10000;
    double m_sleep_seconds = 0.5;
    bool m_game_state_live = true;
    int m_flamethrowers = 5;
    int m_pits = 5;
    int m_mounds = 5;

    std::vector<std::vector<char>> m_terrain; // base: '.','M','P','F'

    std::vector<RobotBase*> m_robots;
    std::vector<void*> m_robot_handles;

    int m_round = 1;
    /// Last arena “starting round …” banner value for this iteration (quiet summary).
    int m_result_round_echo = 1;

    void load_config();
    static std::pair<std::string, std::string> split_config_line(const std::string& line);
    bool parse_bool(const std::string& val) const;

    void allocate_terrain();
    void scatter_obstacle(char obstacle, int count);
    bool load_robots_from_disk();
    bool place_robots_random();

    char cell_visible_type(int row, int col, const RobotBase* skip_robot = nullptr) const;
    RobotBase* robot_at(int row, int col) const;
    RobotBase* sole_winner_if_any() const;

    std::vector<RadarObj> scan_radar(RobotBase& robot, int radar_direction) const;

    static int rnd_int(int low_inclusive, int high_inclusive);
    int apply_damage_with_armor(RobotBase& victim, int base_damage_low, int base_damage_high,
                                std::string& msg_out);

    void handle_shot(RobotBase& shooter, int shot_row, int shot_col, std::string& log_line);

    bool try_move_robot(RobotBase& robot, int direction, int raw_distance, std::string& log_line);

    void print_round_header() const;
    void print_arena() const;
    void cleanup_loaded();
};
