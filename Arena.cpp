#include "Arena.h"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <cmath>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace fs = std::filesystem;

namespace {
void trim(std::string& s)
{
    const auto a = s.find_first_not_of(" \t\r\n\f\v");
    if (a == std::string::npos) {
        s.clear();
        return;
    }
    const auto b = s.find_last_not_of(" \t\r\n\f\v");
    s = s.substr(a, b - a + 1);
}

bool greedy_step(int sr, int sc, int tr, int tc, int& dr, int& dc)
{
    const int dx = tc - sc;
    const int dy = tr - sr;
    if (dx == 0 && dy == 0) {
        return false;
    }
    const int ax = std::abs(dx);
    const int ay = std::abs(dy);
    dr = dc = 0;
    if (ay > ax) {
        dr = (dy > 0) - (dy < 0);
    } else if (ax > ay) {
        dc = (dx > 0) - (dx < 0);
    } else {
        dr = (dy > 0) - (dy < 0);
        dc = (dx > 0) - (dx < 0);
    }
    return true;
}
bool first_step_in_bounds(int sr, int sc, int dr, int dc, int H, int W)
{
    const int nr = sr + dr;
    const int nc = sc + dc;
    return nr >= 0 && nr < H && nc >= 0 && nc < W;
}

void extend_ray_line(int sr, int sc, int dr, int dc, int H, int W, std::vector<std::pair<int, int>>& out)
{
    for (int r = sr, c = sc;;) {
        r += dr;
        c += dc;
        if (r < 0 || r >= H || c < 0 || c >= W) {
            break;
        }
        out.emplace_back(r, c);
    }
}

} // namespace
Arena::Arena(std::string config_path)
    : m_config_path(std::move(config_path))
{
}

void Arena::cleanup_loaded()
{
    for (RobotBase* bot : m_robots) {
        delete bot;
    }
    for (void* h : m_robot_handles) {
        if (h != nullptr) {
            dlclose(h);
        }
    }
    m_robots.clear();
    m_robot_handles.clear();
}

std::pair<std::string, std::string> Arena::split_config_line(const std::string& line_raw)
{
    const auto hash_pos = line_raw.find_first_of('#');
    const std::string line =
        hash_pos == std::string::npos ? line_raw : line_raw.substr(0, hash_pos);
    const auto colon_pos = line.find(':');
    if (colon_pos == std::string::npos) {
        return {"", ""};
    }
    std::string key = line.substr(0, colon_pos);
    std::string val = line.substr(colon_pos + 1);
    trim(key);
    trim(val);
    return {key, val};
}

bool Arena::parse_bool(const std::string& val) const
{
    std::string v = val;
    std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return v == "true" || v == "1" || v == "yes";
}

void Arena::load_config()
{
    std::ifstream fin(m_config_path);
    if (!fin) {
        throw std::runtime_error("Arena: could not open config file " + m_config_path);
    }
    std::string row;
    while (std::getline(fin, row)) {
        const auto [key, val] = split_config_line(row);
        if (key.empty()) {
            continue;
        }
        std::istringstream ins(val);
        if (key == "Arena_Size") {
            ins >> m_height >> m_width;
        } else if (key == "Max_Rounds") {
            ins >> m_max_rounds;
        } else if (key == "Sleep_interval") {
            ins >> m_sleep_seconds;
        } else if (key == "Game_State_Live") {
            std::string s;
            ins >> s;
            m_game_state_live = parse_bool(!s.empty() ? s : val);
        } else if (key == "Flamethrowers") {
            ins >> m_flamethrowers;
        } else if (key == "Pits") {
            ins >> m_pits;
        } else if (key == "Mounds") {
            ins >> m_mounds;
        }
    }
    if (m_height < 3 || m_width < 3) {
        throw std::runtime_error("Arena_Size must specify reasonable Height Width");
    }
}

void Arena::allocate_terrain()
{
    m_terrain.assign(static_cast<std::size_t>(m_height),
                    std::vector<char>(static_cast<std::size_t>(m_width), '.'));
}

void Arena::scatter_obstacle(char obstacle, int count)
{
    std::mt19937 rng{std::random_device{}()};
    std::uniform_int_distribution<int> dr{0, m_height - 1};
    std::uniform_int_distribution<int> dc{0, m_width - 1};
    int placed = 0;
    for (int attempts = 0; placed < count && attempts < 10000; ++attempts) {
        const int r = dr(rng);
        const int c = dc(rng);
        if (m_terrain[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)] == '.') {
            m_terrain[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)] = obstacle;
            ++placed;
        }
    }
    if (placed < count) {
        std::cerr << "Warning: could only place " << placed << " of obstacle '" << obstacle
                  << "' (requested " << count << ").\n";
    }
}

RobotBase* Arena::robot_at(int row, int col) const
{
    for (RobotBase* bot : m_robots) {
        int br = 0;
        int bc = 0;
        bot->get_current_location(br, bc);
        if (br == row && bc == col) {
            return bot;
        }
    }
    return nullptr;
}

char Arena::cell_visible_type(int row, int col, const RobotBase* skip_robot) const
{
    if (row < 0 || col < 0 || row >= m_height || col >= m_width) {
        return '?';
    }
    RobotBase* at = robot_at(row, col);
    if (at != nullptr && at != skip_robot) {
        return at->get_health() > 0 ? 'R' : 'X';
    }
    return m_terrain[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)];
}

RobotBase* Arena::sole_winner_if_any() const
{
    RobotBase* last = nullptr;
    int hits = 0;
    for (RobotBase* bot : m_robots) {
        if (bot->get_health() <= 0) {
            continue;
        }
        last = bot;
        if (++hits > 1) {
            return nullptr;
        }
    }
    return (hits == 1) ? last : nullptr;
}

std::vector<RadarObj> Arena::scan_radar(RobotBase& robot, const int radar_direction) const
{
    std::vector<RadarObj> out;
    int sr = 0;
    int sc = 0;
    robot.get_current_location(sr, sc);

    if (radar_direction == 0) {
        constexpr int deltas[8][2] = {
            {-1, 0}, {-1, 1}, {0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}};
        for (const auto* d : deltas) {
            const int rr = sr + d[0];
            const int cc = sc + d[1];
            if (rr < 0 || cc < 0 || rr >= m_height || cc >= m_width) {
                continue;
            }
            out.emplace_back(cell_visible_type(rr, cc, &robot), rr, cc);
        }
        return out;
    }
    if (radar_direction < 1 || radar_direction > 8) {
        return out;
    }

    const int dr_step = directions[radar_direction].first;
    const int dc_step = directions[radar_direction].second;
    const int prep_r = -dc_step;
    const int prep_c = dr_step;
    for (int step = 1;; ++step) {
        bool progress = false;
        const int base_r = sr + dr_step * step;
        const int base_c = sc + dc_step * step;
        for (int k = -1; k <= 1; ++k) {
            const int rr = base_r + k * prep_r;
            const int cc = base_c + k * prep_c;
            if (rr < 0 || cc < 0 || rr >= m_height || cc >= m_width) {
                continue;
            }
            progress = true;
            out.emplace_back(cell_visible_type(rr, cc, &robot), rr, cc);
        }
        if (!progress) {
            break;
        }
    }
    return out;
}

bool Arena::load_robots_from_disk()
{
    static constexpr const char kRobotsDir[] = "robots";
    if (!fs::exists(kRobotsDir) || !fs::is_directory(kRobotsDir)) {
        std::cerr << "Missing 'robots' directory alongside the arena executable.\n";
        return false;
    }
    std::vector<fs::path> cpp_paths;
    for (const fs::directory_entry& ent : fs::directory_iterator{kRobotsDir}) {
        if (!ent.is_regular_file()) {
            continue;
        }
        const std::string name = ent.path().filename().string();
        constexpr std::string_view pref = "Robot_";
        if (name.size() <= pref.size() + 4 || name.compare(0, pref.size(), pref) != 0
            || name.substr(name.size() - 4) != ".cpp") {
            continue;
        }
        bool bad = false;
        for (char ch : name) {
            const unsigned char u = static_cast<unsigned char>(ch);
            if (!(std::isalnum(u) || u == '_' || u == '-' || u == '.')) {
                bad = true;
                break;
            }
        }
        if (!bad) {
            cpp_paths.push_back(ent.path());
        }
    }
    std::sort(cpp_paths.begin(), cpp_paths.end());
    if (cpp_paths.empty()) {
        std::cerr << "No Robot_*.cpp files found under 'robots/'.\n";
        return false;
    }
    for (const fs::path& cpp_path : cpp_paths) {
        const std::string cpp_str = cpp_path.string();
        const std::string fname = cpp_path.filename().string();
        const std::string so_relative = std::string("robots/lib") + cpp_path.stem().string() + ".so";
        const fs::path so_path(so_relative);
        bool need_compile = true;
        try {
            if (fs::exists(so_path) && fs::exists(cpp_path)) {
                need_compile = fs::last_write_time(cpp_path) > fs::last_write_time(so_path);
                const fs::path rbo{"RobotBase.o"};
                if (!need_compile && fs::exists(rbo)) {
                    need_compile = fs::last_write_time(rbo) > fs::last_write_time(so_path);
                }
            }
        } catch (const fs::filesystem_error&) {
            need_compile = true;
        }
        if (need_compile) {
            std::ostringstream cmd;
            cmd << "g++ -shared -fPIC -std=c++20 -o " << so_relative << " " << cpp_str
                << " RobotBase.o -I.";
            if (std::system(cmd.str().c_str()) != 0) {
                std::cerr << "Compile failed:\n" << cmd.str() << "\nSkipping " << cpp_str << '\n';
                continue;
            }
        }
        void* dl_handle = dlopen(so_relative.c_str(), RTLD_NOW);
        if (!dl_handle) {
            std::cerr << "dlopen failed for " << so_relative << ": " << dlerror() << '\n';
            continue;
        }
        RobotFactory factory = reinterpret_cast<RobotFactory>(dlsym(dl_handle, "create_robot"));
        if (!factory) {
            std::cerr << "Missing create_robot() in " << so_relative << '\n';
            dlclose(dl_handle);
            continue;
        }
        RobotBase* robot = factory();
        if (!robot) {
            std::cerr << "create_robot returned null for " << so_relative << '\n';
            dlclose(dl_handle);
            continue;
        }
        {
            std::string stem = cpp_path.stem().string();
            constexpr std::string_view robot_prefix = "Robot_";
            if (stem.size() > robot_prefix.size() && stem.starts_with(robot_prefix)) {
                robot->m_name = stem.substr(robot_prefix.size());
            } else {
                robot->m_name = std::move(stem);
            }
        }
        std::cout << "Loaded robot " << robot->m_name << " [" << fname << "]\n";
        m_robots.push_back(robot);
        m_robot_handles.push_back(dl_handle);
    }
    if (m_robots.empty()) {
        std::cerr << "Arena: no robots were successfully compiled/loaded.\n";
        cleanup_loaded();
        return false;
    }
    return true;
}

bool Arena::place_robots_random()
{
    constexpr char chars[] = "@#$%&*!^-~?";
    constexpr int nchar = static_cast<int>(sizeof(chars) - 1);
    std::mt19937 rng{std::random_device{}()};
    std::uniform_int_distribution<int> ur{0, m_height - 1};
    std::uniform_int_distribution<int> uc{0, m_width - 1};
    int next_char = 0;

    for (RobotBase* bot : m_robots) {
        bot->set_boundaries(m_height, m_width);
        bool placed = false;
        for (int attempts = 0; !placed && attempts < 10000; ++attempts) {
            const int r = ur(rng);
            const int c = uc(rng);
            char occ = m_terrain[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)];
            if (occ == 'M' || occ == 'P' || occ == 'F' || robot_at(r, c) != nullptr) {
                continue;
            }
            bot->move_to(r, c);
            bot->m_character = chars[next_char++ % nchar];
            placed = true;
        }
        if (!placed) {
            std::cerr << "Could not place robot " << bot->m_name << " on empty terrain.\n";
            cleanup_loaded();
            return false;
        }
    }
    return true;
}

int Arena::rnd_int(const int lo, const int hi)
{
    thread_local std::mt19937 rng{std::random_device{}()};
    return std::uniform_int_distribution<int>{lo, hi}(rng);
}

int Arena::apply_damage_with_armor(RobotBase& victim, const int lo, const int hi, std::string& msg_out)
{
    const int base = rnd_int(lo, hi);
    const int armor_before = victim.get_armor();
    double factor = std::max(0.0, 1.0 - 0.10 * static_cast<double>(armor_before));
    const int damage = std::clamp(static_cast<int>(std::lround(static_cast<double>(base) * factor)), 0, base);
    victim.take_damage(damage);
    victim.reduce_armor(1);
    msg_out = victim.m_name + " takes " + std::to_string(damage) + " damage (armor was "
            + std::to_string(armor_before) + ")";
    return damage;
}

void Arena::handle_shot(RobotBase& shooter, const int shot_row, const int shot_col,
                       std::string& log_line)
{
    std::ostringstream oss;
    oss << shooter.m_name << " (" << shooter.m_character << ") shoots";
    const WeaponType wt = shooter.get_weapon();
    std::vector<std::pair<int, int>> impacted;

    int sr = 0;
    int sc = 0;
    shooter.get_current_location(sr, sc);
    int dr = 0;
    int dc = 0;

    switch (wt) {
    case railgun: {
        if (!greedy_step(sr, sc, shot_row, shot_col, dr, dc)) {
            if (sr + 1 < m_height) {
                dr = 1;
            } else if (sc + 1 < m_width) {
                dc = 1;
            }
        } else if (!first_step_in_bounds(sr, sc, dr, dc, m_height, m_width)) {
            dr = dc = 0;
        }
        if (dr != 0 || dc != 0) {
            extend_ray_line(sr, sc, dr, dc, m_height, m_width, impacted);
        }
        oss << " railgun:";
        break;
    }
    case hammer:
        if (greedy_step(sr, sc, shot_row, shot_col, dr, dc)
            && first_step_in_bounds(sr, sc, dr, dc, m_height, m_width)) {
            int cur_r = sr;
            int cur_c = sc;
            for (;;) {
                cur_r += dr;
                cur_c += dc;
                if (cur_r < 0 || cur_r >= m_height || cur_c < 0 || cur_c >= m_width) {
                    break;
                }
                const char ter =
                    m_terrain[static_cast<std::size_t>(cur_r)][static_cast<std::size_t>(cur_c)];
                if (ter == 'M') {
                    break;
                }
                if (RobotBase* occ = robot_at(cur_r, cur_c)) {
                    if (occ->get_health() > 0) {
                        impacted.emplace_back(cur_r, cur_c);
                    }
                    break;
                }
            }
        }
        oss << " hammer:";
        break;
    case flamethrower:
        if (greedy_step(sr, sc, shot_row, shot_col, dr, dc)
            && first_step_in_bounds(sr, sc, dr, dc, m_height, m_width)) {
            const int prep_r = -dc;
            const int prep_c = dr;
            std::vector<std::pair<int, int>> uniq;
            for (int depth = 1; depth <= 4; ++depth) {
                const int base_r = sr + dr * depth;
                const int base_c = sc + dc * depth;
                for (int k = -1; k <= 1; ++k) {
                    const int rr = base_r + k * prep_r;
                    const int cc = base_c + k * prep_c;
                    if (rr >= 0 && rr < m_height && cc >= 0 && cc < m_width) {
                        uniq.emplace_back(rr, cc);
                    }
                }
            }
            std::sort(uniq.begin(), uniq.end());
            uniq.erase(std::unique(uniq.begin(), uniq.end()), uniq.end());
            impacted = std::move(uniq);
        }
        oss << " flamethrower:";
        break;
    case grenade:
        if (shooter.get_grenades() <= 0) {
            oss << " grenade FAILED (none left)";
            log_line = oss.str();
            return;
        }
        shooter.decrement_grenades();
        for (int ddr = -1; ddr <= 1; ++ddr) {
            for (int ddc = -1; ddc <= 1; ++ddc) {
                const int rr = shot_row + ddr;
                const int cc = shot_col + ddc;
                if (rr >= 0 && rr < m_height && cc >= 0 && cc < m_width) {
                    impacted.emplace_back(rr, cc);
                }
            }
        }
        oss << " grenade:";
        break;
    }

    std::sort(impacted.begin(), impacted.end());
    impacted.erase(std::unique(impacted.begin(), impacted.end()), impacted.end());

    static constexpr int kDmgLo[] = {30, 10, 10, 50};
    static constexpr int kDmgHi[] = {50, 20, 40, 60};
    static_assert(sizeof(kDmgLo) / sizeof(kDmgLo[0]) == 4);

    bool tagged = false;
    for (const auto& [rr, cc] : impacted) {
        RobotBase* target = robot_at(rr, cc);
        if (target == nullptr || target == &shooter || target->get_health() <= 0) {
            continue;
        }
        std::string detail;
        const int wi = static_cast<int>(wt);
        apply_damage_with_armor(*target, kDmgLo[wi], kDmgHi[wi], detail);
        oss << " Hit " << target->m_name << " at (" << rr << "," << cc << "), " << detail << "; ";
        tagged = true;
        if (target->get_health() <= 0) {
            char& cell = m_terrain[static_cast<std::size_t>(rr)][static_cast<std::size_t>(cc)];
            if (cell == 'F') {
                cell = '.';
            }
        }
    }
    if (!tagged) {
        oss << " misses";
    }
    log_line = oss.str();
}

bool Arena::try_move_robot(RobotBase& robot, const int direction, const int raw_distance,
                          std::string& log_line)
{
    std::ostringstream oss;
    if (direction < 1 || direction > 8) {
        log_line = std::string(robot.m_name) + " (" + robot.m_character + "): idle (invalid direction).";
        return false;
    }
    const int max_speed = robot.get_move_speed() <= 0 ? 0 : robot.get_move_speed();
    if (max_speed == 0) {
        log_line = std::string(robot.m_name) + " (" + robot.m_character + "): trapped in pit.";
        return false;
    }
    const int dist = std::min(raw_distance <= 0 ? 0 : raw_distance, max_speed);
    if (dist == 0) {
        log_line =
            std::string(robot.m_name) + " (" + robot.m_character + "): chooses not to move.";
        return false;
    }
    const int dr = directions[direction].first;
    const int dc = directions[direction].second;
    int row = 0;
    int col = 0;
    robot.get_current_location(row, col);
    oss << robot.m_name << " (" << robot.m_character << ") moves from (" << row << "," << col << ")";
    bool moved_any = false;

    for (int step = 0; step < dist; ++step) {
        const int nr = row + dr;
        const int nc = col + dc;
        if (nr < 0 || nc < 0 || nr >= m_height || nc >= m_width) {
            break;
        }
        if (RobotBase* occ = robot_at(nr, nc); occ != nullptr && occ != &robot) {
            break;
        }
        auto& tcell = m_terrain[static_cast<std::size_t>(nr)][static_cast<std::size_t>(nc)];
        const char terrain = tcell;
        if (terrain == 'M') {
            break;
        }
        if (terrain == 'P') {
            robot.move_to(nr, nc);
            row = nr;
            col = nc;
            moved_any = true;
            robot.disable_movement();
            oss << "; fell into pit at (" << row << "," << col << "), movement disabled";
            break;
        }
        if (terrain == 'F') {
            std::string burn_msg;
            apply_damage_with_armor(robot, 30, 50, burn_msg);
            oss << "; passes flame (" << burn_msg << ")";
        }
        robot.move_to(nr, nc);
        row = nr;
        col = nc;
        moved_any = true;
        if (robot.get_health() <= 0) {
            if (terrain == 'F') {
                tcell = '.';
            }
            oss << "; died while moving.";
            log_line = oss.str();
            return moved_any;
        }
    }
    oss << " to (" << row << "," << col << ")";
    log_line = oss.str();
    return moved_any;
}

void Arena::print_round_header() const
{
    std::cout << "\n         =========== starting round " << m_round << " ===========\n\n";
}

void Arena::print_arena() const
{
    std::cout << "    ";
    for (int cc = 0; cc < m_width; ++cc) {
        std::cout << std::setw(3) << cc;
    }
    std::cout << '\n';
    for (int rr = 0; rr < m_height; ++rr) {
        std::cout << std::setw(3) << rr << ' ';
        for (int cc = 0; cc < m_width; ++cc) {
            char g;
            char d = ' ';
            if (RobotBase* at = robot_at(rr, cc)) {
                g = at->get_health() > 0 ? 'R' : 'X';
                d = at->m_character;
            } else {
                g = m_terrain[static_cast<std::size_t>(rr)][static_cast<std::size_t>(cc)];
                if (g == '.') {
                    std::cout << std::setw(4) << '.';
                    continue;
                }
            }
            if (g == 'R') {
                std::cout << 'R' << d << ' ';
            } else if (g == 'X') {
                std::cout << g << d << ' ';
            } else {
                std::cout << std::setw(4) << g;
            }
        }
        std::cout << '\n';
    }
    std::cout << '\n';
    for (RobotBase* bot : m_robots) {
        int r = 0;
        int c = 0;
        bot->get_current_location(r, c);
        std::cout << bot->m_name << " (" << bot->m_character << ") at (" << r << "," << c;
        if (bot->get_health() <= 0) {
            std::cout << ") - is out\n";
        } else {
            std::cout << ") Health: " << bot->get_health() << " Armor: " << bot->get_armor() << '\n';
        }
    }
    std::cout << '\n';
}

void Arena::run()
{
    load_config();
    allocate_terrain();
    scatter_obstacle('M', std::max(0, m_mounds));
    scatter_obstacle('P', std::max(0, m_pits));
    scatter_obstacle('F', std::max(0, m_flamethrowers));
    if (!load_robots_from_disk() || !place_robots_random()) {
        return;
    }

    RobotBase* announced_winner = nullptr;
    bool game_over = false;

    while (!game_over && m_round <= m_max_rounds) {
        m_result_round_echo = m_round;
        if (RobotBase* w = sole_winner_if_any()) {
            announced_winner = w;
            break;
        }
        if (m_game_state_live) {
            print_round_header();
            print_arena();
        }

        for (RobotBase* bot_ptr : m_robots) {
            if (RobotBase* w_mid = sole_winner_if_any()) {
                announced_winner = w_mid;
                game_over = true;
                break;
            }
            if (bot_ptr->get_health() <= 0) {
                continue;
            }

            int radar_dir = 1;
            bot_ptr->get_radar_direction(radar_dir);
            std::ostringstream activity;
            int r0 = 0;
            int c0 = 0;
            bot_ptr->get_current_location(r0, c0);
            activity << bot_ptr->m_name << " (" << bot_ptr->m_character << ") at (" << r0 << "," << c0
                     << ") Health: " << bot_ptr->get_health() << " Armor: " << bot_ptr->get_armor()
                     << "\n  radar_direction=" << radar_dir;

            std::vector<RadarObj> radar_objs = scan_radar(*bot_ptr, radar_dir);
            bot_ptr->process_radar_results(radar_objs);
            if (!radar_objs.empty()) {
                activity << " saw:";
                for (const RadarObj& ro : radar_objs) {
                    activity << ' ' << ro.m_type << '@' << ro.m_row << ',' << ro.m_col << ';';
                }
            }
            activity << '\n';

            int shot_row = 0;
            int shot_col = 0;
            if (bot_ptr->get_shot_location(shot_row, shot_col)) {
                if (shot_row < 0 || shot_col < 0 || shot_row >= m_height || shot_col >= m_width) {
                    activity << "  shot ignored (bad coordinates)\n";
                } else if (bot_ptr->get_weapon() == grenade && bot_ptr->get_grenades() <= 0) {
                    activity << "  grenade aborted (ammo empty)\n";
                } else {
                    std::string shot_msg;
                    handle_shot(*bot_ptr, shot_row, shot_col, shot_msg);
                    activity << "  " << shot_msg << '\n';
                }
            } else {
                int move_direction = 0;
                int move_distance = 0;
                bot_ptr->get_move_direction(move_direction, move_distance);
                std::string move_msg;
                try_move_robot(*bot_ptr, move_direction, move_distance, move_msg);
                activity << "  " << move_msg << '\n';
            }

            if (m_game_state_live) {
                std::cout << activity.str();
                std::flush(std::cout);
            }
            if (RobotBase* winner = sole_winner_if_any()) {
                announced_winner = winner;
                game_over = true;
                break;
            }
        }

        if (game_over) {
            break;
        }
        if (RobotBase* w = sole_winner_if_any()) {
            announced_winner = w;
            break;
        }
        if (m_game_state_live && m_sleep_seconds > 0.0) {
            const auto ms = static_cast<long>(
                std::lround(std::clamp(m_sleep_seconds, 0.0, 60.0) * 1000.0));
            std::this_thread::sleep_for(std::chrono::milliseconds(ms));
        }
        ++m_round;
    }

    if (!announced_winner) {
        std::cout << "\n\nMax rounds reached! Finding survivors...\n";
        print_arena();
        for (RobotBase* robot : m_robots) {
            if (robot->get_health() > 0) {
                int r = 0;
                int c = 0;
                robot->get_current_location(r, c);
                std::cout << "Survivor: " << robot->m_name << " at (" << r << "," << c
                          << ") Health: " << robot->get_health() << " Armor: " << robot->get_armor()
                          << '\n';
            }
        }
    } else {
        std::cout << "\n\n╔════════════════════════════════════════╗\n║         WINNER: " << std::left
                  << std::setw(20) << announced_winner->m_name << "  ║\n╚════════════════════════════════════════╝\n\n";
        print_arena();
        int r = 0;
        int c = 0;
        announced_winner->get_current_location(r, c);
        std::cout << "Final Stats:\n  Name: " << announced_winner->m_name << "\n  Position: (" << r
                  << "," << c << ")\n  Health: " << announced_winner->get_health() << "\n  Armor: "
                  << announced_winner->get_armor()
                  << "\nVictory achieved in round " << m_result_round_echo << "!\n";
    }
    cleanup_loaded();
}
