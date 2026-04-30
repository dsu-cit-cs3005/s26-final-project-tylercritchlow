#include "RobotBase.h"
#include <vector>
#include <iostream>
#include <algorithm>

class Robot_BEAR : public RobotBase 
{
private:
    bool m_moving_down = true; 
    int to_shoot_row = -1;   
    int to_shoot_col = -1;   
    int m_radar_sweep = 0;
    
    std::vector<RadarObj> known_obstacles; 

    bool is_obstacle(int row, int col) const 
    {
        return std::any_of(known_obstacles.begin(), known_obstacles.end(), 
                           [&](const RadarObj& obj) {
                               return obj.m_row == row && obj.m_col == col;
                           });
    }

    void add_obstacle(const RadarObj& obj) 
    {
        if ((obj.m_type == 'M' || obj.m_type == 'P' || obj.m_type == 'F') && 
            !is_obstacle(obj.m_row, obj.m_col)) 
        {
            known_obstacles.push_back(obj);
        }
    }

public:
    Robot_BEAR() : RobotBase(2, 5, railgun) {} 

    virtual void get_radar_direction(int& radar_direction) override 
    {
        radar_direction = m_radar_sweep;
        m_radar_sweep = (m_radar_sweep + 1) % 8;
    }

    virtual void process_radar_results(const std::vector<RadarObj>& radar_results) override 
    {
        to_shoot_row = -1;
        to_shoot_col = -1;

        for (const auto& obj : radar_results) 
        {
            add_obstacle(obj);

            if (obj.m_type == 'R') 
            {
                to_shoot_row = obj.m_row;
                to_shoot_col = obj.m_col;
                break; 
            }
        }
    }

    virtual bool get_shot_location(int& shot_row, int& shot_col) override 
    {
        if (to_shoot_row != -1) 
        {
            shot_row = to_shoot_row;
            shot_col = to_shoot_col;
            return true;
        }
        return false;
    }

    virtual void get_move_direction(int& move_direction, int& move_distance) override 
    {
        int r, c;
        get_current_location(r, c);
        int speed = get_move_speed();

        if (c > 0) 
        {
            move_direction = 7;
            move_distance = std::min(speed, c);
        } 
        else 
        {
            move_distance = speed;
            if (m_moving_down) 
            {
                if (r + speed >= m_board_row_max) 
                {
                    m_moving_down = false;
                    move_direction = 1;
                } 
                else 
                {
                    move_direction = 5;
                }
            } 
            else 
            {
                if (r - speed < 0) 
                {
                    m_moving_down = true;
                    move_direction = 5;
                } 
                else 
                {
                    move_direction = 1;
                }
            }
        }


        int target_r = r, target_c = c;
    }
};

extern "C" RobotBase* create_robot() 
{
    return new Robot_BEAR();
}

extern "C" const char* robot_summary()
{
    return "BEAR: Tanky railgunner with sweeping radar.";
}