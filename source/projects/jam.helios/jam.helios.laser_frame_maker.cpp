//
//  jam.helios.laser_frame_maker.cpp
//  jam.helios
//
//  Created by Jan Mech on 18/04/2026.
//

#include "jam.helios.laser_frame_maker.hpp"

namespace jam::helios {
    
    lpvec FrameMaker::makeEllipsePoints(
        coord_point_t c,
        coord_point_t r,
        const number theta_start,
        const number theta_end,
        int segments
    ) {
        lpvec points;
        if(theta_start == theta_end) {
            return points;
        }
        number rad_start = theta_start * (PI / 180.);
        number rad_end = theta_end * (PI / 180.);
        number rad_range = rad_end - rad_start;
        
        
        for (int i = 0; i < segments; ++i) {
            coord_point_t cp;
           
            number angle = rad_start + (rad_range * i / segments);
            // number angle = rad_start + (number)(rad_range * (number)i / (number)segments);
            cp.x = c.x + r.x * std::cos(angle);
            cp.y = c.y + r.y * std::sin(angle);
            
            cp.x = std::clamp(cp.x, -1., 1.);
            cp.y = std::clamp(cp.y, -1., 1.);
            
            laser_point_t lp = this->_toLaserPoint(cp);
            points.push_back(lp);
        }
        points[segments - 1] = points[0];
        return points;
    }
    
    lpvec FrameMaker::makeLinePoints(
          const coord_point_t& p1,
          const coord_point_t& p2
          ) {
        lpvec points;
        number delta_x = p2.x - p1.x;
        number delta_y = p2.y - p1.y;
        for (int i = 0; i < POINTS_PER_DIRECT_DRAW_FRAME; ++i) {
            number t = static_cast<number>(i) / POINTS_PER_DIRECT_DRAW_FRAME;
            number x = p1.x + t * delta_x;
            number y = p1.y + t * delta_y;
            coord_point_t cp{std::clamp(x, -1., 1.), std::clamp(y, -1., 1.)};
            laser_point_t lp = this->_toLaserPoint(cp);
            points.push_back(lp);
        }
        for(int i = 1; i < 10; i++) {
            points[POINTS_PER_DIRECT_DRAW_FRAME - i] = points[0];
            points[POINTS_PER_DIRECT_DRAW_FRAME - i].blanking = true;
        }
        
        return points;
    }
    
    laser_point_t FrameMaker::_toLaserPoint(coord_point_t p, bool set_blanking) {
        laser_point_t lp;
        // from -1/1 to 0/0xFFFF (65535)
        number x_mapped = this->_map(p.x, -1., 1., 0., 65535.);
        number y_mapped = this->_map(p.y, -1., 1.,  0., 65535.);
        lp.x = (uint16_t)x_mapped;
        lp.y = (uint16_t)y_mapped;
        if(set_blanking) {
            lp.blanking = !this->_laserPointVisible(lp);
        }
        return lp;
    };
    
    // protected methods
    template <typename T> T FrameMaker::_map(T x, T in_min, T in_max, T out_min, T out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
    
    bool FrameMaker::_laserPointVisible(laser_point_t &lp) {
        
        return lp.x >= X_Y_MIN && lp.x <= X_Y_MAX && lp.y >= X_Y_MIN && lp.y <= X_Y_MAX;
    }
}
