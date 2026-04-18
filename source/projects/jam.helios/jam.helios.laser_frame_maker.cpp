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
        const int segments
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
            cp.x = c.x + r.x * std::cos(angle);
            cp.y = c.y + r.y * std::sin(angle);
            
            cp.x = std::clamp(cp.x, -1., 1.);
            cp.y = std::clamp(cp.y, -1., 1.);
            
            laser_point_t lp = this->toLaserPoint(cp);
            points.push_back(lp);
        }
        points[segments - 1] = points[0];
        return points;
    }
    
    lpvec FrameMaker::makeLinePoints(
          const coord_point_t& p1,
          const coord_point_t& p2,
          const int segments
          ) {
        
        lpvec in_points;
        in_points.push_back(this->toLaserPoint(p1));
        in_points.push_back(this->toLaserPoint(p2));
        in_points.push_back(this->toLaserPoint(p1));
        
        lpvec out_points = this->interpolatePoints(in_points);
        return out_points;
        
        
        
//        lpvec points;
//        number delta_x = p2.x - p1.x;
//        number delta_y = p2.y - p1.y;
//        for (int i = 0; i < segments; ++i) {
//            number t = static_cast<number>(i) / segments;
//            number x = p1.x + t * delta_x;
//            number y = p1.y + t * delta_y;
//            coord_point_t cp{std::clamp(x, -1., 1.), std::clamp(y, -1., 1.)};
//            laser_point_t lp = this->toLaserPoint(cp);
//            points.push_back(lp);
//        }
//        for(int i = 1; i < 10; i++) {
//            points[segments - i] = points[0];
//            points[segments - i].blanking = true;
//        }
        
//        return points;
    }
    
    laser_point_t FrameMaker::toLaserPoint(coord_point_t p, bool set_blanking) {
        laser_point_t lp;
        // from -1/1 to 0/0xFFFF (65535)
        number x_mapped = this->map(p.x, -1., 1., 0., 65535.);
        number y_mapped = this->map(p.y, -1., 1.,  0., 65535.);
        lp.x = (uint16_t)x_mapped;
        lp.y = (uint16_t)y_mapped;
        if(set_blanking) {
            lp.blanking = !this->laserPointVisible(lp);
        }
        return lp;
    };
    
    coord_point_t FrameMaker::toCoordPoint(const laser_point_t &lp) {
        coord_point_t cp;
    
        cp.x = this->map((number)lp.x, 0., 65535., -1., 1.);
        cp.y = this->map((number)lp.y, 0., 65535., -1., 1.);
        return cp;;
    }
    
    lpvec FrameMaker::makeDotPoints(number x, number y) {
        lpvec points;
        number x_coord_prev = 0.;
        for(int i = 0; i < POINTS_PER_DIRECT_DRAW_FRAME; i++) {
            coord_point_t cp;
            cp.y = y;
            number offset = (2. / (number)(POINTS_PER_DIRECT_DRAW_FRAME) * (number)i);
            number x_coord = -1. + offset;
            cp.x = x_coord;
            bool blanking = !(x >= x_coord_prev && x <= x_coord);
            laser_point_t lp = this->toLaserPoint(cp, false);
            lp.blanking = blanking;
            x_coord_prev = x_coord;
            points.push_back(lp);
        }
        return points;
    }
    
    bool FrameMaker::laserPointVisible(laser_point_t &lp, unsigned int xy_min, unsigned int xy_max) {
        
        return lp.x >= xy_min && lp.x <= xy_max && lp.y >= xy_min && lp.y <= xy_max;
    }
    
    lpvec FrameMaker::interpolatePoints(lpvec &in_laser_points) {
        lpvec out_laser_points;

        for (size_t i = 0; i < in_laser_points.size() - 1; ++i) {
            coord_point_t p1 = this->toCoordPoint(in_laser_points[i]);
            coord_point_t p2 = this->toCoordPoint(in_laser_points[i+1]);

            number dx = p2.x - p1.x;
            number dy = p2.y - p1.y;
            number dist = sqrt(dx*dx + dy*dy);

            int numSteps = std::max(1, (int)ceil(dist / this->_segment_size));

            for (int j = 0; j < numSteps; ++j) {
                number t = (number)j / numSteps;
                
                coord_point_t new_coordinate_point;
                // Interpolate
                new_coordinate_point.x = p1.x + t * dx;
                new_coordinate_point.y = p1.y + t * dy;
                

                laser_point_t new_lp = this->toLaserPoint(new_coordinate_point);
                

                out_laser_points.push_back(new_lp);
            }
        }
        return out_laser_points;
    };
    
    void FrameMaker::setSegmentSize(number segment_size) {
//        this->_segment_size = std::clamp(segment_size, 0.05, 0.1);
        this->_segment_size = segment_size;
    };
}
