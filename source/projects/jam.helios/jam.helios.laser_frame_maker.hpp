
#pragma once
#include <stdio.h>
#include "c74_min_api.h"

#define POINTS_PER_DIRECT_DRAW_FRAME 1000
#define X_Y_MAX 65535 //65500
#define X_Y_MIN 0 //35

using number = c74::min::number;

namespace jam::helios {
    
    typedef struct LaserPoint {
        uint16_t x = 32767; // 65535 (0xFFFF)  / 2
        uint16_t y = 32767; // 65535 (0xFFFF)  / 2
        bool blanking = false;
    } laser_point_t;
    
    typedef struct CoordPoint {
        number x = 0.;
        number y = 0.;
    } coord_point_t;
    
    
    using lpvec = std::vector<laser_point_t>;
    
    class FrameMaker {
        
    public:
        FrameMaker(){};
        ~FrameMaker(){};
        
        lpvec makeEllipsePoints(
                                 coord_point_t c,
                                 coord_point_t r,
                                 const number theta_start = 0,
                                 const number theta_end = 360,
                                 const int segments = POINTS_PER_DIRECT_DRAW_FRAME
                                 );
        lpvec makeLinePoints(
                              const coord_point_t& p1,
                              const coord_point_t& p2,
                              const int segments = POINTS_PER_DIRECT_DRAW_FRAME
                             );
        lpvec makeDotPoints(number x, number y);
        
        laser_point_t toLaserPoint(coord_point_t p, bool set_blanking = true);
        coord_point_t toCoordPoint(const laser_point_t &lp);
        
        template <typename T> T map(T x, T in_min, T in_max, T out_min, T out_max) {
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        }
        
        bool laserPointVisible(laser_point_t &lp, unsigned int xy_min = X_Y_MIN, unsigned int xy_max = X_Y_MAX);
        
        lpvec interpolatePoints(lpvec &in_laser_points);
        
        void setSegmentSize(number segment_size);
        
        
    protected:
        number _segment_size = 0.05f;
    };
    
}




