
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
                                 int segments = 1000
                                 );
        lpvec makeLinePoints(
                              const coord_point_t& p1,
                              const coord_point_t& p2
                             );
        
        laser_point_t _toLaserPoint(coord_point_t p, bool set_blanking = true);
        
    protected:
        template <typename T> T _map(T x, T in_min, T in_max, T out_min, T out_max);
        bool _laserPointVisible(laser_point_t &lp);
        
    };
}




