    //
    //  jam.hpp
    //  jam.ilda.compose
    //
    //  Created by Jan Mech on 24/4/25.
    //

#ifndef jam_compose_point_hpp
#define jam_compose_point_hpp

#include "c74_min_api.h"
#include "../jam.ilda_common/ilda_frame.hpp"
#include "../jam.ilda_common/ilda_data_record.hpp"

using number = c74::min::number;

namespace jam::compose {
    struct DataPoint {
        number x = 0.;
        number y = 0.;
        number z = 0.;
        number r = 0.;
        number g = 0.;
        number b = 0.;
        bool blanking = false;
        bool last_point = false;
    };
    
    class DataSet {
    protected:
        int _deNormalizePosition(double pos) {
            int de_normalized = static_cast<int>(pos * 32000);
            return std::clamp(de_normalized, -32767, 32767);
        };
    public:
        number scale_factor = 1.;
        number rotation = 0.;
        std::vector<DataPoint> points;
        jam::ilda::IldaFrame frameSkeleton(){
            jam::ilda::IldaFrame f;
            jam::ilda::IldaHeader h;
            h.setFormatCode(jam::ilda::FORMAT_1);
            f.setHeader(h);
            for(auto it = this->points.begin(); it != this->points.end(); it++) {
                // point is inside visible area
                jam::ilda::IldaDataRecord dr;
                if(it->x <=1. && it->x >= -1. && it->y <=1. && it->y >= 1.) {
                    dr.setRed(static_cast<uint8_t>(it->r * 255.));
                    dr.setGreen(static_cast<uint8_t>(it->g * 255.));
                    dr.setBlue(static_cast<uint8_t>(it->b * 255.));
                    dr.setBlanking(it->blanking);
                    dr.setLastPoint(it->last_point);
                    f.pushRecord(dr);
                } else { // point is outside visible area
                    // calculate where it hits the bounds
                    // make a new point where it hits
                    // UFF mor difficult than I thought... we need to know then context: where does it come from, where does it go to....
                    // case A) Point is visible and previous is visible --> add point
                    // case B) Point is visible as previous is NOT visible  --> add a point where it hits the bounds ad mark as such
                    // case C) Point is NOT visible and previous point is NOT visible --> ignore point
                    // case D) Point is NOT visible and previous point IS visible --> add point where it hits the bounds and mark as such
                    
                    
                    // if the previous generated point is a bounding box point set to blanking
                    
                }
            }
            return f;
        };
    };
};

#endif /* jam_hpp */
