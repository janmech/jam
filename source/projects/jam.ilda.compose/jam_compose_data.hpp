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
#include "jam.shape.hpp"

#ifndef PI
/** The pi constant.  */
#define PI 3.14159265358979323846
#endif

using number = c74::min::number;
using Point2D = jam::Point2D;

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
        Point2D _scale_factor = {1., 1.};           // x/y scale factor
        number _rotation_rad = 0.;                  // rotation angle in rad
        Point2D _rotation_anchor = {0., 0.};        // rotate DataSet around this point
        std::vector<DataPoint> _points_raw;         // raw points: scaling  and rotation not applied
        std::vector<DataPoint> _points_processed;    // processed points:scaling  and rotation applied
        
        int _deNormalizePosition(double pos);       // translate  -1. to 1. coordinates to ILDA coordinates
        // apply scazling and rotation
        void _processDataPoints();                  // apply scaling and rotation to raw data points

        
    public:
        void setScale(number s);                    // set scaling factor for x and y direction
        void setScale(number sx, number sy);        // set scaling factor for x and y direction separately
        void setRotaion(number angle);              // set rotation in degree (0º to 360º)
        void setRotationAnchor(Point2D anchor);     // set rotation anchor
        std::vector<DataPoint> getRawPoints();      // returns a vector with raw DataPoints
        void addRawPoint(DataPoint p);              // Push back a raw data point
        void clearRawPoints();                      // Clear raw data points
        std::vector<DataPoint> getProcessedPoints();// returns processed data points
        jam::ilda::IldaFrame toIldaFrame();
    };
};

#endif /* jam_hpp */
