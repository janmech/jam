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
using IldaFrame = jam::ilda::IldaFrame;
using IldaDataRecord = jam::ilda::IldaDataRecord;

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
    
    using OptionalDataPointPair = std::optional<std::pair<DataPoint, DataPoint>>;

    
    class DataSet {
    protected:
        Point2D _scale_factor = {1., 1.};           // x/y scale factor
        number _rotation_rad = 0.;                  // rotation angle in rad
        Point2D _rotation_anchor = {0., 0.};        // rotate DataSet around this point
        std::vector<DataPoint> _points_raw;         // raw points: scaling  and rotation not applied
        std::vector<DataPoint> _points_processed;    // processed points:scaling  and rotation applied
        
        int _deNormalizePosition(double pos);       // translate  -1. to 1. coordinates to ILDA coordinates
        // apply scazling and rotation
        void _processDataPoints();                   // apply scaling and rotation to raw data points
        
        OptionalDataPointPair _clipLineSegment(const DataPoint& p1, const DataPoint& p2);
        
        // remove consecutive blanked DataRecords
        void _thinFrameData(IldaFrame &f);
        
        bool _dataRecordsRedundant(IldaDataRecord &d1, IldaDataRecord &d2);

        
    public:
        void setScale(number s);                       // set scaling factor for x and y direction
        void setScale(number sx, number sy);           // set scaling factor for x and y direction separately
        void setRotaion(number angle, Point2D anchor); // set rotation in degree (0º to 360º) and the anchor point to rotate around
        std::vector<DataPoint> getDataPoints();        // returns a vector with raw DataPoints
        void addDataPoint(DataPoint p);                // Push back a raw data point
        void clearRawPoints();                         // Clear raw data points
        IldaFrame toIldaFrame();
        
        
        static DataSet frameToDataSet(IldaFrame f) {
            
            auto normalizePosition = [&](int pos) -> number {
                pos = (pos < -32768) ? -32768 : pos;
                pos = (pos > 32767) ? 32767 : pos;
                if (pos > 0) {
                    return ((number)pos / 32767.);
                }
                return ((number)pos / 32768);
            };
            
            DataSet ds;
            f.reset();
            IldaDataRecord data_record;
            while(f.getNext(&data_record)) {
                DataPoint dp;
                dp.blanking = data_record.getBlanking();
                dp.last_point = data_record.getLastPoint();
                dp.r = static_cast<number>(data_record.getRed()) / 255.;
                dp.g = static_cast<number>(data_record.getGreen()) / 255.;
                dp.b = static_cast<number>(data_record.getBlue()) / 255.;
                dp.x = normalizePosition(data_record.getPosX());
                dp.y = normalizePosition(data_record.getPosY());
                dp.z = normalizePosition(data_record.getPosZ());
                ds.addDataPoint(dp);
            }
            return ds;
        };
        
        static std::vector<DataSet> framesToDataSets(std::vector<IldaFrame> frames) {
           
            std::vector<DataSet> data_sets;
            
            for(auto it = frames.begin(); it != frames.end(); it++) {
                data_sets.push_back(DataSet::frameToDataSet(*it));
            }
            
            return data_sets;
        };
    };
};

#endif /* jam_hpp */
