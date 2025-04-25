//
//  jam.cpp
//  jam.ilda.compose
//
//  Created by Jan Mech on 24/4/25.
//

#include "jam_compose_data.hpp"

namespace jam::compose {
    
    /// Public methods
    
    void DataSet::setScale(number s) {
        this->setScale(s, s);
    };
    
    void DataSet::setScale(number sx, number sy) {
        this->_scale_factor.x = sx;
        this->_scale_factor.y = sy;
    };
    
    void DataSet::setRotaion(number angle, Point2D anchor) {
        angle = -1. * angle;
        number angle_rad = angle * (PI / 180.);
        this->_rotation_rad = angle_rad;
        this->_rotation_anchor = anchor;
    };

    std::vector<DataPoint> DataSet::getDataPoints() {
        return this->_points_raw;
    };
    
    void DataSet::addDataPoint(DataPoint p) {
        this->_points_raw.push_back(p);
    };
    
    void DataSet::clearRawPoints() {
        this->_points_raw.clear();
    };
    
    IldaFrame DataSet::toIldaFrame() {
        static std::mutex frame_parsing_lock;
        frame_parsing_lock.lock();
        this->_processDataPoints();
        
        jam::ilda::IldaFrame f;
        jam::ilda::IldaHeader h;
        h.setFormatCode(jam::ilda::FORMAT_5);
        f.setHeader(h);
        
        
        // parse processed points to IldaDataRecods
        if(this->_points_processed.size() > 1) {
            for(size_t i = 0; i < this->_points_processed.size() - 1; i++) {
                DataPoint p1 = this->_points_processed[i];
                DataPoint p2 = this->_points_processed[i + 1];
                OptionalDataPointPair cp = this->_clipLineSegment(p1,p2);
                    // Line segment is fully invisible, ignore it
                if(cp.has_value()) {
                    std::pair<DataPoint, DataPoint> clipped = cp.value();
                    
                    bool p1_clipped = (clipped.first.x != p1.x || clipped.first.y != p1.y);
                    bool p2_clipped = (clipped.second.x != p2.x || clipped.second.y != p2.y);
                    if(p1_clipped) { // if p1 was clipped (outside the visible bounds, move to the intersection)
                        IldaDataRecord dr_move;
                        dr_move.setPosX(this->_deNormalizePosition(clipped.first.x));
                        dr_move.setPosY(this->_deNormalizePosition(clipped.first.y));
                        dr_move.setRed(0);
                        dr_move.setGreen(0);
                        dr_move.setBlue(0);
                        dr_move.setBlanking(true);
                        dr_move.setLastPoint(clipped.first.last_point);
                        f.pushRecord(dr_move);
                    }
                    
                    IldaDataRecord dr_first;
                    dr_first.setPosX(this->_deNormalizePosition(clipped.first.x));
                    dr_first.setPosY(this->_deNormalizePosition(clipped.first.y));
                    dr_first.setRed(static_cast<uint8_t>(clipped.first.r * 255.));
                    dr_first.setGreen(static_cast<uint8_t>(clipped.first.g * 255.));
                    dr_first.setBlue(static_cast<uint8_t>(clipped.first.b * 255.));
                    dr_first.setBlanking(p1.blanking);
                    dr_first.setLastPoint(p1.last_point);
                    f.pushRecord(dr_first);
                    
                        // if(p2_clipped || i + 1 == this->_points_processed.size() - 1) {
                    if(p2_clipped || i == this->_points_processed.size() - 2) { // if p2 was clipped (outside the visible bounds, draw to the intersection). add the last point in any case
                        IldaDataRecord dr_second;
                        dr_second.setPosX(this->_deNormalizePosition(clipped.second.x));
                        dr_second.setPosY(this->_deNormalizePosition(clipped.second.y));
                        dr_second.setRed(static_cast<uint8_t>(clipped.second.r * 255.));
                        dr_second.setGreen(static_cast<uint8_t>(clipped.second.g * 255.));
                        dr_second.setBlue(static_cast<uint8_t>(clipped.second.b * 255.));
                        dr_second.setBlanking(p2.blanking);
                        dr_second.setLastPoint(p2.blanking);
                        f.pushRecord(dr_second);
                    }
                }
            }
        }
        
         this->_thinFrameData(f);
        frame_parsing_lock.unlock();
        
        return f;
        
    }
    
        /// Protected methods
    int DataSet::_deNormalizePosition(double pos) {
        return static_cast<int>(pos * 32000);
        
        int de_normalized = static_cast<int>(pos * 32000);
        return std::clamp(de_normalized, -32767, 32767);
    }
    
    void DataSet::_processDataPoints() {
        static std::mutex points_precessing_lock;
        points_precessing_lock.lock();
            // apply scaling
            // apply rotation
        number cos_a      = std::cos(this->_rotation_rad);
        number sin_a      = std::sin(this->_rotation_rad);
        this->_points_processed.clear();
        for (auto p : this->_points_raw) {
            DataPoint pp;
                // Copy color information
            pp.r = p.r;
            pp.g = p.g;
            pp.b = p.b;
            
            pp.blanking = p.blanking;
            pp.last_point = p.last_point;
            
            pp.x = p.x * this->_scale_factor.x;
            pp.y = p.y * this->_scale_factor.y;
            
            
                // calculate delta x/y - ajust for rotation anchor
            Point2D delta = { pp.x - this->_rotation_anchor.x, pp.y - this->_rotation_anchor.y};
            
            
                // calculate rotated x/y
            number rx = (delta.x * cos_a) - (delta.y * sin_a) + this->_rotation_anchor.x;
            number ry = (delta.x * sin_a) + (delta.y * cos_a) + this->_rotation_anchor.y;
            
            pp.x = rx;
            pp.y = ry;
            
                // add the processed point
            this->_points_processed.push_back(pp);
            points_precessing_lock.unlock();
            
        }
    };
    
        // Liang-Barsky clipping
    OptionalDataPointPair DataSet::_clipLineSegment(const DataPoint& p1, const DataPoint& p2) {
        number x1 = p1.x, y1 = p1.y;
        number dx = p2.x - x1;
        number dy = p2.y - y1;
        
        number t_min = 0.0f;
        number t_max = 1.0f;
        
        auto clip = [&](number p, number q) -> bool {
            if (p == 0) return q >= 0; // Parallel line
            number r = q / p;
            if (p < 0) {
                if (r > t_max) return false;
                if (r > t_min) t_min = r;
            } else {
                if (r < t_min) return false;
                if (r < t_max) t_max = r;
            }
            return true;
        };
        
            // Test all 4 boundaries
        if (
            clip(-dx, x1 + 1) &&
            clip( dx, 1 - x1) &&
            clip(-dy, y1 + 1) &&
            clip( dy, 1 - y1)
            ) {
                DataPoint clipped_1 = p1;
                clipped_1.x = x1 + dx * t_min;
                clipped_1.y = y1 + dy * t_min;
                
                DataPoint clipped_2 = p2;
                clipped_2.x = x1 + dx * t_max;
                clipped_2.y = y1 + dy * t_max;
                
                return std::make_pair(clipped_1, clipped_2);
            }
        return std::nullopt; // Fully outside
    }
    
    void DataSet::_thinFrameData(IldaFrame &f) {
        std::vector<IldaDataRecord> frame_data_records = f.getDataRecords();
        std::vector<IldaDataRecord> no_redundan;
        size_t i = 0;
        // always add the first
        while (i < frame_data_records.size()) {
            if(i == 0) {
                no_redundan.push_back(frame_data_records[i]);
            };
            // look for the next non redundant frame
            size_t j = i;
            while (j + 1 < frame_data_records.size() && this->_dataRecordsRedundant(frame_data_records[i], frame_data_records[j + 1])) {
                ++j;
            }
            no_redundan.push_back(frame_data_records[j]); // Keep only the last blanking point
            i = j + 1;
        }
        
        f.setDataRecords(no_redundan);
    };
    
    bool DataSet::_dataRecordsRedundant(IldaDataRecord &d1, IldaDataRecord &d2) {
        
            // DR are redundant if they are either Identical or both blanked
        return (d1.getRed() == d2.getRed()
                && d1.getGreen() == d2.getGreen()
                && d1.getBlue() == d2.getBlue()
                && d1.getPosX() == d2.getPosX()
                && d1.getPosY() == d2.getPosY())
        || (d1.getBlanking() && d2.getBlanking());
    };
};
