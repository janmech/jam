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
    
    void DataSet::setRotaion(number angle) {
        angle = -1. * angle;
        number angle_rad = angle * (PI / 180.);
        this->_rotation_rad = angle_rad;
    };
    
    void DataSet::setRotationAnchor(Point2D anchor) {
        this->_rotation_anchor = anchor;
    };
    
    std::vector<DataPoint> DataSet::getRawPoints() {
        return this->_points_raw;
    };
    
    void DataSet::addRawPoint(DataPoint p) {
        this->_points_raw.push_back(p);
    };
    
    void DataSet::clearRawPoints() {
        this->_points_raw.clear();
    };
    
    std::vector<DataPoint> DataSet::getProcessedPoints() {
        return this->_points_processed;
    };
    
    
    jam::ilda::IldaFrame DataSet::toIldaFrame() {
        this->_processDataPoints();
        jam::ilda::IldaFrame f;
        jam::ilda::IldaHeader h;
        h.setFormatCode(jam::ilda::FORMAT_4);
        f.setHeader(h);
        for(auto it = this->_points_processed.begin(); it != this->_points_processed.end(); it++) {
            jam::ilda::IldaDataRecord dr;
            dr.setPosX(this->_deNormalizePosition(it->x));
            dr.setPosY(this->_deNormalizePosition(it->y));
            dr.setRed(static_cast<uint8_t>(it->r * 255.));
            dr.setGreen(static_cast<uint8_t>(it->g * 255.));
            dr.setBlue(static_cast<uint8_t>(it->b * 255.));
            dr.setBlanking(it->blanking);
            dr.setLastPoint(it->last_point);
            f.pushRecord(dr);
        }
        return f;
        
            // UFF mor difficult than I thought... we need to know then context: where does it come from, where does it go to....
            // case A) Point is visible and previous is visible --> add point
            // case B) Point is visible as previous is NOT visible  --> add a point where it hits the bounds ad mark as such
            // case C) Point is NOT visible and previous point is NOT visible --> ignore point
            // case D) Point is NOT visible and previous point IS visible --> add point where it hits the bounds and mark as such
            
            
            // if the previous generated point is a bounding box point set to blanking
            
    }

    
    /// Protected methods
    int DataSet::_deNormalizePosition(double pos) {
        int de_normalized = static_cast<int>(pos * 32000);
        return std::clamp(de_normalized, -32767, 32767);
    }

    void DataSet::_processDataPoints() {
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
//            Point2D delta = { p.x - this->_rotation_anchor.x, p.y - this->_rotation_anchor.y};
//            
//        
//            // calculate rotated x/y
//            number rx = (delta.x * cos_a) - (delta.y * sin_a) + this->_rotation_anchor.x;
//            number ry = (delta.x * sin_a) + (delta.y * cos_a) + this->_rotation_anchor.y;
//            
//            pp.x = rx;
//            pp.y = ry;
            
            // add the processed point
            this->_points_processed.push_back(pp);
        }
    };
};
