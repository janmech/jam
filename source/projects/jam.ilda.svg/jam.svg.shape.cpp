    //
    //  jam.ilda.cpp
    //  jam.ilda.svg
    //
    //  Created by Jan Mech on 9/4/25.
    //

#include "jam.svg.shape.hpp"

namespace jam::svg {
    
        // Public Methods
    
    std::string Shape::getName() {
        return this->name;;
    };
    
    void Shape::setName(std::string name) {
        this->name = name;
    };
    
    void Shape::clearPoints() {
        this->points.clear();
    }
    
    std::vector<Point2D> Shape::getPoints() {
        return this->points;;
    };
    
    
    void Shape::setPoints(std::vector<Point2D> points){
        this->points =points;
    };
    
    RGBColor Shape::getColor() {
        return this->color;;
    };
    
    void Shape::setColor(RGBColor color) {
        this->color = color;
    };
    
    void Shape::addPoint(const Point2D& point) {
        this->points.push_back(point);
    }
    
    void Shape::thinShape(float angleThresholdRadians) {
        std::vector<Point2D> thinned_points;
        const auto& pts = this->points;
        
        if (pts.size() < 3) {
            return; // Nothing to thin
        }
        
        
        thinned_points.push_back(pts[0]); // Always keep the first point
        
        // Iterate over points. When the angle between the previous and the next point is < angleThresholdRadians we consider it a straight line and skip the middle point
        for (size_t i = 1; i < pts.size() - 1; ++i) {
            DirectionVector prev = { pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y };
            DirectionVector next = { pts[i + 1].x - pts[i].x, pts[i + 1].y - pts[i].y };
            float angle = this->_vecAngle(prev, next);
            
            if (angle > angleThresholdRadians) {
                thinned_points.push_back(pts[i]); // Keep it if the angle changes enough
            }
        }
        
        thinned_points.push_back(pts.back()); // Always keep the last point
        this->points = thinned_points;
        
    };
    
    
        // Protected Methods
    
    float Shape::_vecScalarProduct(const Point2D& a, const Point2D& b){
        return a.x * b.x + a.y * b.y;
    };
    
    float Shape::_vecLength(const Point2D& v){
        return std::sqrt(v.x * v.x + v.y * v.y);
    };
    
    float Shape::_vecAngle(const Point2D& a, const Point2D& b){
        float lenA = this->_vecLength(a);
        float lenB = this->_vecLength(b);
        if (lenA == 0 || lenB == 0) return 0;
        float cosAngle = this->_vecScalarProduct(a, b) / (lenA * lenB);
        return std::acos(std::clamp(cosAngle, -1.0f, 1.0f)); // radians
    };
    
};
