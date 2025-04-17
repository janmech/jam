    //
    //  jam.ilda.hpp
    //  jam.ilda.svg
    //
    //  Created by Jan Mech on 9/4/25.
    //

#ifndef jam_shape_hpp
#define jam_shape_hpp

#include <vector>
#include <cmath>
#include <string>
#include <cstdio>
#include "c74_min_api.h"

using number = c74::min::number;

#ifndef PI
#define PI 3.14159265358979323846
#endif

namespace jam {
    
    struct Point2D {
        double x, y;
    };
    
        // Mathematically both are the same (a struct with an x and value). For better code readability we use DirectionVector instead of Point2D
    using DirectionVector = Point2D;
    
    struct RGBColor {
        uint8_t r = 255;
        uint8_t g = 255;
        uint8_t b = 255;
        bool visible = true;
    };
    
    class Shape {
        
    protected:
        
        RGBColor color;
        
        std::vector<Point2D> points;
        
        std::string name;
        
        number _vecScalarProduct(const Point2D& a, const Point2D& b);
        
        number _vecLength(const Point2D& v);
        
        number _vecAngle(const Point2D& a, const Point2D& b);
        
        
    public:
        
        
        Shape() = default;
        Shape(const std::string& shapeName) : name(shapeName) {}
        
        std::string getName();
        
        void setName(std::string name);
        
        void clearPoints();
        
        std::vector<Point2D> getPoints();
        
        void setPoints(std::vector<Point2D> points);
        
        void addPoint(const Point2D& point);
        
        RGBColor getColor();
        
        void setColor(RGBColor color);
        
            //// Thin out points to simplify the shape for better performace. Desctructive
        void thinShape(number angleThresholdRadians = 0.01f);
        
    
    };
    
    
};

#endif /* jam_ilda_hpp */
