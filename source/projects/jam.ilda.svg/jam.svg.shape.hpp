    //
    //  jam.ilda.hpp
    //  jam.ilda.svg
    //
    //  Created by Jan Mech on 9/4/25.
    //

#ifndef jam_ilda_hpp
#define jam_ilda_hpp

#include <vector>
#include <cmath>
#include <string>
#include <cstdio>

#ifndef PI
#define PI 3.14159265358979323846
#endif

namespace jam::svg {
    
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
        
        float _vecScalarProduct(const Point2D& a, const Point2D& b);
        
        float _vecLength(const Point2D& v);
        
        float _vecAngle(const Point2D& a, const Point2D& b);
        
        
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
        void thinShape(float angleThresholdRadians = 0.01f);
        
    
    };
    
    
};

#endif /* jam_ilda_hpp */
