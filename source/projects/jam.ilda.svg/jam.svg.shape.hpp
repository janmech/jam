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

namespace jam::svg {
    constexpr double PI_VAL = 3.14159265358979f;
    
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
            ///
        void thinShape(float angleThresholdRadians = 0.01f);
        
            //        // Circle
            //        void approximateCircle(const Point2D& center, float radius, int segments = 32) {
            //            clear();
            //            for (int i = 0; i < segments; ++i) {
            //                float angle = (2.0f * PI * i) / segments;
            //                float x = center.x + radius * std::cos(angle);
            //                float y = center.y + radius * std::sin(angle);
            //                addPoint({x, y});
            //            }
            //            addPoint(points.front());  // Close shape
            //            name = "Circle";
            //        }
            //
            //        // Ellipse
            //        void approximateEllipse(const Point2D& center, float rx, float ry, int segments = 32) {
            //            clear();
            //            for (int i = 0; i < segments; ++i) {
            //                float angle = (2.0f * PI * i) / segments;
            //                float x = center.x + rx * std::cos(angle);
            //                float y = center.y + ry * std::sin(angle);
            //                addPoint({x, y});
            //            }
            //            addPoint(points.front());  // Close shape
            //            name = "Ellipse";
            //        }
            //
            //        // Arc (from startAngle to endAngle in radians)
            //        void approximateArc(const Point2D& center, float radius, float startAngle, float endAngle, int segments = 32) {
            //            clear();
            //            float angleRange = endAngle - startAngle;
            //            for (int i = 0; i <= segments; ++i) {
            //                float angle = startAngle + (angleRange * i / segments);
            //                float x = center.x + radius * std::cos(angle);
            //                float y = center.y + radius * std::sin(angle);
            //                addPoint({x, y});
            //            }
            //            name = "Arc";
            //        }
            //
            //        // Cubic Bézier
            //        void approximateBezier(const Point2D& p0, const Point2D& p1, const Point2D& p2, const Point2D& p3, int segments = 24) {
            //            clear();
            //            for (int i = 0; i <= segments; ++i) {
            //                float t = static_cast<float>(i) / segments;
            //                float u = 1.0f - t;
            //                float x = u*u*u*p0.x + 3*u*u*t*p1.x + 3*u*t*t*p2.x + t*t*t*p3.x;
            //                float y = u*u*u*p0.y + 3*u*u*t*p1.y + 3*u*t*t*p2.y + t*t*t*p3.y;
            //                addPoint({x, y});
            //            }
            //            name = "Bezier";
            //        }
            //
            //        // Debug print
            //        void print() const {
            //            printf("Shape: %s\n", name.c_str());
            //            for (const auto& pt : points) {
            //                printf("  (%.4f, %.4f)\n", pt.x, pt.y);
            //            }
            //        }
    };
    
    
};

#endif /* jam_ilda_hpp */
