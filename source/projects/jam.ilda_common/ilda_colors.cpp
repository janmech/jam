//
//  ilda_colors.cpp
//  jam
//
//  Created by Jan Mech on 8/3/25.
//

#include "ilda_colors.hpp"

namespace jam::ilda {
    
     std::vector<float> Colors::getFloatColorByIndex(uint8_t color_index) {
         color_index = (color_index > 63) ? 63 : color_index;
        std::vector<float> color_vector;
        int offset = (int)color_index * 3;
         color_vector.push_back((float)Colors::ilda_color_pallet_float[offset]);
        color_vector.push_back((float)Colors::ilda_color_pallet_float[offset+1]);
        color_vector.push_back((float)Colors::ilda_color_pallet_float[offset+2]);
        return color_vector;
    };
    
};
