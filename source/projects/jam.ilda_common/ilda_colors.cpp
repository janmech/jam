    //
    //  ilda_colors.cpp
    //  jam
    //
    //  Created by Jan Mech on 8/3/25.
    //

#include "ilda_colors.hpp"

namespace jam::ilda {
    
    std::vector<double> Colors::getFloatColorByIndex(uint8_t color_index) {
        color_index = (color_index > 63) ? 63 : color_index;
        std::vector<double> color_vector;
        int offset = (int)(color_index) * 3;
        color_vector.push_back(Colors::ilda_color_pallet_float[offset]);
        color_vector.push_back(Colors::ilda_color_pallet_float[offset+1]);
        color_vector.push_back(Colors::ilda_color_pallet_float[offset+2]);
        color_vector.push_back(1.); // ILDA pallets don`t have an alpha channel
        return color_vector;
    };
    
};
