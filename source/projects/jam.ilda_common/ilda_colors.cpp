    //
    //  ilda_colors.cpp
    //  jam
    //
    //  Created by Jan Mech on 8/3/25.
    //

#include "ilda_colors.hpp"

namespace jam::ilda {
    
    std::vector<double> Colors::getFloatColorByIndex(size_t color_index, double opacity) {
        color_index = (color_index > 63) ? 63 : color_index;
        std::vector<double> color_vector;
        color_vector.clear();
        size_t offset = color_index * 3;
        color_vector.push_back(Colors::ilda_color_pallet_float[offset]);      // Red
        color_vector.push_back(Colors::ilda_color_pallet_float[offset + 1]);  // Green
        color_vector.push_back(Colors::ilda_color_pallet_float[offset + 2]);  // Blue
        color_vector.push_back(opacity); // ILDA pallets don`t have an alpha channel
        return color_vector;
    };
    
};
