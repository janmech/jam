    //
    //  attribute_args_helper.hpp
    //  jam
    //
    //  Created by Jan Mech on 21/3/25.
    //

#ifndef attribute_args_helper_hpp
#define attribute_args_helper_hpp

#include <stdio.h>
#include "c74_min_api.h"

namespace jam {
    using namespace c74::min;
    
    enum ArgVectSize {
        correct,
        too_short,
        too_long
    };
    
    template <typename T> ArgVectSize checkAndFillAttrArgs(const atoms &args, atoms * cleaned_args, int required_count, T fill)
    {
    ArgVectSize result = ArgVectSize::correct;
    if(args.size() < required_count) {
        result = ArgVectSize::too_short;
    };
    if(args.size() > required_count) {
        result = ArgVectSize::too_long;
    };
    
    for(size_t i = 0; i < args.size(); i++) {
        cleaned_args->push_back((T)args[i]);
        if (i == required_count - 1) {
            break;
        }
    }
    while(cleaned_args->size() < required_count) {
        cleaned_args->push_back((T)fill);
    }
    
    return result;
    }
    
}

#endif /* attribute_args_helper_hpp */
