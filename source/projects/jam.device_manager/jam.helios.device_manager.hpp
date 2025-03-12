    //
    //  jam.helios.hpp
    //  jam.helios
    //
    //  Created by Jan Mech on 12/3/25.
    //

#pragma once

#include <stdio.h>
#include <iostream>
#include <string>
#include "../jam.helios/helios-sdk/cpp/HeliosDac.h"
#include "../jam.helios/helios-sdk/cpp/libusb.h"

namespace jam::helios {
    
    class HeliosDeviceManager{
        
    public:
        
        HeliosDeviceManager(const HeliosDeviceManager&) = delete;
        ~HeliosDeviceManager() {
            
        }
        
        static HeliosDeviceManager & get() {
            static HeliosDeviceManager instance;
            return instance;
        }
        
        int test() {
            return 100;
        }
    private:
        HeliosDeviceManager() {
            bool test = false;
        }
    };
    
}


