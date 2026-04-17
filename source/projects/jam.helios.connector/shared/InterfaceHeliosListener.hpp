#pragma once // Standard include guard

namespace jam::helios {
    class InterfaceHeliosListener {
    public:
        virtual ~InterfaceHeliosListener() {}
    
        virtual void onConnectionReset() = 0;

    };
}

