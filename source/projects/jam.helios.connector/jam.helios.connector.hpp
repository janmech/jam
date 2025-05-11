#pragma once


namespace jam::helios {
    
    class Connector {
        
        Connector(const Connector&) = delete;
        
        static Connector & get() {
            static Connector instance;
            return instance;
        }
        
    private:
        Connector() {};
    };
}
