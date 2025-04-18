    /// Definition of the jam.ilda.manager data structures.
    /// Unlike the convention this is not done in the c file, because other object files need acces to this as well
#include "c74_min_api.h"
#include <mutex>
#include <string>
#include <vector>
#include "../jam.dmxusbpro.connector/jam.dmxusbpro.connector.hpp"



#ifndef jam_dmxusbpro_manager_h
#define jam_dmxusbpro__manager_h

typedef struct _jam_dmxdm
{
    std::mutex _file_access_lock;
    c74::max::t_object s_obj;     // t_object header
    Connector * _connector = nullptr;
    
} t_jam_dmxdm;
#endif //jam_dmxusbpro_manager_h


    


