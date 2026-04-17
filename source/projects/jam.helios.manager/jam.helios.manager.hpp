    /// Definition of the jam.ilda.manager data structures.
    /// Unlike the convention this is not done in the c file, because other object files need acces to this as well
#include "c74_min_api.h"
#include <map>
#include <string>
#include <mutex>
#include "../jam.helios.connector/jam.helios.connector.hpp"
#include "InterfaceHeliosListener.hpp"



#ifndef helios_manager_h
#define helios_manager_h

typedef struct _jam_hm
{
    c74::max::t_object s_obj;     // t_object
    jam::helios::Connector * _connector = nullptr;
    
} t_jam_hm;
#endif


