    /// Definition of the jam.ilda.manager data structures.
    /// Unlike the convention this is not done in the c file, because other object files need acces to this as well
#include "c74_min_api.h"
#include <map>
#include <string>
#include <mutex>



#ifndef helios_manager_h
#define helios_manager_h

typedef struct _jam_hm
{
    std::mutex _file_access_lock;
    c74::max::t_object s_obj;     // t_object
    

} t_jam_hm;
#endif


