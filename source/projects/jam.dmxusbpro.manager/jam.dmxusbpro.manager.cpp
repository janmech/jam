/// This is global max objext that shall be loaded as an extension by placing it inside the extension folder of the package
/// this is a frankenstein-ish patching a traditional c-style max object into the project, because it seems that it is not possible to create nobox object with min-defkit

#include "jam.dmxusbpro.manager.hpp"

using namespace c74::max;

C74_EXPORT


    // function prototypes
void *jam_dmxdm_new(t_symbol *s, long argc, t_atom *argv);

Connector *  jam_dmxdm_get_connector(t_jam_dmxdm *dl);



static t_class *s_jam_dmxdm_class; // global pointer to our class definition that is setup in ext_main()


void ext_main(void *r)
{
    t_class *c;
    c = class_new("jam.dmxusbpro.manager", (method)jam_dmxdm_new, (method)NULL, sizeof(t_jam_dmxdm), 0L, 0);
    class_addmethod(c, (method)jam_dmxdm_get_connector, "get_device_list", 0);
    s_jam_dmxdm_class = c;
    class_register(CLASS_NOBOX, c);
}

void *jam_dmxdm_new(t_symbol *s, long argc, t_atom *argv)
{
    static t_jam_dmxdm *x = NULL;
    if(x == NULL) {
        x = (t_jam_dmxdm *)object_alloc(s_jam_dmxdm_class);
        x->_connector = &Connector::get();
    }
    return x;
}

Connector * jam_dmxdm_get_connector(t_jam_dmxdm *x) {
    return x->_connector;
};


