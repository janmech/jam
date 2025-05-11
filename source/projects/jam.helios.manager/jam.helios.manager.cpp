/// This is global max objext that shall be loaded as an extension by placing it inside the extension folder of the package
/// this is a frankenstein-ish patching a traditional c-style max object into the project, because it seems that it is not possible to create nobox object with min-defkit

#include "jam.helios.manager.hpp"

using namespace c74::max;

C74_EXPORT


    // function prototypes
void *jam_hm_new(t_symbol *s, long argc, t_atom *argv);
jam::helios::Connector * jam_hm_get_connector(t_jam_hm *x);



static t_class *s_jam_hm_class; // global pointer to our class definition that is setup in ext_main()


void ext_main(void *r)
{
    t_class *c;
    c = class_new("jam.helios.manager", (method)jam_hm_new, (method)NULL, sizeof(t_jam_hm), 0L, 0);
    class_addmethod(c, (method)jam_hm_get_connector, "get_connector", 0);
    s_jam_hm_class = c;
    class_register(CLASS_NOBOX, c);
}

void *jam_hm_new(t_symbol *s, long argc, t_atom *argv)
{
    static t_jam_hm *x = NULL;
    if(x == NULL) {
        x = (t_jam_hm *)object_alloc(s_jam_hm_class);
        x->_connector = &jam::helios::Connector::get();
    }
    return x;
}

jam::helios::Connector * jam_hm_get_connector(t_jam_hm *x) {
    return x->_connector;
};


