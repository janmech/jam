/// This is global max objext that shall be loaded as an extension by placing it inside the extension folder of the package
/// this is a frankenstein-ish patching a traditional c-style max object into the project, because it seems that it is not possible to create nobox object with min-defkit

#include "jam.ilda.manager.hpp"

using namespace c74::max;

C74_EXPORT


    // function prototypes
void *jam_im_new(t_symbol *s, long argc, t_atom *argv);
t_jam_im *  jam_im_get_struct(t_jam_im *x);



static t_class *s_jam_im_class; // global pointer to our class definition that is setup in ext_main()


void ext_main(void *r)
{
    t_class *c;
    c = class_new("jam.ilda.manager", (method)jam_im_new, (method)NULL, sizeof(t_jam_im), 0L, 0);
    class_addmethod(c, (method)jam_im_get_struct, "get_struct", 0);
    class_register(CLASS_NOBOX, c);
    s_jam_im_class = c;
}

void *jam_im_new(t_symbol *s, long argc, t_atom *argv)
{
    static t_jam_im *x = NULL;
    if(x == NULL) {
        x = (t_jam_im *)object_alloc(s_jam_im_class);
    }
    return x;
}

t_jam_im *  jam_im_get_struct(t_jam_im *x) {
    return x;
};


