# How to build the project
## Building the Xcode project:

**1.** cd into the project folder:

``cd jam``

**2.** create the 'build' directory:
``mkdir build``

**3.**  `` cd build`` and run ``cmake cmake -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -G Xcode ..`` 

## Manual steps to configure the build targets:

> I haven't figuered out yet how to automize these steps with cmake. For now there steps have to be done manually:

### Target: *jam.dmxusbpro*
Select the tab *Build Phases*. In the section *Link Binary With Libraries* add:

* 	CoreFoundation.framework
* 	IOKit.framework

### Target: *jam.dmxusbpro_tilde*
Select the tab *Build Phases*. In the section *Link Binary With Libraries* add:

* 	CoreFoundation.framework
* 	IOKit.framework

### Target: *jam.helios*
Select the tab *Build Phases*. In the section *Link Binary With Libraries* add:

* 	CoreFoundation.framework
* 	IOKit.framework
*  Security.framework

