# jam - Max Package
The jam package consists of two externals for [Cycling'74 Max](https://cycling74.com) for connecting to and controlling ENTTEC DMX USB Pro devices. `jam.dmxusbpro` and `jam.dmxusbpro~`. The former allows sending DMX data to and receiving them from a connectect interface using Max messages. The latter allows to send DMX data via audio signals.

The project was born out of curiosity, wanting to learn how to write Max externals. I chose to create the DMX USB Pro objects, because the formerly wide spread external by nullmedium ([http://www.nullmedium.de/dev/dmxusbpro/](http://www.nullmedium.de/dev/dmxusbpro/)) is end of life since a while and there are frequently people on the Max Forum looking for a replacement. 




## Contributors / Acknowledgements

The jam package is created with the [Min-DevKit](http://cycling74.github.io/min-devkit/) of Cycling'74 and publisched under The MIT Lisense.


## Installing
Download the [latests release](https://github.com/janmech/jam/releases/) unzip and copy the folder to the Max packages folder.

## System Requirements:
Max: 8.0 or higher

MacOS: 10.15 or higher 

Windows: unsupported 

 
-----------------------------

# How to build the project from scatch
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

