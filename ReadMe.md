# jam - Max Package
The jam package is a collections of externals for [Cycling'74 Max](https://cycling74.com) for working to interface with a ENTTEC DMX USB Pro DMX controller and to work with shoe laser projectors that have an ILDA interface.

List of externals:

* `jam.dmxusbpro`- Connect to the ENTTEC DMX USB Pro interface. Conrol DMX data with lists.
* `jam.dmxusbpro~` - Connect to the ENTTEC DMX USB Pro interface. Conrol DMX data with signals.
* `jam.helios` - Connect to a Helios ILDA DAC
* `jam.ilda.compose` - Create and mofify ILDA files for laser animation.
* `jam.ilda.dict` - Create a dictionary from an ILDA file with file information
* `jam.ilda.file` - Load an ILDA file (laser animation file) from disk.
* `jam.jit.gl.ilda.sketch` - Render frames from an ILDA file to an Open GL context.





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

### Target: *jam.dmxusbpro.manager*
Select the tab *Build Phases*. In the section *Link Binary With Libraries* add:

* 	CoreFoundation.framework
* 	IOKit.framework

### Target: *jam.helios*
Select the tab *Build Phases*. In the section *Link Binary With Libraries* add:

* 	CoreFoundation.framework
* 	IOKit.framework
*  Security.framework

### Target: *jam.ilda.compose*
Select the tab *Build Phases*. In the section *Link Binary With Libraries* add:

* 	CoreFoundation.framework
* 	CoreText.framework

