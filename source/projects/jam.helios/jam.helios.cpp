    /// @file
    ///	@ingroup    jam
    ///	@copyright	Copyright 2018 The Min-DevKit Authors. All rights reserved.
    ///	@license	Use of this source code is governed by the MIT License found in the License.md file.

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include "c74_min.h"
#include "helios-sdk/cpp/HeliosDac.h"
#include "helios-sdk/cpp/libusb.h"

#define OBJECT_VERSION "jam.helios v.0.0.0"
#define HELIOS_FILE_CHUNK 1024


using namespace c74::min;
namespace s_chrono = std::chrono;

class helios : public object<helios>
{
    
protected:
    HeliosDac _helios_dac;
    
public:
    
    helios(const atoms& args = {}) {}
    
    ~helios() {
        this->_helios_dac.CloseDevices();
    }
    
    static constexpr const char* my_description {"foo"};
    
    MIN_DESCRIPTION     { "Connect to the Helios ILDA DAC" };
    
    MIN_TAGS            { "utilities" };
    MIN_AUTHOR          { "Jan Mech" };
    MIN_RELATED         { "jam.dmxusbpro~, jam.dmxusbpro, serial"};
    
    inlet<> input_1    { this, "(anything) Control Messages", "anything" };
    inlet<> input_2     {this, "(dictionary) ilda file dictionary" , "dictionary"};
    outlet<> output_1   { this, "(list) DMX Output <startcode> <channel> <value>", "list" };
    
    message<threadsafe::yes> version {
        this, "version",
        "dev test message",
        MIN_FUNCTION {
            cout << OBJECT_VERSION << endl;
            
            return {};
        }
    };
    
    message<> dictionary {
        this, "dictionary", "Dictionary containing an ILDA file animation for sending to the DAC",
        MIN_FUNCTION {
            if (inlet == 1) {
                dict ilda_file = {args[0]};
                    // Turn the atom_reference from mindict["innerdict"] into an atom
                c74::min::symbol key {"sections"};
                auto section_dict_atom = c74::min::atom(ilda_file[key].begin());
                
                    // Create an unregistered subdict from the atom
                dict sections_dict {section_dict_atom};
                int i = 0;
                try {
                    while(true) {
                        auto section_dict = sections_dict.at(symbol(i));
                        i++;
                        if(1 > 1024) {
                            break;
                        }
                    }
                } catch (std::runtime_error& e) {
                    cerr << "could not fetch key called 'pattern'" << endl;
                }
              
                
                
                bool test = true;
    
                    // Generate a unique name (you could also name it yourself if you want)
                    //                    auto sym = c74::min::symbol(true);
                    //                    subdict.register_as(sym);
                    //                c74::min::atom_reference sections = ilda_file["sections"];
                    //                for(auto section = sections.begin(); section < sections.end();section++ ) {
                    //                    c74::min::atom foo = *section;
                    //                    auto bar = (int)foo;
                    //                }
                
            }
            return {};
        }
    };
    
    message<threadsafe::yes> menu {
        this, "menu", "Get list of connected devices and build menu from it.",
        MIN_FUNCTION {
            if (args.size() > 1) {
                cwarn << "extra argument for message 'menu'" << endl;
            }
            return {};
        }
    };
    
    message<threadsafe::no> test {
        this, "test", "foooo",
        MIN_FUNCTION {
                // Assemble test frames
                // This is a simple line moving upward in a loop, but for real graphics you should optimize the point stream for laser scanners by
                // interpolating long vectors including blanked sections, adding points at sharp corners, etc.
            HeliosPointHighRes** frame = new HeliosPointHighRes*[30];
            const int numPointsPerFrame = 1000;
            const int pointsPerSecond = 30000;
            int x = 0;
            int y = 0;
            for (int i = 0; i < 30; i++)
                {
                frame[i] = new HeliosPointHighRes[numPointsPerFrame];
                y = i * 0xFFFF / 30;
                for (int j = 0; j < numPointsPerFrame; j++)
                    {
                    if (j < (numPointsPerFrame/2))
                        x = j * 0xFFFF / (numPointsPerFrame/2);
                    else
                        x = 0xFFFF - ((j - (numPointsPerFrame / 2)) * 0xFFFF / (numPointsPerFrame / 2));
                    
                    frame[i][j].x = x;
                    frame[i][j].y = y;
                    frame[i][j].r = 0xD0FF;
                    frame[i][j].g = 0xFFFF;
                    frame[i][j].b = 0xD0FF;
                        //frame[i][j].user1 = 0; // Use HeliosPointExt with WriteFrameExtended() if you need more channels
                        //frame[i][j].user2 = 10;
                        //frame[i][j].user3 = 20;
                        //frame[i][j].user4 = 30;
                        //frame[i][j].i = 0xFFFF;
                    }
                }
            
            int numDevs = this->_helios_dac.OpenDevices();
            
            if (numDevs <= 0)
                {
                cout << "No DACs found.\n"<< endl;
                return {};
                }
            printf("Found %d DACs:\n", numDevs);
            for (int j = 0; j < numDevs; j++)
                {
                char name[32];
                if (this->_helios_dac.GetName(j, name) == HELIOS_SUCCESS)
                    printf("- %s: USB?: %d, FW %d\n", name, this->_helios_dac.GetIsUsb(j), this->_helios_dac.GetFirmwareVersion(j));
                else
                    printf("- (unknown dac): USB?: %d, FW %d\n", this->_helios_dac.GetIsUsb(j), _helios_dac.GetFirmwareVersion(j));
                }
            
            
            printf("Outputting animation...\n");
            
            int i = 0;
            while (1)
                {
                i++;
                if (i > 200)
                    {
                    break;
                    }
                
                
                    // Send each frame to the DAC.
                for (int j = 0; j < numDevs; j++)
                    {
                        // Wait for ready status. You must call GetStatus() until it returns 1 before each and every WriteFrame*() call that you do.
                    for (unsigned int k = 0; k < 1024; k++)
                        {
                        int status = this->_helios_dac.GetStatus(j);
                        if (status == 1)
                            {
                            this->_helios_dac.WriteFrameHighResolution(j, pointsPerSecond, HELIOS_FLAGS_DEFAULT, frame[i % 30], numPointsPerFrame);
                            break;
                            }
                        else if (status < 0)
                            {
                            printf("Error when polling status for device #%d: %d\n", j, status);
                            break;
                            }
                        }
                        // In this loop, timing is handled by the GetStatus polling, which only returns 1 once there is room in the DAC to send the next frame.
                        // You need to call WriteFrame*() in time (before the previously written frame finished playing), to not let the buffers in the DAC underrun.
                        // You should also make frames large enough to account for transfer overheads and timing jitter. Frames should be 10 milliseconds or longer on average, generally speaking.
                    }
                }
            
                // Freeing connection when we're done
            this->_helios_dac.CloseDevices();
            
            return {};
        }
    };
    
    
};


MIN_EXTERNAL(helios);
