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
#include "jam.helios.device_manager.hpp"

#define OBJECT_VERSION "jam.helios v.0.0.0"
#define HELIOS_FILE_CHUNK 1024

using namespace c74::min;

class helios : public object<helios>
{
    
protected:
    HeliosDac _helios_dac;
    std::thread _device_scan_thread;
    bool is_scanning = false;
    fifo<atoms> _to_max_queue { 1000 };
    std::mutex _enqueue_msg_lock;
    jam::helios::DeviceManager & _deviceManager = jam::helios::DeviceManager::get();
    
    void _enqueue_msg_to_max(const atoms &msg_to_max) {
        _enqueue_msg_lock.lock();
        this->_to_max_queue.try_enqueue(msg_to_max);
        _enqueue_msg_lock.unlock();
    }

    bool _dequeue_msg_to_max(atoms &msg_data) {
        _enqueue_msg_lock.lock();

        bool result = this->_to_max_queue.try_dequeue(msg_data);

        _enqueue_msg_lock.unlock();
        return result;
    }
    
    
public:
    
    helios(const atoms& args = {}) {
        this->_deviceManager.addObjInstance(this->maxobj());
    }
    
    ~helios() {
        this->_deviceManager.removeObjInstance(this->maxobj());
//        this->_helios_dac.CloseDevices();
    }
    
    static constexpr const char* my_description {"foo"};
    
    MIN_DESCRIPTION     { "Connect to a Helios ILDA DAC" };
    
    MIN_TAGS            { "utilities" };
    MIN_AUTHOR          { "Jan Mech" };
    MIN_RELATED         { "jam.dmxusbpro~, jam.dmxusbpro"};
    
    inlet<> input_1             { this, "(anything) Control Messages", "anything" };
    inlet<> input_2             { this, "(dictionary) ilda file dictionary" , "dictionary"};
    outlet<> output_dev_menu    { this, "(anything) Connect to umenu", "message"};
    outlet<> output_dumpout     { this, "dumpout"};
    
    attribute<number> m_duration { this, "duration", 3000.0, description {"Duration of the process."} };
    
    message<> dictionary {
        this, "dictionary", "Dictionary containing an ILDA file animation for sending to the DAC",
        MIN_FUNCTION {
            if (inlet == 1) {
                dict ilda_file = {args[0]};
                    // Turn the atom_reference from mindict["innerdict"] into an atom
                c74::min::symbol key {"frames"};
                auto frames_dict_atom = c74::min::atom(ilda_file[key].begin());
                
                    // Create an unregistered subdict from the atom
                dict framess_dict {frames_dict_atom};
                int i = 0;
                try {
                    while(true) {
                        auto frame_dict = framess_dict.at(symbol(i));
                        i++;
                        if(1 > 1024) {
                            break;
                        }
                    }
                } catch (std::runtime_error& e) {
                    cerr << "could not fetch key called 'pattern'" << endl;
                }
                
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
            std::vector<jam::helios::device_info_t> open_devices = this->_deviceManager.getOpenDevices();
            output_dev_menu.send("clear");
            atoms out_atoms;
            out_atoms.push_back("append");
            out_atoms.push_back("(Select Interface)");
            output_dev_menu.send(out_atoms);
            out_atoms.clear();
            for(size_t i = 0; i < open_devices.size(); i++) {
                out_atoms.push_back("append");
                out_atoms.push_back(open_devices[i].name);
                output_dev_menu.send(out_atoms);
            }
            return {};
        }
    };
    
    message<threadsafe::yes> devicescan {
        this, "devicescan", "Scan for connected Helios DACs",
        MIN_FUNCTION {
            if (args.size() > 1) {
                cwarn << "extra argument for message 'menu'" << endl;
            }
            if(this->_deviceManager.isScanning()) {
                cwarn << "scan already in progress" << endl;
                return {};
            }
            
            this->_device_scan_thread = std::thread([this]() {
                if(this->is_scanning) {
                    cwarn << "scan already in progress" << endl;
                    return;
                }
                atoms scan_result;
                this->is_scanning = true;
                
                auto b = this->box();
                number current_progress {-1.};
                b("startprogress", &current_progress);
                int numDevs = this->_deviceManager.deviceScan();
                scan_result.clear();
                scan_result.push_back(atom("devicescan"));
                scan_result.push_back(atom(numDevs));
                _enqueue_msg_to_max(scan_result);
                this->deliverer_to_max.delay(0);
                this->_helios_dac.CloseDevices();
                b("stopprogress");
                this->is_scanning = false;
                
            });
            this->_device_scan_thread.detach();
            
            return {};
        }
    };
    
    message<threadsafe::no> test {
        this, "test", "test function for dev stuff",
        MIN_FUNCTION {
           
                // Assemble test frames
                // This is a simple line moving upward in a loop, but for real graphics you should optimize the point stream for laser scanners by
                // interpolating long vectors including blanked sections, adding points at sharp corners, etc.
            
            cout << this->_deviceManager.deviceScan() << endl;
            cout << "TEST" << endl;
            

       
            return {};
                //            int numDevs = this->_helios_dac.OpenDevices();
                //
                //            if (numDevs <= 0)
                //                {
                //                cout << "No DACs found.\n"<< endl;
                //                return {};
                //                }
                //            cout << "Found" << numDevs << "DAC()s" << endl;
                //            for (int j = 0; j < numDevs; j++)
                //                {
                //                char name[32];
                //                if (this->_helios_dac.GetName(j, name) == HELIOS_SUCCESS)
                //                    cout << name << ": USB: " << this->_helios_dac.GetIsUsb(j) << "Firmware: " << this->_helios_dac.GetFirmwareVersion(j) << endl;
                //                else
                //                    cout << "(unknown dac): USB: " << this->_helios_dac.GetIsUsb(j) << "Firmware: " << this->_helios_dac.GetFirmwareVersion(j) << endl;
                //                }
                //
                //
                //
                //            cout << "Outputting animation..." << endl;
                //
                //            HeliosPointHighRes** frame = new HeliosPointHighRes*[30];
                //            const int numPointsPerFrame = 1000;
                //            const int pointsPerSecond = 30000;
                //            int x = 0;
                //            int y = 0;
                //            for (int i = 0; i < 30; i++)
                //                {
                //                frame[i] = new HeliosPointHighRes[numPointsPerFrame];
                //                y = i * 0xFFFF / 30;
                //                for (int j = 0; j < numPointsPerFrame; j++)
                //                    {
                //                    if (j < (numPointsPerFrame/2))
                //                        x = j * 0xFFFF / (numPointsPerFrame/2);
                //                    else
                //                        x = 0xFFFF - ((j - (numPointsPerFrame / 2)) * 0xFFFF / (numPointsPerFrame / 2));
                //
                //                    frame[i][j].x = x;
                //                    frame[i][j].y = y;
                //                    frame[i][j].r = 0xD0FF;
                //                    frame[i][j].g = 0xFFFF;
                //                    frame[i][j].b = 0xD0FF;
                //                        //frame[i][j].user1 = 0; // Use HeliosPointExt with WriteFrameExtended() if you need more channels
                //                        //frame[i][j].user2 = 10;
                //                        //frame[i][j].user3 = 20;
                //                        //frame[i][j].user4 = 30;
                //                        //frame[i][j].i = 0xFFFF;
                //                    }
                //                }
                //
                //            int i = 0;
                //            while (1)
                //                {
                //                i++;
                //                if (i > 200)
                //                    {
                //                    break;
                //                    }
                //
                //
                //                    // Send each frame to the DAC.
                //                for (int j = 0; j < numDevs; j++)
                //                    {
                //                        // Wait for ready status. You must call GetStatus() until it returns 1 before each and every WriteFrame*() call that you do.
                //                    for (unsigned int k = 0; k < 1024; k++)
                //                        {
                //                        int status = this->_helios_dac.GetStatus(j);
                //                        if (status == 1)
                //                            {
                //                            this->_helios_dac.WriteFrameHighResolution(j, pointsPerSecond, HELIOS_FLAGS_DEFAULT, frame[i % 30], numPointsPerFrame);
                //                            break;
                //                            }
                //                        else if (status < 0)
                //                            {
                //                            cwarn << "Error when polling status for device #" << j << "status: " << status << endl;
                //                            break;
                //                            }
                //                        }
                //                        // In this loop, timing is handled by the GetStatus polling, which only returns 1 once there is room in the DAC to send the next frame.
                //                        // You need to call WriteFrame*() in time (before the previously written frame finished playing), to not let the buffers in the DAC underrun.
                //                        // You should also make frames large enough to account for transfer overheads and timing jitter. Frames should be 10 milliseconds or longer on average, generally speaking.
                //                    }
                //                }
                //
                //                // Freeing connection when we're done
                //            this->_helios_dac.CloseDevices();
                //            this->_helios_dac.SetShutter(1, true);
                //
                //            return {};
        }
    };
    
    timer<> deliverer_to_max {
        this, MIN_FUNCTION {
            atoms queue_data;
            
            while (_dequeue_msg_to_max(queue_data)) {
                atoms message;
                
                for(std::size_t i = 0; i < queue_data.size(); i++) {
                    message.push_back(queue_data[i]);
                }
                output_dumpout.send(message);
            }
            return {};
        }
    };
    
};


MIN_EXTERNAL(helios);
