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
#include "ilda_file_processor.hpp"

#define OBJECT_VERSION "jam.helios v.0.0.0"
#define HELIOS_FILE_CHUNK 1024


using namespace c74::min;
namespace s_chrono = std::chrono;

class helios : public object<helios>
{
    
private:
    dict _d_file_info{symbol(true)};
    dict _d_file_info_header{symbol(true)};
    
protected:
    HeliosDac _helios_dac;
    jam::ilda::IldaFileProcessor _fileProcessor;
    
    void _fileToDict(std::string file_name) {
        this->_d_file_info["file_name"] = file_name;
        this->_d_file_info_header["format_code"] = this->_fileProcessor.getFileHeader().formatCode;
        this->_d_file_info_header["frame_name"] = this->_fileProcessor.getFileHeader().frameName;
        this->_d_file_info_header["company_name"] = this->_fileProcessor.getFileHeader().companyName;
        this->_d_file_info_header["record_count"] = this->_fileProcessor.getFileHeader().recordCount;
        this->_d_file_info_header["frame_number"] = this->_fileProcessor.getFileHeader().frameNumber;
        this->_d_file_info_header["frames_in_sequence"] = this->_fileProcessor.getFileHeader().framesInSequence;
        this->_d_file_info_header["is_color_pallet"] = this->_fileProcessor.getFileHeader().isColorPallet;
    }
    
    
public:
    
    helios(const atoms& args = {}) {
        _d_file_info["file_name"] = "";
        _d_file_info["header"] = _d_file_info_header;
    }
    
    ~helios() {
        this->_helios_dac.CloseDevices();
    }
    
    static constexpr const char* my_description {"foo"};
    
    MIN_DESCRIPTION     { "Connect to the Helios ILDA DAC" };
    
    MIN_TAGS            { "utilities" };
    MIN_AUTHOR          { "Jan Mech" };
    MIN_RELATED         { "jam.dmxusbpro~, jam.dmxusbpro, serial"};
    
    inlet<> input_1    { this, "(anything) Control Messages", "anything" };
    outlet<> output_1   { this, "(list) DMX Output <startcode> <channel> <value>", "list" };
    outlet<> output_2   { this, "Dictionary with loaded file information", "dictionary" };
    outlet<> output_3   { this, "file opration success/failure notification", "list" };
    
    message<threadsafe::yes> version {
        this, "version",
        "dev test message",
        MIN_FUNCTION {
            cout << OBJECT_VERSION << endl;
            
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
    
    message<threadsafe::no>import {
        this, "import", "import an ILDA file",
        MIN_FUNCTION {
            atoms file_message;
            c74::max::t_filehandle ilda_file_handle;
            char filename[c74::max::MAX_PATH_CHARS];
            short path;
            short open_result;
            c74::max::t_fourcc filetype = 'ILDA', outtype;
            
            if (args.size() > 1) {
                cwarn << "extra argument for message 'menu'" << endl;
            }
            if (args.size() == 0) {
                open_result = c74::max::open_dialog(filename, &path, &outtype, &filetype, (short)1);
                if(open_result != 0) {
                    cerr << "Couldn't open file" << endl;
                    file_message.push_back("import");
                    file_message.push_back(filename);
                    file_message.push_back(0);
                    output_3.send(file_message);
                    return {};
                }
            } else {
                std::string user_filename = args[0];
                if(user_filename.size() > c74::max::MAX_PATH_CHARS - 1) {
                    cerr << "file name too long" << endl;
                    file_message.push_back("import");
                    file_message.push_back(filename);
                    file_message.push_back(0);
                    output_3.send(file_message);
                    return {};
                }
                strcpy(filename, user_filename.c_str());
                open_result = c74::max::locatefile_extended(filename, &path, &outtype, &filetype, (short)1);
                if(open_result != 0) {
                    cerr << "Couldn't open file" << endl;
                    file_message.push_back("import");
                    file_message.push_back(filename);
                    file_message.push_back(0);
                    output_3.send(file_message);
                    return {};
                }
            }
            
            open_result = c74::max::path_opensysfile( filename, path, &ilda_file_handle,c74::max::READ_PERM);
            if(open_result != 0) {
                cerr << "Couldn't open file" << endl;
                file_message.push_back("import");
                file_message.push_back(filename);
                file_message.push_back(0);
                output_3.send(file_message);
                return {};
            }
            std::vector<char> ilda_file_bytes;
            char file_buffer[HELIOS_FILE_CHUNK];
            c74::max::t_ptr_size chunk_size = HELIOS_FILE_CHUNK;
            c74::max::t_max_err read_result = 0;
            while(true) {
                read_result = c74::max::sysfile_read(ilda_file_handle,&chunk_size,file_buffer);
                for(size_t i = 0; i < chunk_size; i++) {
                    ilda_file_bytes.push_back(file_buffer[i]);
                }
                if (read_result < 0) {
                    break;
                }
              
            }
            // Set, parse and validate file date
            if(!this->_fileProcessor.setAndParseIldaFile(ilda_file_bytes)) {
                cerr << "Error parsing file data" << endl;
                file_message.push_back("import");
                file_message.push_back(filename);
                file_message.push_back(0);
                output_3.send(file_message);
            }
            this->_fileToDict(std::string(filename));
            file_message.push_back("import");
            file_message.push_back(filename);
            file_message.push_back(1);
            output_3.send(file_message);
            file_message.clear();
            file_message.push_back("bytes");
            file_message.push_back(ilda_file_bytes.size());
            output_3.send(file_message);
            return {};
            
        }
    };
    
    message<threadsafe::no>fileinfo {
        this, "fileinfo", "Get information about the loaded ILDA file.",
        MIN_FUNCTION {
            if(!this->_fileProcessor.fileLoaded()) {
                cwarn << "No file loaded." << endl;
            }
            output_2("dictionary", _d_file_info.name());
            return {};
        }
    };
    
};


MIN_EXTERNAL(helios);
