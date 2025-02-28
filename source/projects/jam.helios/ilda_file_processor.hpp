    //
    //  ilda_file_processor.hpp
    //  jam.helios
    //
    //  Created by Jan Mech on 27/2/25.
    //

#ifndef ilda_file_processor_h
#define ilda_file_processor_h

#include <vector>
#include <cstddef>

namespace jam {
    
#define FILE_HEADER_SIZE 32
    
#define FILE_HEADER_ILDA_TAG_START 0
#define FILE_HEADER_ILDA_TAG_END 3
    
#define FILE_HEADER_RESERVED_START 4
#define FILE_HEADER_RESERVED_END 6
    
#define FILE_HEADER_FORMAT_CODE_START 7
#define FILE_HEADER_FORMAT_CODE_END 7
    
#define FILE_HEADER_FRAME_NAME_START 8
#define FILE_HEADER_FRAME_NAME_END 15
    
#define FILE_HEADER_COMPANY_NAME_START 16
#define FILE_HEADER_COMPANY_NAME_END 23
    
#define FILE_HEADER_NUMBER_OF_RECODRS_START 24
#define FILE_HEADER_NUMBER_OF_RECODRS_END 25
    
#define FILE_HEADER_FRAME_NUMBER_START 26
#define FILE_HEADER_FRAME_NUMBER_END 27
    
#define FILE_HEADER_FRAMES_IN_SEQUENCE_START 28
#define FILE_HEADER_FRAMES_IN_SEQUENCE_END 29
    
#define FILE_HEADER_PROJECTOR_NUMBER_START 30
#define FILE_HEADER_PROJECTOR_NUMBER_END 30
    
    typedef struct header {
        uint8_t formatCode = 0;
        std::string frameName = "";
        std::string companyName = "";
        std::uint16_t recordCount = 0;
        std::uint16_t frameNumber = 0;
        std::uint16_t framesInSequence = 0;
        bool isColorPallet = false;
        
    } ilda_header_t;
    
    class IldaFileProcessor {
    public:
        IldaFileProcessor() {}
        IldaFileProcessor(std::vector<char> ilda_file) {
            this->_ilda_file = ilda_file;
        }
        
        bool setAndParseIldaFile(std::vector<char>ilda_file);
        void clearFileData();
        bool fileLoaded();
        ilda_header_t &getFileHeader();
        
        
        
        
    protected:
        bool _fileLoaded = false;
        ilda_header_t _header;
        std::vector<char> _ilda_file;
        void _readHeaderSection(char* buffer, uint8_t start, uint8_t end);
        std::uint16_t _parseUint16(char* bytes, std::size_t byte_count);
        bool _parseHeader();
        
    };
}
#endif /* ilda_file_processor_h */
