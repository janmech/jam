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
#include "ilda_definitions.hpp"
#include "ilda_header.hpp"
#include "ilda_data_record.hpp"
#include "ilda_section.hpp"

namespace jam::helios {
    class IldaFileProcessor {
    public:
        IldaFileProcessor() {}
        IldaFileProcessor(std::vector<char> ilda_file) {
            this->_ilda_file = ilda_file;
        }
        
        void setFileData(std::vector<char>ilda_file);
        void clearFileData();
        ParseResult parseFileData();
        bool fileLoaded();
        
        
    protected:
        
        bool _fileLoaded = false;
        std::vector<char> _ilda_file;
        std::vector<IldaSection> _ilda_sections;
        
        void _readHeaderSection(char* buffer, size_t start, size_t byte_count);
        std::uint16_t _parseUint16(char* bytes, std::size_t byte_count);
        ParseResult _extractSection(IldaSection &section, size_t *byte_index);
        size_t _getRecordByteSize(RecordFormat format);
        
    };
}
#endif /* ilda_file_processor_h */
