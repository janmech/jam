    //
    //  ilda_data_record.hpp
    //  jam.helios
    //
    //  Created by Jan Mech on 1/3/25.
    //

#ifndef ilda_data_record_hpp
#define ilda_data_record_hpp

#include <stdio.h>
#include <cstddef>
#include "ilda_definitions.hpp"
#include "ilda_header.hpp"

namespace jam::ilda {
    class IldaDataRecord {
    public:
        
        IldaDataRecord(
                   int pos_x = 0, int pos_y = 0, int pos_z = 0,
                   uint8_t red = 0, uint8_t green = 0, uint8_t blue = 0,
                   uint8_t color_index = 0,
                   bool last_point = false, bool blanking = false
                   ): red(red), green(green), blue(blue), color_index(color_index), last_point(last_point), blanking(blanking) {
                       this->setPosX(pos_x);
                       this->setPosY(pos_y);
                       this->setPosZ(pos_z);
                   };
        
        
        void setPosX(int pos_x);
        int getPosX();
        void setPosY(int pos_y);
        int getPosY();
        void setPosZ(int pos_z);
        int getPosZ();
        void setColorIndex(uint8_t color_index);
        uint8_t getColorIndex();
        void setRed(uint8_t red);
        uint8_t getRed();
        void setGreen(uint8_t green);
        uint8_t getGreen();
        void setBlue(uint8_t blue);
        uint8_t getBlue();
        void setLastPoint(bool last_point);
        bool getLastPoint();
        void setBlanking(bool blanking);
        bool getBlanking();
        void parseStatusByte(uint8_t status_byte);
        
    protected:
        int pos_x = 0;
        int pos_y = 0;
        int pos_z = 0;
        uint8_t red = 0;
        uint8_t green = 0;
        uint8_t blue = 0;
        uint8_t color_index = 0;
        bool last_point = false;
        bool blanking = false;
        
        private:
        int _clipPositionValue(int pos);
        
        
        
        
    };
};

#endif /* ilda_data_record_hpp */
