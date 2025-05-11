    /// @file
    ///	@ingroup    jam
    ///	@copyright	Copyright 2018 The Min-DevKit Authors. All rights reserved.
    ///	@license	Use of this source code is governed by the MIT License found in the License.md file.

#include <cstddef>
#include <queue>
#include <vector>
#include <string>
#include "c74_min.h"

using namespace c74::min;

class heliosinfo : public object<heliosinfo>
{
    
protected:
    typedef struct QuededMessage {
        outlet<>* out;
        atoms msg_atoms;
        void set(outlet<>* o, atoms ma) {
            this->out = o;
            this->setAtoms(ma);
        }
        
        void setAtoms(atoms ma) {
            this->msg_atoms.clear();
            this->msg_atoms = ma;
        }
        
        void send(heliosinfo* me) {
            me->_enqueue_msg_to_max(*this);
            me->deliverer_to_max.delay(0);
        }
    } queued_message_t;
    
    fifo<queued_message_t> _to_max_queue_2{ 1000 };
    std::mutex _enqueue_msg_lock;
    
    
    void _enqueue_msg_to_max(const queued_message_t& msg_to_max) {
        _enqueue_msg_lock.lock();
        this->_to_max_queue_2.try_enqueue(msg_to_max);
        _enqueue_msg_lock.unlock();
    }
    
    bool _dequeue_msg_to_max(queued_message_t& msg_data) {
        _enqueue_msg_lock.lock();
        bool result = this->_to_max_queue_2.try_dequeue(msg_data);
        _enqueue_msg_lock.unlock();
        return result;
    }
    
    
    
public:
    heliosinfo(const atoms& args = {}) {
        if (!dummy()) {
            
        }
    }
    
    ~heliosinfo() {
        if (!dummy()) {
            
        }
    }
    
    MIN_DESCRIPTION { "Connect to a Helios ILDA DAC" };
    MIN_TAGS { "laser control" };
    MIN_AUTHOR{ "Jan Mech" };
    MIN_RELATED{ "jam.dmxusbpro~, jam.dmxusbpro" };
    
    inlet<> input_1{ this, "(anything) Control Messages", "anything" };
    outlet<> outlet_menu{ this, "(anything) Connect to umenu", "message" };
    
    
    timer<> deliverer_to_max {
        this, MIN_FUNCTION {
            queued_message_t queue_msg;
            
            while (_dequeue_msg_to_max(queue_msg)) {
                queue_msg.out->send(queue_msg.msg_atoms);
            }
            return {};
        }
    };
};

MIN_EXTERNAL(heliosinfo);
