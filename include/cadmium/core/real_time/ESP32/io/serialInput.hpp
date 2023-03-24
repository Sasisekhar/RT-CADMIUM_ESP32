/**
* Sasisekhar Mangalam Govind
* ARSLab - Carleton University
* SENSE  - VIT Chennai
*
* Serial Input:
* Model to interface with a Serial port for Embedded Cadmium.
*/

#ifndef SERIALINPUT_HPP
#define SERIALINPUT_HPP

#include <stdio.h>
// #include <optional>
#include "../../../modeling/atomic.hpp"
// #include <limits>
// #include <math.h> 
// #include <assert.h>
// #include <memory>

#ifndef NO_LOGGING
	// #include <iomanip>
	#include <iostream>
	// #include <fstream>
#endif

// #include <chrono>
// #include <algorithm>
// #include <limits>
// #include <random>

#ifdef RT_ESP32
    #include <stdio.h>
    #include <string.h>
    #include "esp_system.h"
    #include "esp_console.h"
    #include "esp_vfs_dev.h"
    #include "esp_vfs_fat.h"
    #include "driver/uart.h"
#endif

using namespace std;

namespace cadmium {
  
  struct SerialInputState {
      float output;
      float last;
      double sigma;

      /**
      * Processor state constructor. By default, the processor is idling.
      * 
      */
      explicit SerialInputState(): output(0.0), last(0.0), sigma(2){
      }

  }; 

#ifndef NO_LOGGING
  /**
     * Insertion operator for ProcessorState objects. It only displays the value of sigma.
     * @param out output stream.
     * @param s state to be represented in the output stream.
     * @return output stream with sigma already inserted.
     */
    std::ostream& operator<<(std::ostream &out, const SerialInputState& state) {
        out << "Input: " << state.output; 
        return out;
    }
#endif

  class SerialInput : public Atomic<SerialInputState> {
      public:
      
        Port<float> out;

        // default constructor
        SerialInput(const std::string& id): Atomic<SerialInputState>(id, SerialInputState())  {
          out = addOutPort<float>("out");
          scanf("%f", &state.output);
        };
      
      // internal transition
      void internalTransition(SerialInputState& state) const override {
        state.last = state.output;
        float output = state.last;
        scanf("%f", &output);
        state.output = output;
      }

      // external transition
      void externalTransition(SerialInputState& state, double e) const override {
        // MBED_ASSERT(false);
        // throw std::logic_error("External transition called in a model with no input ports");
      }
      
      // output function
      void output(const SerialInputState& state) const override {
        if(state.last != state.output) {
          float output = state.output;
          out->addMessage(output);
        }
      }

      // time_advance function
      [[nodiscard]] double timeAdvance(const SerialInputState& state) const override {     
          return state.sigma;
      }

  };
}

#endif