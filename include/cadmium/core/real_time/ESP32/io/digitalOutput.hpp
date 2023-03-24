/**
* Sasisekhar Mangalam Govind
* ARSLab - Carleton University
* SENSE  - VIT Chennai
*
* Digital Output:
* Model to interface with a digital Output pin for Embedded Cadmium.
*/

#ifndef RT_DIGITALOUTPUT_TEST_HPP
#define RT_DIGITALOUTPUT_TEST_HPP

#ifndef NO_LOGGING
	#include <iostream>
#endif
#include <optional>
#include "../../../modeling/atomic.hpp"

#include <limits>
#include <math.h> 
#include <assert.h>
#include <memory>
#ifndef NO_LOGGING
	#include <iomanip>
	#include <iostream>
	#include <fstream>
#endif
#include <string>
#include <chrono>
#include <algorithm>
#include <limits>
#include <random>

#ifdef RT_ESP32
  #include "driver/gpio.h"
#endif


using namespace std;

namespace cadmium {
  
  struct DigitalOutputState {
      bool output;
      double sigma;

      /**
      * Processor state constructor. By default, the processor is idling.
      * 
      */
      explicit DigitalOutputState(): output(true), sigma(0){
      }

  }; 

#ifndef NO_LOGGING
  /**
     * Insertion operator for ProcessorState objects. It only displays the value of sigma.
     * @param out output stream.
     * @param s state to be represented in the output stream.
     * @return output stream with sigma already inserted.
     */
    std::ostream& operator<<(std::ostream &out, const DigitalOutputState& state) {
        out << "Pin: " << (state.output ? 1 : 0); 
        return out;
    }
#endif
  class DigitalOutput : public Atomic<DigitalOutputState> {
      public:
      
        Port<bool> in;
        //Parameters to be overwriten when instantiating the atomic model
        gpio_num_t digiPin;

        // default constructor
        DigitalOutput(const std::string& id, int pin): Atomic<DigitalOutputState>(id, DigitalOutputState())  {
          in = addInPort<bool>("in");
          digiPin = (gpio_num_t) pin;
          gpio_reset_pin(digiPin);
          gpio_set_direction(digiPin, GPIO_MODE_OUTPUT);
        };
      
      // internal transition
      void internalTransition(DigitalOutputState& state) const override {
      }

      // external transition
      void externalTransition(DigitalOutputState& state, double e) const override {
        if(!in->empty()){
				  for( const auto x : in->getBag()){
					  state.output = x;
				  }

          gpio_set_level(digiPin, state.output ? true : false);
			  }
      }
      
      
      // output function
      void output(const DigitalOutputState& state) const override {
      };

      // time_advance function
      [[nodiscard]] double timeAdvance(const DigitalOutputState& state) const override {     
          return std::numeric_limits<double>::infinity();
      }

  };
}

#endif // RT_DIGITALOUTPUT_TEST_HPP
