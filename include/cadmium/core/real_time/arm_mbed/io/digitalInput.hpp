/**
* Jon Menard
* ARSLab - Carleton University
*
* Digital Input:
* Model to interface with a digital Input pin for Embedded Cadmium.
*/

#ifndef RT_DIGITALINPUT_TEST_HPP
#define RT_DIGITALINPUT_TEST_HPP

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
  
  struct DigitalInputState {
      bool output;
      bool last;
      double sigma;

      /**
      * Processor state constructor. By default, the processor is idling.
      * 
      */
      explicit DigitalInputState(): output(true), last(false), sigma(0){
      }

  }; 

#ifndef NO_LOGGING
  /**
     * Insertion operator for ProcessorState objects. It only displays the value of sigma.
     * @param out output stream.
     * @param s state to be represented in the output stream.
     * @return output stream with sigma already inserted.
     */
    std::ostream& operator<<(std::ostream &out, const DigitalInputState& state) {
        out << "Pin: " << (state.output ? 1 : 0); 
        return out;
    }
#endif

  class DigitalInput : public Atomic<DigitalInputState> {
      public:
      
        Port<bool> out;
        //Parameters to be overwriten when instantiating the atomic model
        gpio_num_t digiPin;
        double   pollingRate;
        // default constructor
        DigitalInput(const std::string& id, int pin): Atomic<DigitalInputState>(id, DigitalInputState())  {
          out = addOutPort<bool>("out");
          digiPin = (gpio_num_t) pin;
          gpio_reset_pin(digiPin);
          gpio_set_direction(digiPin, GPIO_MODE_INPUT);
          pollingRate = 0.10; 
          state.output = gpio_get_level(digiPin);
          state.last = state.output;
        };
      
      // internal transition
      void internalTransition(DigitalInputState& state) const override {
        state.last = state.output;
        state.output = gpio_get_level(digiPin);
        state.sigma = pollingRate;
      }

      // external transition
      void externalTransition(DigitalInputState& state, double e) const override {
        // MBED_ASSERT(false);
        // throw std::logic_error("External transition called in a model with no input ports");
      }
      
      // output function
      void output(const DigitalInputState& state) const override {
        if(state.last != state.output) {
          bool output = state.output;
          out->addMessage(output);
        }
      }

      // time_advance function
      [[nodiscard]] double timeAdvance(const DigitalInputState& state) const override {     
          return state.sigma;
      }

  };
} 

#endif // BOOST_SIMULATION_PDEVS_DIGITALINPUT_HPP
