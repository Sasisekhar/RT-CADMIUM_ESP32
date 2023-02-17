

/**
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2021  Román Cárdenas Rodríguez
 * ARSLab - Carleton University
 * GreenLSI - Polytechnic University of Madrid
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef RT_EXECUTE
#define RT_EXECUTE
#endif

#ifndef CADMIUM_CORE_SIMULATION_ROOT_COORDINATOR_HPP_
#define CADMIUM_CORE_SIMULATION_ROOT_COORDINATOR_HPP_

#include <limits>
#include <memory>
#include <utility>
#include <vector>
#include "coordinator.hpp"

// #ifndef RT_ARM_MBED
#ifndef NO_LOGGING
	#include "../logger/logger.hpp"
#endif

#include "../real_time/rt_clock.hpp"

extern "C" { 

namespace cadmium {
	//! Root coordinator class.
    class RootCoordinator {
     private:
        std::shared_ptr<Coordinator> topCoordinator;  //!< Pointer to top coordinator.
		
//		#ifndef RT_ARM_MBED
		#ifndef NO_LOGGING
			std::shared_ptr<Logger> logger;               //!< Pointer to simulation logger.
			std::shared_ptr<Logger> debugLogger;          //!< Pointer to simulation debug logger.
		#endif
		#ifdef RT_ESP32
			RTClock timmer;
		#endif
		
		void simulationAdvance(double timeNext) {
//			#ifndef RT_ARM_MBED
			#ifndef NO_LOGGING
				if (logger != nullptr) {
					logger->lock();
					logger->logTime(timeNext);
					logger->unlock();
				}
				if (debugLogger != nullptr) {
					debugLogger->lock();
					debugLogger->logTime(timeNext);
					debugLogger->unlock();
				}
			#endif


			topCoordinator->collection(timeNext);
			topCoordinator->transition(timeNext);
			topCoordinator->clear();
		}
     public:

#ifndef NO_LOGGING
		#ifndef RT_ESP32
			RootCoordinator(std::shared_ptr<Coupled> model, double time):
				topCoordinator(std::make_shared<Coordinator>(std::move(model), time)), logger(), debugLogger() {

				}
			explicit RootCoordinator(std::shared_ptr<Coupled> model): RootCoordinator(std::move(model), 0) {
			}
		#else
			RootCoordinator(std::shared_ptr<Coupled> model, double time):
//				topCoordinator(std::make_shared<Coordinator>(std::move(model), time)), timmer(topCoordinator->get_async_subjects()) {}
				topCoordinator(std::make_shared<Coordinator>(std::move(model), time)), logger(), debugLogger(), timmer(topCoordinator->get_async_subjects()) {} // Edited this line to add loggers (EZE)
			explicit RootCoordinator(std::shared_ptr<Coupled> model): RootCoordinator(std::move(model), 0) {
			}
		#endif
#else
		#ifndef RT_ESP32
			RootCoordinator(std::shared_ptr<Coupled> model, double time):
				topCoordinator(std::make_shared<Coordinator>(std::move(model), time)) {

				}
			explicit RootCoordinator(std::shared_ptr<Coupled> model): RootCoordinator(std::move(model), 0) {
			}
		#else
			RootCoordinator(std::shared_ptr<Coupled> model, double time):
//				topCoordinator(std::make_shared<Coordinator>(std::move(model), time)), timmer(topCoordinator->get_async_subjects()) {}
				topCoordinator(std::make_shared<Coordinator>(std::move(model), time)), timmer(topCoordinator->get_async_subjects()) {} // Edited this line to add loggers (EZE)
			explicit RootCoordinator(std::shared_ptr<Coupled> model): RootCoordinator(std::move(model), 0) {
			}
		#endif
#endif

		std::shared_ptr<Coordinator> getTopCoordinator() {
			return topCoordinator;
		}

//		#ifndef RT_ARM_MBED
		#ifndef NO_LOGGING
			void setLogger(const std::shared_ptr<Logger>& log) {
				logger = log;
				topCoordinator->setLogger(log);
			}

			void setDebugLogger(const std::shared_ptr<Logger>& log) {
				debugLogger = log;
				topCoordinator->setDebugLogger(log);
			}
		#endif

		void start() {
//			#ifndef RT_ARM_MBED
			#ifndef NO_LOGGING
				if (logger != nullptr) {
					logger->start();
				}
				if (debugLogger != nullptr) {
					debugLogger->start();
				}
			#endif
			topCoordinator->setModelId(0);
			topCoordinator->start(topCoordinator->getTimeLast());
		}

		void stop() {
			topCoordinator->stop(topCoordinator->getTimeLast());
//			#ifndef RT_ARM_MBED
			#ifndef NO_LOGGING
				if (logger != nullptr) {
					logger->stop();
				}
				if (debugLogger != nullptr) {
					debugLogger->stop();
				}
			#endif
		}

		[[maybe_unused]] void simulate(long nIterations) {
			double timeNext = topCoordinator->getTimeNext();
            while (nIterations-- > 0 && timeNext < std::numeric_limits<double>::infinity()) {
				simulationAdvance(timeNext);
                timeNext = topCoordinator->getTimeNext();
            }
        }

		#ifdef RT_ESP32
		void simulateRT(double timeInterval){
			double timeNext = topCoordinator->getTimeNext(); // the time of the first known event
			timmer.startSimulation(); // reset RT clock to be at 0 seconds
			double currentTime = 0;
			double e;
            while(1) {
				e = timmer.wait_for(timeNext - currentTime);
				if(!timmer.interrupted && e == 0){ // if no interrupt occured
					currentTime = timeNext;
					simulationAdvance(currentTime);	
				}else{ // There was an interupt
					currentTime += e;
                    for(auto s : timmer.getAsyncSubjects()){
						timmer.interrupted = false;
                    	if(s->interrupted){
							s->interrupted = false;
							topCoordinator->inject(e, s->getPort(), true); // insert a message into the right port triggering external event
						}
                    }
				}
				timeNext = topCoordinator->getTimeNext();
            }
		}
		#endif


		[[maybe_unused]] void simulate(double timeInterval) {
			#ifdef RT_ESP32
				simulateRT(timeInterval);
			#else
				double currentTime = 0;
				double e;
				double timeNext = topCoordinator->getTimeNext();
				double timeFinal = topCoordinator->getTimeLast()+timeInterval;
				while(timeNext < timeFinal) {
					// e = timer.wait_for(timeNext - currentTime);
					currentTime = timeNext;
					simulationAdvance(currentTime);
					timeNext = topCoordinator->getTimeNext();
				}
			#endif
        }
    };
}

#endif //CADMIUM_CORE_SIMULATION_ROOT_COORDINATOR_HPP_
}