/**
 * Virtual interface for implementing loggers in Cadmium 2
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

#ifndef CADMIUM_CORE_LOGGER_LOGGER_HPP_
#define CADMIUM_CORE_LOGGER_LOGGER_HPP_

#include <mutex>
#include <string>

namespace cadmium {
	//! Cadmium Logger abstract class.
	class Logger {
	 private:
#ifndef RT_ESP32
		std::mutex mutex;  //!< Mutex for enabling a good parallel execution.
#endif
	 public:
		//! Constructor function.
#ifndef RT_ESP32
		Logger(): mutex() {}
#endif
		// if RT_ESP32 then use default constructor

		//! Destructor function.
		virtual ~Logger() = default;

		//! It locks the logger mutex.
		inline void lock() {
#ifndef RT_ESP32
			mutex.lock();
#endif
		}

		//! It unlocks the logger mutex.
		inline void unlock() {
#ifndef RT_ESP32
			mutex.unlock();
#endif
		}

		//! Virtual method to execute any task prior to the simulation required by the logger.
		virtual void start() = 0;

		//! Virtual method to execute any task after the simulation required by the logger.
		virtual void stop() = 0;

		/**
		 * Virtual method to log the simulation time after a simulation step. By default, it does nothing.
		 * @param time new simulation time.
		 */
		virtual void logTime(double time) {}

		/**
		 * Virtual method to log atomic models' output messages.
		 * @param time current simulation time.
		 * @param modelId ID of the model that generated the output message.
		 * @param modelName name of the model that generated the output message.
		 * @param portName name of the model port in which the output message was created.
		 * @param output string representation of the output message.
		 */
		virtual void logOutput(double time, long modelId, const std::string& modelName, const std::string& portName, const std::string& output) = 0;

		/**
		 * Virtual method to log atomic models' states.
		 * @param time current simulation time.
		 * @param modelId ID of the model that generated the output message.
		 * @param modelName name of the model that generated the output message.
		 * @param state string representation of the state.
		 */
		virtual void logState(double time, long modelId, const std::string& modelName, const std::string& state) = 0;
	};
}

#endif //CADMIUM_CORE_LOGGER_LOGGER_HPP_
