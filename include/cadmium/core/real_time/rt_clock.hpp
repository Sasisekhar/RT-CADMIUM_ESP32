/**
 * Custom Clock generator for ESP32
 * Copyright (C) 2022  Sasisekhar MG
 * ARSLab - Carleton University
 * SENSE  - VIT Chennai
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

#ifndef CADMIUM_RT_CLOCK_HPP
#define CADMIUM_RT_CLOCK_HPP



#ifdef RT_ESP32
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  #include "freertos/queue.h"
  #include "driver/gptimer.h"
  #include "esp_log.h"
  const char* TAG = "rt_clock.hpp";
#endif

#include <chrono>
#ifndef NO_LOGGING
  #include <iostream>
#endif
#include "../exception.hpp"
#include "linux/asynchronous_events.hpp"

extern "C" {

  static long SEC_TO_MICRO   = (1000*1000);

  #ifndef MISSED_DEADLINE_TOLERANCE
    #define MISSED_DEADLINE_TOLERANCE 8000
  #endif
  
  namespace cadmium {
          #ifdef RT_ESP32
          
          class RTClock : public AsyncEventObserver {
          private:

            //Time since last time advance, how long the simulator took to advance
            gptimer_handle_t executionTimer;
            gptimer_handle_t timeoutTimer;
            QueueHandle_t queue;
            bool expired;

            // If the next event (actual_delay) is in the future AKA we are ahead of schedule it will be reset to 0
            // If actual_delay is negative we are behind schedule, in this case we will store how long behind schedule we are in scheduler_slip.
            // This is then added to the next actual delay and updated until we surpass the tolerance or recover from the slip.
            long scheduler_slip = 0;

            //Return a long of time in microseconds
            long get_time_in_micro_seconds(const double t) const {

              //Ignore Anything below 1 microsecond
              return t * SEC_TO_MICRO;
            }

            double micro_seconds_to_time(long us) const {
              return us / SEC_TO_MICRO;
            }

            //Given a long in microseconds, sleep for that time
          
            uint64_t set_timeout(uint64_t delay_us) {
              expired = false;
              uint64_t timeLeft = delay_us;
              gptimer_set_raw_count(executionTimer, 0);
              gptimer_start(executionTimer);

              if(!interrupted && timeLeft > 0) {          //Not tested for wait times over INT_MAX
                gptimer_disable(timeoutTimer);
                gptimer_alarm_config_t alarm_config = {
                  .alarm_count = (uint64_t) timeLeft,
                };
                ESP_ERROR_CHECK(gptimer_set_alarm_action(timeoutTimer, &alarm_config));
                ESP_ERROR_CHECK(gptimer_enable(timeoutTimer));
                ESP_ERROR_CHECK(gptimer_set_raw_count(timeoutTimer, 0));
                ESP_ERROR_CHECK(gptimer_start(timeoutTimer));
                while (!expired && !interrupted) {
                  bool flag = false;
                  if(xQueueReceive(queue, &flag, pdMS_TO_TICKS(2000)) == pdTRUE) {
                    expired = flag;
                  }
                }
              }

              gptimer_stop(executionTimer);
              expired = false;
              if(interrupted) {
                timeLeft = 0;
                gptimer_get_raw_count(executionTimer, &timeLeft);
                if(delay_us < timeLeft ) return 0;
                return delay_us - timeLeft;
              }
              return 0;
            }

        public:

            

            volatile bool interrupted;


            RTClock(std::vector<std::shared_ptr<AsyncEvent>> asyncSubjects): AsyncEventObserver(asyncSubjects){
              
              interrupted = false;

              queue = xQueueCreate(10, sizeof(bool));
              if (!queue) {
                  ESP_LOGE(TAG, "Creating queue failed");
                  return;
              }

              gptimer_config_t timer_config1 = {
                  .clk_src = GPTIMER_CLK_SRC_DEFAULT,
                  .direction = GPTIMER_COUNT_UP,
                  .resolution_hz = 1000000, // 1MHz, 1 tick=1us
              };
              ESP_ERROR_CHECK(gptimer_new_timer(&timer_config1, &executionTimer));

              gptimer_config_t timer_config2 = {
                  .clk_src = GPTIMER_CLK_SRC_DEFAULT,
                  .direction = GPTIMER_COUNT_UP,
                  .resolution_hz = 1000000, // 1MHz, 1 tick=1us
              };
              ESP_ERROR_CHECK(gptimer_new_timer(&timer_config2, &timeoutTimer));

              gptimer_event_callbacks_t cbs = {
                  .on_alarm = cadmium::RTClock::timeout_expired,
              };

              ESP_ERROR_CHECK(gptimer_register_event_callbacks(timeoutTimer, &cbs, queue));
              ESP_ERROR_CHECK(gptimer_enable(executionTimer));
              ESP_ERROR_CHECK(gptimer_enable(timeoutTimer));
            }
            
            double wait_for(const double t) {
              long actual_delay;

              //If negative time, halt and print error over UART
              if(t < 0){
                ESP_LOGE(TAG, "Time is negative - rt_clock.hpp");
                while (true) {
                  vTaskDelay(1000/portTICK_PERIOD_MS);
                }
              }

              //Wait forever
              if (t == std::numeric_limits<double>::infinity()) {
                while (!expired && !interrupted) {}
              }

              gptimer_stop(executionTimer);
              uint64_t count = 0;
              gptimer_get_raw_count(executionTimer, &count);
              actual_delay = get_time_in_micro_seconds(t) - count + scheduler_slip;
              // Slip keeps track of how far behind schedule we are.
              scheduler_slip = actual_delay;
              // If we are ahead of schedule, then reset it to zero
              if (scheduler_slip >= 0) {
                scheduler_slip = 0;
              }

              if (MISSED_DEADLINE_TOLERANCE != -1 ) {
                if (actual_delay >= -MISSED_DEADLINE_TOLERANCE) {
                  if(actual_delay < 0){
                    ESP_LOGW(TAG, "Slip by: %d us", actual_delay);
                  }
                  actual_delay = set_timeout(actual_delay);
                } else {
                  //Missed Real Time Deadline and could not recover (Slip is passed the threshold)
                  ESP_LOGE(TAG, "MISSED SCHEDULED TIME ADVANCE DEADLINE BY: %d us\n", -actual_delay);
                  while (true) {
                    vTaskDelay(1000/portTICK_PERIOD_MS);
                  }
                }
              }

              gptimer_set_raw_count(executionTimer, 0);
              gptimer_start(executionTimer);

              return micro_seconds_to_time(actual_delay);
            }
            
            void update(){
              interrupted = true;
            }

            static bool timeout_expired(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
              BaseType_t high_task_awoken = pdFALSE;
              QueueHandle_t queue = (QueueHandle_t)user_data;
              gptimer_stop(timer);
              bool flag = true;
              xQueueSendFromISR(queue, &flag, &high_task_awoken);
              return (high_task_awoken == pdTRUE);
            }

            void startSimulation(){
              gptimer_set_raw_count(executionTimer, 0);
              gptimer_start(executionTimer);
            }
          };

          #else
            #include "rt_clock_linux.hpp"
          #endif
          
  }
  #endif //CADMIUM_RT_CLOCK_HPP
}