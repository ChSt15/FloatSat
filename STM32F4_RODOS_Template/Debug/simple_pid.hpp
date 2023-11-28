#ifndef SIMPLE_PID_HPP_
#define SIMPLE_PID_HPP_

#include "stdint.h"

#include "rodos.h"

namespace VCTR
{
    namespace CTRL
    {

        /**
         * @brief Simple one dimensional PID controller.
         */
        class SimplePID
        {
        private:
            float kp_;
            float ki_;
            float kd_;

            float integral_;
            float lastError_;

            float maxOutput_;

            float setPoint_;

            int64_t lastTime_;

        public:
            /**
             * @brief Construct a new SimplePID object.
             * @param kp The proportional gain.
             * @param ki The integral gain.
             * @param kd The derivative gain.
             * @param maxOutput The maximum output value the actuator is capable of.
             */
            SimplePID(float kp, float ki, float kd, float maxOutput);

            /**
             * @brief Set the setpoint of the PID controller.
             * @param setPoint The setpoint vector. First value is the position, second is the derivative.
             */
            void setSetPoint(float setPoint);

            /**
             * @brief Get the setpoint of the PID controller.
             * @returns The setpoint vector. First value is the position, second is the derivative.
             */
            float getSetPoint() const;

            /**
             * @brief Update the PID controller.
             * @param state The state vector. First value is the position, second is the derivative.
             * @param stateTime The time of the state vector. Defaults to current time via NOW()
             * @returns The output of the PID controller.
             */
            float update(float state, int64_t stateTime = NOW());
        };

    }
}

#endif