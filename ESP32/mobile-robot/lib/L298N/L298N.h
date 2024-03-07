/**
 * @file L298N.h
 * @brief Header file for L298N motor driver class.
 * @date 2023-11-05
 */

#ifndef L298N_H
#define L298N_H

#include "Arduino.h"
#include <stdint.h>

/**
 * @class L298N
 * @brief Class representing the L298N motor driver.
 */
class L298N{

    private:
        uint8_t enable_A;
        uint8_t enable_B;
        uint8_t in1_A;
        uint8_t in2_A;
        uint8_t in1_B;
        uint8_t in2_B;
        uint16_t pwm_A;
        uint16_t pwm_B;
        uint8_t resolution;
        int motor_A_status;
        int motor_B_status;

    /**
     * @brief Enumeration for motor status (STOPPED, FORWARD, BACKWARD).
     */
    public:
        typedef enum{
            STOPPED,
            FORWARD,
            BACKWARD
        }STATUS;

        /**
         * @brief Constructor for the L298N motor driver.
         * @param enable_A Pin number for motor A enable.
         * @param enable_B Pin number for motor B enable.
         * @param in1_A Pin number for motor A input 1.
         * @param in2_A Pin number for motor A input 2.
         * @param in1_B Pin number for motor B input 1.
         * @param in2_B Pin number for motor B input 2.
         * @param resolution PWM resolution (default is 8).
         */
        L298N(uint8_t enable_A,uint8_t enable_B,uint8_t in1_A,uint8_t in2_A,uint8_t in1_B,uint8_t in2_B, uint8_t resolution);
        /**
         * @brief Set up pin modes and PWM configurations.
         */
        void setUpPinMode();

        void setSpeed_A(uint16_t pwm);

        /**
         * @brief Set the speed of motor B.
         * @param pwm PWM value for motor B.
         */
        void setSpeed_B(uint16_t pwm);

        /**
         * @brief Get the speed of motor A.
         * @return Speed of motor A.
         */
        uint16_t getSpeedA();

        /**
         * @brief Get the speed of motor B.
         * @return Speed of motor B.
         */
        uint16_t getSpeedB();

        /**
         * @brief Move motor A forward.
         */
        void moveForward_A();

        /**
         * @brief Move motor B forward.
         */
        void moveForward_B();

        /**
         * @brief Move motor A backward.
         */
        void moveBackward_A();

        /**
         * @brief Move motor B backward.
         */
        void moveBackward_B();

        /**
         * @brief Update the speed of motor A.
         * @param pwm New PWM value for motor A.
         */
        void updateSpeed_A(uint16_t pwm);

        /**
         * @brief Update the speed of motor B.
         * @param pwm New PWM value for motor B.
         */
        void updateSpeed_B(uint16_t pwm);

        /**
         * @brief Stop motor A.
         */
        void stop_A();

        /**
         * @brief Stop motor B.
         */
        void stop_B();

        /**
         * @brief Get the status of motor A.
         * @return Status of motor A (STOPPED, FORWARD, BACKWARD).
         */
        uint8_t getMotorAStatus();

        /**
         * @brief Get the status of motor B.
         * @return Status of motor B (STOPPED, FORWARD, BACKWARD).
         */
        uint8_t getMotorBStatus();
};

#endif