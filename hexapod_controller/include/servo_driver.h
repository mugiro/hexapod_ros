// ROS Hexapod Locomotion Node
// Copyright (c) 2014, Kevin M. Ochs
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of the <organization> nor the
//     names of its contributors may be used to endorse or promote products
//     derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Author: Kevin M. Ochs

// modificado 5


#ifndef SERVO_DRIVER_H_
#define SERVO_DRIVER_H_

#include <cmath>
#include <ros/ros.h>
// #include <dynamixel_sdk/dynamixel_sdk.h>
#include <sensor_msgs/JointState.h>

#include <stdint.h>

#define CHANNEL(x)	(static_cast<uint8_t>(x))
#define ANGLE(x)	(static_cast<uint8_t>(x))
// Default setting
#define BAUDRATE    1000000
// #define DEVICENAME  "/dev/ttyUSB0"      // Check which port is being used on your controller
// #define PROTOCOL_VERSION   1.0          // See which protocol version is used in the Dynamixel
#define TORQUE_ON   1
#define TORQUE_OFF  0
#define LEN_GOAL_POSITION  2

#define PCA9685_I2C_ADDRESS_DEFAULT	0x40
#define PCA9685_I2C_ADDRESS_FIXED	0x70
#define PCA9685_I2C_ADDRESSES_MAX	62

// #define CHANNEL(x)	(static_cast<uint8_t>(x))
#define VALUE(x)	(static_cast<uint16_t>(x))

#define PCA9685_VALUE_MIN	VALUE(0)
#define PCA9685_VALUE_MAX	VALUE(4096)

#define PCA9685_PWM_CHANNELS	16

// #define ANGLE(x)	(static_cast<uint8_t>(x))

#define SERVO_LEFT_DEFAULT_US	1000
#define SERVO_CENTER_DEFAULT_US	1500
#define SERVO_RIGHT_DEFAULT_US 	2000

struct TPCA9685FrequencyRange {
	static constexpr uint32_t MIN = 24;
	static constexpr uint32_t MAX = 1526;
};

enum TPCA9685Och {
	PCA9685_OCH_STOP = 0,
	PCA9685_OCH_ACK = 1 << 3
};

//==============================================================================
// Define the class(s) for Servo Drivers.
//==============================================================================

class ServoDriver
{
    public:
        ServoDriver( void );
        ~ServoDriver( void );
        void SetAngle(uint8_t nChannel, uint8_t nAngle);
        void transmitServoPositions( const sensor_msgs::JointState &joint_state );
        void makeSureServosAreOn( const sensor_msgs::JointState &joint_state );
        void freeServos( void );

        // pca9685
        // ServoDriver(uint8_t nAddress = PCA9685_I2C_ADDRESS_DEFAULT);
        // ~ServoDriver(void);

        void SetAddress(uint8_t nAddress = PCA9685_I2C_ADDRESS_DEFAULT);

        void SetPreScaller(uint8_t);
        uint8_t GetPreScaller(void);

        void SetFrequency(uint16_t);
        uint16_t GetFrequency(void);

        void SetOCH(TPCA9685Och);
        TPCA9685Och GetOCH(void);

        void SetInvert(bool);
        bool GetInvert(void);

        void SetOutDriver(bool);
        bool GetOutDriver(void);

        void Write(uint8_t, uint16_t, uint16_t);
        void Read(uint8_t, uint16_t *, uint16_t *);

        void Write(uint16_t, uint16_t);
        void Read(uint16_t *, uint16_t *);

        void Write(uint8_t, uint16_t);
        void Write(uint16_t);

        void SetFullOn(uint8_t, bool);
        void SetFullOff(uint8_t, bool);

        void Dump(void);

        //PCA9685Servo
        // ServoDriver(uint8_t nAddress = 0x40);
        // ~ServoDriver(void);

        void SetLeftUs(uint16_t);
        uint16_t GetLeftUs(void) const;

        void SetRightUs(uint16_t);
        uint16_t GetRightUs(void) const;

        void Set(uint8_t nChannel, uint16_t nData);
        void Set(uint8_t nChannel, uint8_t nData);

        // void SetAngle(uint8_t nChannel, uint8_t nAngle);

    private:
        // dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME); // Initialize PacketHandler instance
        // dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION); // Set the protocol version
        // uint8_t dxl_error = 0;                          // Dynamixel error
        // uint16_t dxl_present_position = 0;              // Present position
        uint16_t currentPos;
        uint8_t param_goal_position[2];
        void convertAngles( const sensor_msgs::JointState &joint_state );
        std::vector<int> cur_pos_; // Current position of servos
        std::vector<int> goal_pos_; // Goal position of servos
        std::vector<int> pose_steps_; // Increment to use going from current position to goal position
        std::vector<int> write_pos_; // Position of each servo for sync_write packet
        std::vector<double> OFFSET; // Physical hardware offset of servo horn
        std::vector<int> ID; // Servo IDs
        std::vector<int> TICKS; // Total number of ticks, meaning resolution of dynamixel servo
        std::vector<int> CENTER; // Center value of dynamixel servo
        std::vector<double> RAD_TO_SERVO_RESOLUTION; // Radians to servo conversion
        std::vector<double> MAX_RADIANS; // Max rotation your servo is manufactured to do. i.e. 360 degrees for MX etc.
        XmlRpc::XmlRpcValue SERVOS; // Servo map from yaml config file
        std::vector<int> servo_orientation_; // If the servo is physically mounted backwards this sign is flipped
        std::vector<std::string> servo_map_key_;
        bool portOpenSuccess = false;
        bool torque_on = true;
        bool torque_off = true;
        bool writeParamSuccess = true;
        bool servos_free_;
        int SERVO_COUNT;
        int TORQUE_ENABLE, PRESENT_POSITION_L, GOAL_POSITION_L, INTERPOLATION_LOOP_RATE;

    //PCA9685
    private:
        uint8_t CalcPresScale(uint16_t);
        uint16_t CalcFrequency(uint8_t);

    private:
        void Sleep(bool);
        void AutoIncrement(bool);

    private:
        void I2cSetup(void);

        void I2cWriteReg(uint8_t, uint8_t);
        uint8_t I2cReadReg(uint8_t);

        void I2cWriteReg(uint8_t, uint16_t);
        uint16_t I2cReadReg16(uint8_t);

        void I2cWriteReg(uint8_t, uint16_t, uint16_t);

    private:
        uint8_t m_nAddress;

    // PCA9685Servo
    private:
        void CalcLeftCount(void);
        void CalcRightCount(void);

    private:
        uint16_t m_nLeftUs;
        uint16_t m_nRightUs;
        uint16_t m_nLeftCount;
        uint16_t m_nRightCount;
};


#endif // SERVO_DRIVER_H_