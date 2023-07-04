/*
Copyright (c) 07/2023, Vincent FOUCAULT
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef PCANTOMOTEUS_HPP
#define PCANTOMOTEUS_HPP

#include <iostream>
#include <ostream> 
#include <iterator>
#include <ctime>
#include <vector>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <ros/ros.h>

#include <PCANBasic.h> // Peak m2canFd board lib

//define pcan 4 ports to their respective physical adress
#define PCAN_DEV1	PCAN_PCIBUS1 // one complete leg (HAA, HFE, KFE)
#define PCAN_DEV2	PCAN_PCIBUS2
#define PCAN_DEV3	PCAN_PCIBUS3
#define PCAN_DEV4	PCAN_PCIBUS4

/* moteus controllers TX bytes adress */
#define MSGTX_ADDR_POSITION     0x06 
#define MSGTX_ADDR_VELOCITY     0x0A
#define MSGTX_ADDR_FFTORQUE     0x0E
#define MSGTX_ADDR_KP           0x12
#define MSGTX_ADDR_KD           0x16
#define MSGTX_ADDR_MAXTORQUE    0x1A

/* moteus controllers RX bytes adress */
#define MSGRX_ADDR_POSITION     0x02
#define MSGRX_ADDR_VELOCITY     0x06
#define MSGRX_ADDR_TORQUE       0x0A
#define MSGRX_ADDR_VOLTAGE      0x16
#define MSGRX_ADDR_TEMPERATURE  0x1A
#define MSGRX_ADDR_FAULT        0x1E

// about moteus controllers faults errors:
/*  32 - calibration fault - the encoder was not able to sense a magnet during calibration
    33 - motor driver fault - the most common reason for this is undervoltage, moteus attempted to draw more current than the supply could provide. Other electrical faults may also report this error, the drv8323 diagnostic tree has more information.
    34 - over voltage - the bus voltage exceeded servo.max_voltage. This can happen due to misconfiguration, or if the controller regenerated power with a supply that cannot sink power and no flux braking was configured.
    35 - encoder fault - the encoder readings are not consistent with a magnet being present.
    36 - motor not configured - the moteus_tool --calibrate procedure has not been run on this motor.
    37 - pwm cycle overrun - an internal firmware error
    38 - over temperature - the maximum configured temperature has been exceeded
    39 - outside limit - an attempt was made to start position control while outside the bounds configured by servopos.position_min and servopos.position_max.*/

/* power board RX bytes adress */ // TODO check adresses
#define MSGPBRX_ADDR_STATE          0x02 // 2 bytes per value (int16)   
#define MSGPBRX_ADDR_FAULT_CODE     0x04  
#define MSGPBRX_ADDR_SWITCH_STATUS  0x06
#define MSGPBRX_ADDR_OUT_VOLTAGE    0x08
#define MSGPBRX_ADDR_OUT_CURRENT    0x0A
#define MSGPBRX_ADDR_TEMPERATURE    0x0C
#define MSGPBRX_ADDR_ENERGY         0x0E

// about moteus power board faults errors:
/* TODO */


// a structure for robot controllers setup (id orders, affiliate Peak port, joint direction )
struct MotorAdapter{

  public:

    MotorAdapter()
    {
      idx_ =        -1;
      sign_ =        1; // -1 will invert Qdd100 rotation
      port_ =        0;
    }

    MotorAdapter(int idx, int sign, int port)
    {
      idx_         = idx;
      sign_        = sign;
      port_        = port;
    }

    const int& getIdx()         {return idx_;}
    const int& getSign()        {return sign_;}
    const int& getPort()        {return port_;}
    void setIdx(int idx)                 {idx_ = idx;}
    void setSign(int sign)               {sign_ = sign;}
    void setPort(int port)               {port_ = port;}

  private:

    int idx_;
    int sign_;
    int port_;
};


// the YloTwoPcanToMoteus class
class YloTwoPcanToMoteus{

  public:

    /* initialize PEAK M2 all ports*/
    bool Can_init();

    /* reset PEAK M2 all ports (4)*/
    bool Can_reset();

    /* zero a single moteus controller*/
    bool send_moteus_zero_order(int id, int port, float position);

    /* send a single canFD STOP frame command, 
       and query informations about moteus controller*/
    bool send_moteus_stop_order(int id, int port);

    /* send a single canFD TX frame
       and query informations about moteus controller :
       values will wait into the canFD queue*/
    bool send_moteus_TX_frame(int id, int port, float pos, float vel, float fftorque, float kp, float kd);

    /* query a single canFD RX Queue, 
       and read ID params*/
    bool read_moteus_RX_queue(int id, int port, float& position, float& velocity, float& torque, float& voltage, float& temperature, float& fault);

    /* send a single canFD command frame
       ask informations about moteus power board*/
    bool send_power_board_order();

    /* query power board to read values*/
    bool read_power_board_RX_queue(float& state, float& fault_code, float& switch_status, float& out_volt, float& out_curr, float& board_temp);

    /*  initialize Peak canFD board, and reset the 4 ports.*/
    bool peak_fdcan_board_initialization();

    /*  send a stop order to all motors  */
    bool stop_motors();

    /* Checking initial pose of robot, when sitted, to be certain that it is sitted as needed
      1/ zero joints with initial_ground_joints_pose vector
      2/ check angle error between asked rezero, and read position
      3/ loop until success */
    bool check_initial_ground_pose();

    uint32_t _id; // ID of a moteus controller

    YloTwoPcanToMoteus();
    virtual ~YloTwoPcanToMoteus();

    std::vector<MotorAdapter> motor_adapters_;

    bool all_moteus_controllers_ok = true; // not used for now
    //bool can_error = false; // error in Peak FDCAN send order or query
    //bool is_calibrated = true; // is the robot in right startup pose

    //for moteus RX
    float _rx_pos   = 0.0;
    float _rx_vel   = 0.0;
    float _rx_tor   = 0.0;

    float _rx_volt  = 0.0;
    float _rx_temp  = 0.0;
    float _rx_fault = 0;

    //for moteus TX
    float _comm_position      = 0.0; // NAN, if torque mode
    float _comm_velocity      = 0.0;
    float _comm_fftorque      = 0.0;
    float _comm_kp            = 0.0;
    float _comm_kd            = 0.0;
    float _comm_maxtorque     = 0.5; // Max possible torque is NAN value

  private:

    // for pcanbasic library
    TPCANStatus Status; // the return of a command, to check success

    // Define the compatible Moteus FD Bitrate string
    TPCANBitrateFD BitrateFD = (char*) "f_clock_mhz = 80, nom_brp = 1, nom_tseg1 = 50, nom_tseg2 = 29, nom_sjw = 10, data_brp = 1, data_tseg1 = 8, data_tseg2 = 7, data_sjw = 12";
    TPCANTimestampFD timestamp;

    TPCANMsgFD _stop; // the stop canFD message 
    TPCANMsgFD _zero; // the stop canFD message 
    TPCANMsgFD moteus_tx_msg; // the canFD message to send order to a moteus controller
    TPCANMsgFD moteus_rx_msg; // the canFD message to read moteus controller
    TPCANMsgFD _power_board_tx_msg; // the canFD message to ask moteus power board
    TPCANMsgFD _power_board_rx_msg; // the canFD message to read moteus power board

    char strMsg[256];
    std::vector<int> pcanPorts_ = {PCAN_DEV1, PCAN_DEV2, PCAN_DEV3, PCAN_DEV4};

    int idx_;
    int sign_;
    int port_;
    int stop_pos_low_;
    int stop_pos_high_;



    /* query variables for moteus controllers */
    float _position     = 0.0;
    float _velocity     = 0.0;
    float _torque       = 0.0;
    float _voltage      = 0.0;
    float _temperature  = 0.0;
    float _fault        = 0.0;

    /* query variables for moteus power board */
    float _state          = 0.0;
    float _fault_code     = 0.0;
    float _switch_status  = 0.0;
    float _lock_time      = 0.0;
    float _boot_time      = 0.0;
    float _out_volt       = 0.0;
    float _out_curr       = 0.0;
    float _board_temp     = 0.0;
    float _energy         = 0.0;

    //  zero position of controllers, to check . robot position is sitted 
    std::vector<float>initial_ground_joints_pose = {0.05833955109119415, 0.1817300021648407, -0.4297744333744049,  // 6, 4, 5 (Qdd100 ID's)
                                                    -0.05806489288806915, -0.1830422580242157, 0.3962658643722534, // 3, 1, 2
                                                    -0.06296806037425995, 0.18036942183971405, -0.4263542890548706, // 12, 10, 11
                                                    0.055633656680583954, -0.18712398409843445, 0.4280807077884674};   // 9, 7, 8


    float calibration_error = 0.01; // 3.6 degrees
};

#endif // PCANTOMOTEUS_HPP
