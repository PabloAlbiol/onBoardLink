/*==============================================================================
    Pablo Albiol http://github.com/PabloAlbiol
==============================================================================*/


#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <cstdio>
#include <cstring>
#include <pthread.h>
#include <iostream>
#include <time.h>
#include <sys/time.h>
#include <vector>

#include "linkAbstract.h"
#include "linkSerial.h"
#include "linkUDP.h"
#include "MAVLink/pabloag/mavlink.h"
#include "utilities.h"

#define MAXSIZESEND 60


// Bridge function to access runTelemetry (telemetry class)
void *auxTelemetry(void *vartel);

class telemetry
{
    friend void *auxTelemetry(void *vartel);

    // Public Functions
    public:
        telemetry();
        ~telemetry();

        int setupTelemetry();
        void startTelemetry();

        void write_attitude(float _pitch, float _roll, float _yaw, float _pitchspeed, float _rollspeed, float _yawspeed);
        void write_global_position_int(int32_t _lat, int32_t _lon, int32_t alt, int32_t _relative_alt, int16_t _vx, int16_t _vy, int16_t _vz, uint16_t _hdg);
        void write_heartbeat(uint8_t _base_mode, uint32_t _custom_mode, uint8_t _system_status);
        void write_sys_status(uint32_t _onboard_control_sensors_present, uint32_t _onboard_control_sensors_enabled, uint32_t _onboard_control_sensors_health, uint16_t _load, uint16_t _voltage_battery, int16_t _current_battery, int8_t _battery_remaining);
        void write_mission_current(uint16_t _seq);
        void write_mission_item_reached(uint16_t seq);

        bool b_set_mode();
        bool b_mission_set_current();
        bool b_mission_item();
        bool b_onboard_pid_parameters_attitude();
        bool b_onboard_pid_parameters_position();
        bool b_onboard_references_quadrotor();

        void get_set_mode(uint8_t *get_base_mode, uint32_t *get_custom_mode);
        void get_mission_set_current(uint8_t *get_seq);
        void get_mission_item(std::vector<mavlink_mission_item_t> *get_mission_item);
        void get_onboard_pid_parameters_attitude(mavlink_onboard_pid_parameters_attitude_t *get_onboard_pid_parameters_attitude);
        void get_onboard_pid_parameters_position(mavlink_onboard_pid_parameters_position_t *get_onboard_pid_parameters_position);
        void get_onboard_references_quadrotor(mavlink_onboard_references_quadrotor_t *get_onboard_references_quadrotor);

    // Private variables
    private:
        linkAbstract *link;
        bool runLoop; // Variable to stop the thread

        std::vector<uint8_t> dataWrite;
        uint8_t *bufWriteSend;
        uint8_t bufRead[MAVLINK_MAX_PACKET_LEN];
        uint8_t bufWrite[MAVLINK_MAX_PACKET_LEN];
        uint8_t bufWritePrev[MAVLINK_MAX_PACKET_LEN];

        uint16_t len;
        uint16_t lenPrev;
        int32_t sizReceive;
        int32_t sizSend;
        int32_t sizSended;

        // Variables that store information about the threads
        pthread_t telemetryThread;
        pthread_attr_t attr; // Attributes
        void *returnTelemetry;

        // Semaphores
        pthread_mutex_t mutexReceive;
        pthread_mutex_t mutexReceiveReferences; // One semaphore for the reference message (just in case it's received regularly) to increase parallelism
        pthread_mutex_t mutexWrite; // The write functions are not meant to be called simultaneously, so one semaphore is enough

        // Timing variables
        struct timeval tStart, tEnd, tTimeout;
        long int elapTime, elapTimeout;

        // Timeout
        bool waitTimeout;
        uint8_t retryNum;

        // MAVLink variables
        mavlink_message_t msgSend;
        mavlink_message_t msgReceive;
        mavlink_status_t status;
        mavlink_system_t mavlink_system;
        uint8_t channel;
        uint8_t system_id;
        uint8_t component_id;

        // MAVLink message structures
        mavlink_heartbeat_t heartbeat;
        mavlink_sys_status_t sys_status;
        mavlink_set_mode_t set_mode;
        mavlink_param_value_t param_value;
        mavlink_attitude_t attitude;
        mavlink_global_position_int_t global_position_int;
        mavlink_command_long_t command_long;
        mavlink_command_ack_t command_ack;
        mavlink_set_pid_parameters_attitude_t set_pid_parameters_attitude;
        mavlink_onboard_pid_parameters_attitude_t onboard_pid_parameters_attitude;
        mavlink_read_pid_parameters_attitude_t read_pid_parameters_attitude;
        mavlink_set_pid_parameters_position_t set_pid_parameters_position;
        mavlink_onboard_pid_parameters_position_t onboard_pid_parameters_position;
        mavlink_read_pid_parameters_position_t read_pid_parameters_position;
        mavlink_set_references_quadrotor_t set_references_quadrotor;
        mavlink_onboard_references_quadrotor_t onboard_references_quadrotor;
        uint8_t bitmaskAux;

        // WAYPOINT PROTOCOL
        std::vector<mavlink_mission_item_t> mission_item;
        std::vector<mavlink_mission_item_t> mission_item_copy;
        mavlink_mission_item_t mission_item_aux;
        mavlink_mission_request_t mission_request;
        mavlink_mission_set_current_t mission_set_current;
        mavlink_mission_current_t mission_current;
        mavlink_mission_request_list_t mission_request_list;
        mavlink_mission_count_t mission_count;
        mavlink_mission_clear_all_t mission_clear_all;
        mavlink_mission_item_reached_t mission_item_reached;
        mavlink_mission_ack_t mission_ack;

        // Bool variables to check if a certain message has been received
        bool _set_mode;
        bool _mission_set_current;
        bool _mission_ack; // Means that a waypoint list has been received
        bool _onboard_pid_parameters_attitude;
        bool _onboard_pid_parameters_position;
        bool _onboard_references_quadrotor;

    // Private functions
    private:
        void runTelemetry();
        void send();
        void receive();
        void initialMsgValues();
        void write(const uint8_t *data, uint32_t len);

        void write_onboard_references_quadrotor();
        void write_onboard_pid_parameters_position();
        void write_onboard_pid_parameters_attitude();
        void write_param_value();
        void write_mission_count();
        void write_mission_item();
        void write_mission_request();
        void write_mission_ack();
};

#endif // TELEMETRY_H
