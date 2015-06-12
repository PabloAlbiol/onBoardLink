/*==============================================================================
    Pablo Albiol http://github.com/PabloAlbiol
==============================================================================*/


/** MAVLINK
http://qgroundcontrol.org/mavlink/start
*/

#include "telemetry.h"

#define UINT16_MAX 65535 // Some MAVLink messages need this value
#define TIMEOUT 500 // If no response is received within this time, the message is sent again
#define NUM_RETRIES 3 // Number of retries of the resending message

#define PI_NUMBER 3.14159265358979323846
#define PERIODTELEMETRY 50 // Period in ms
#define LINKTYPE 1 // Serial = 0, UDP = 1


telemetry::telemetry()
{
    switch (LINKTYPE)
    {
        case 0:
            link = new linkSerial("/dev/ttyO0", 57600); // "/dev/ttyO0" for UART1 (IGEP). Pins JA41:9 (TX), JA41:10 (RX)
        break;

        case 1:
            link = new linkUDP("192.168.3.101", 15555, "192.168.3.100", 15556); // void setConnect(const char getipSend[], uint32_t getudpPortSend, const char getipListen[], uint32_t getudpPortListen)
        break;
    }
}

telemetry::~telemetry()
{
    runLoop = false;

    link->closeLink();
    delete link;

    // Wait for the thread to be closed
    pthread_join(telemetryThread, &returnTelemetry);

    // Delete the attributes
    pthread_attr_destroy(&attr);

    // Uninitialize mutex
    pthread_mutex_destroy(&mutexReceive);
    pthread_mutex_destroy(&mutexReceiveReferences);
    pthread_mutex_destroy(&mutexWrite);
}

int telemetry::setupTelemetry(void)
{
    // Open the link
    if(link->setupLink() == -1)
    {
        std::cout << "Error opening link" << std::endl;
        return -1;
    }
    else
    {
        std::cout << "The link is opened" << std::endl;
        return 1;
    }
}

// This function creates the telemetry thread
void telemetry::startTelemetry(void)
{
    initialMsgValues();

    dataWrite.clear();
    mission_item.clear();
    mission_item_copy.clear();

    retryNum = 0;
    waitTimeout = false;

    runLoop = true;

    // Initialize mutex
    pthread_mutex_init(&mutexReceive, NULL);
    pthread_mutex_init(&mutexReceiveReferences, NULL);
    pthread_mutex_init(&mutexWrite, NULL);

    // Attributes. Joinable thread
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    // Creates the thread
    pthread_create(&telemetryThread, &attr, auxTelemetry, this);
}

// Bridge function to access runTelemetry (telemetry class)
void *auxTelemetry(void *varTel)
{
    ((telemetry *)varTel)->runTelemetry();
	return NULL;
}


// Telemetry thread
void telemetry::runTelemetry()
{
    while (runLoop)
    {
        // Get the time at the beginning of the loop
        gettimeofday(&tStart, NULL);

        send();
        receive();

        // Get the time at the end of the loop
        gettimeofday(&tEnd, NULL);

        // Check timeout (for waypoint protocol mainly)
        if (waitTimeout == true)
        {
            elapTimeout = timeDiff(&tEnd, &tTimeout);
            if (elapTimeout > TIMEOUT * 1000)
            {
                if (retryNum < NUM_RETRIES)
                {
                    pthread_mutex_lock(&mutexWrite);
                    write(bufWritePrev, lenPrev);
                    pthread_mutex_unlock(&mutexWrite);
                    gettimeofday(&tTimeout, NULL);
                    retryNum++;
                    std::cout << "Timeout. Message resent" << std::endl;
                }
                else
                {
                    retryNum = 0;
                    waitTimeout = false;
                    std::cout << "Timeout. Number of retries exceeded. Transaction failed" << std::endl;
                }
            }
        }

        // Calculate the elapsed time in the loop
        elapTime = timeDiff(&tEnd, &tStart);

        // Sleep to accomplish the desired frequency
        if (elapTime < PERIODTELEMETRY*1000)
        {
            usleep(PERIODTELEMETRY*1000 - elapTime);
        }
        else
        {
            std::cout << "Error Telemetry period" << std::endl;
        }
    }
}

// Append messages to a internal buffer
void telemetry::write(const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        dataWrite.push_back(data[i]);
    }
}


// Send function
void telemetry::send(void)
{
    pthread_mutex_lock(&mutexWrite);

    sizSend = dataWrite.size();
    if (sizSend > MAXSIZESEND)
    {
        sizSend = MAXSIZESEND;
    }

    if (sizSend > 0)
    {
        // bufWriteSend points to dataWrite
        bufWriteSend = &dataWrite[0];

        // Send
        sizSended = link->writeLink((char *)bufWriteSend, sizSend);

        if (sizSended > 0)
        {
            // Delete sended bytes from the buffer
            dataWrite.erase(dataWrite.begin(), dataWrite.begin() + sizSended);
        }
    }

    pthread_mutex_unlock(&mutexWrite);
}


// Receive function
void telemetry::receive(void)
{
    if ((sizReceive = link->readLink((char *)bufRead)) > 0)
    {
        for (int32_t i = 0; i < sizReceive; i++)
        {
            if (mavlink_parse_char(channel, bufRead[i], &msgReceive, &status))
            {
                switch(msgReceive.msgid)
                {

                    case 11:
                        pthread_mutex_lock(&mutexReceive);
                        mavlink_msg_set_mode_decode(&msgReceive, &set_mode);
                        _set_mode = true;
                        pthread_mutex_unlock(&mutexReceive);
                        break;

                    case 21:
                        write_param_value();
                        break;

                    case 39:
                        mavlink_msg_mission_item_decode(&msgReceive, &mission_item_aux);
                        mission_item.push_back(mission_item_aux);
                        retryNum = 0;
                        waitTimeout = false;
                        if (mission_request.seq < mission_count.count - 1)
                        {
                            mission_request.seq=mission_item[mission_request.seq].seq+1;
                            write_mission_request();
                        }
                        else
                        {
                            write_mission_ack();
                        }
                        break;

                    case 40:
                        mavlink_msg_mission_request_decode(&msgReceive, &mission_request);
                        retryNum = 0;
                        waitTimeout = false;
                        write_mission_item();
                        break;

                    case 41:
                        pthread_mutex_lock(&mutexReceive);
                        mavlink_msg_mission_set_current_decode(&msgReceive, &mission_set_current);
                        _mission_set_current = true;
                        pthread_mutex_unlock(&mutexReceive);
                        break;

                    case 43:
                        if (waitTimeout == false) // The previous transaction has to end before a new one
                        {
                            mavlink_msg_mission_request_list_decode(&msgReceive, &mission_request_list);
                            write_mission_count();
                        }
                        break;

                    case 44:
                        if (waitTimeout == false) // The previous transaction has to end before a new one
                        {
                            mavlink_msg_mission_count_decode(&msgReceive, &mission_count);
                            if (mission_count.count > 0)
                            {
                                mission_item.clear(); // Delete previous mission_item
                                mission_request.seq = 0;
                                write_mission_request();
                            }
                        }
                        break;

                    case 45:
                        mavlink_msg_mission_clear_all_decode(&msgReceive, &mission_clear_all);
                        mission_item.clear();
                        write_mission_ack();
                        break;

                    case 47:
                        pthread_mutex_lock(&mutexReceive);
                        mavlink_msg_mission_ack_decode(&msgReceive, &mission_ack);
                        mission_item_copy = mission_item;
                        retryNum = 0;
                        waitTimeout = false;
                        _mission_ack = true;
                        pthread_mutex_unlock(&mutexReceive);
                        break;

                    case 76:
                        mavlink_msg_command_long_decode(&msgReceive, &command_long);
                        break;

                    case 180:
                        pthread_mutex_lock(&mutexReceive);
                        mavlink_msg_set_pid_parameters_attitude_decode(&msgReceive, &set_pid_parameters_attitude);
                        onboard_pid_parameters_attitude.pitch_p = set_pid_parameters_attitude.pitch_p;
                        onboard_pid_parameters_attitude.roll_p = set_pid_parameters_attitude.roll_p;
                        onboard_pid_parameters_attitude.yaw_p = set_pid_parameters_attitude.yaw_p;
                        onboard_pid_parameters_attitude.height_p = set_pid_parameters_attitude.height_p;
                        onboard_pid_parameters_attitude.pitch_d = set_pid_parameters_attitude.pitch_d;
                        onboard_pid_parameters_attitude.roll_d = set_pid_parameters_attitude.roll_d;
                        onboard_pid_parameters_attitude.yaw_d = set_pid_parameters_attitude.yaw_d;
                        onboard_pid_parameters_attitude.height_d = set_pid_parameters_attitude.height_d;
                        onboard_pid_parameters_attitude.pitch_i = set_pid_parameters_attitude.pitch_i;
                        onboard_pid_parameters_attitude.roll_i = set_pid_parameters_attitude.roll_i;
                        onboard_pid_parameters_attitude.yaw_i = set_pid_parameters_attitude.yaw_i;
                        onboard_pid_parameters_attitude.height_i = set_pid_parameters_attitude.height_i;
                        onboard_pid_parameters_attitude.pitch_p_sat = set_pid_parameters_attitude.pitch_p_sat;
                        onboard_pid_parameters_attitude.roll_p_sat = set_pid_parameters_attitude.roll_p_sat;
                        onboard_pid_parameters_attitude.yaw_p_sat = set_pid_parameters_attitude.yaw_p_sat;
                        onboard_pid_parameters_attitude.height_p_sat = set_pid_parameters_attitude.height_p_sat;
                        onboard_pid_parameters_attitude.pitch_d_sat = set_pid_parameters_attitude.pitch_d_sat;
                        onboard_pid_parameters_attitude.roll_d_sat = set_pid_parameters_attitude.roll_d_sat;
                        onboard_pid_parameters_attitude.yaw_d_sat = set_pid_parameters_attitude.yaw_d_sat;
                        onboard_pid_parameters_attitude.height_d_sat = set_pid_parameters_attitude.height_d_sat;
                        onboard_pid_parameters_attitude.pitch_i_sat = set_pid_parameters_attitude.pitch_i_sat;
                        onboard_pid_parameters_attitude.roll_i_sat = set_pid_parameters_attitude.roll_i_sat;
                        onboard_pid_parameters_attitude.yaw_i_sat = set_pid_parameters_attitude.yaw_i_sat;
                        onboard_pid_parameters_attitude.height_i_sat = set_pid_parameters_attitude.height_i_sat;

                        _onboard_pid_parameters_attitude = true;
                        pthread_mutex_unlock(&mutexReceive);
                        write_onboard_pid_parameters_attitude();
                        break;

                    case 182:
                        write_onboard_pid_parameters_attitude();
                        break;

                    case 183:
                        pthread_mutex_lock(&mutexReceive);
                        mavlink_msg_set_pid_parameters_position_decode(&msgReceive, &set_pid_parameters_position);
                        onboard_pid_parameters_position.x_p = set_pid_parameters_position.x_p;
                        onboard_pid_parameters_position.y_p = set_pid_parameters_position.y_p;
                        onboard_pid_parameters_position.vel_p = set_pid_parameters_position.vel_p;
                        onboard_pid_parameters_position.x_d = set_pid_parameters_position.x_d;
                        onboard_pid_parameters_position.y_d = set_pid_parameters_position.y_d;
                        onboard_pid_parameters_position.vel_d = set_pid_parameters_position.vel_d;
                        onboard_pid_parameters_position.x_i = set_pid_parameters_position.x_i;
                        onboard_pid_parameters_position.y_i = set_pid_parameters_position.y_i;
                        onboard_pid_parameters_position.vel_i = set_pid_parameters_position.vel_i;
                        onboard_pid_parameters_position.x_p_sat = set_pid_parameters_position.x_p_sat;
                        onboard_pid_parameters_position.y_p_sat = set_pid_parameters_position.y_p_sat;
                        onboard_pid_parameters_position.vel_p_sat = set_pid_parameters_position.vel_p_sat;
                        onboard_pid_parameters_position.x_d_sat = set_pid_parameters_position.x_d_sat;
                        onboard_pid_parameters_position.y_d_sat = set_pid_parameters_position.y_d_sat;
                        onboard_pid_parameters_position.vel_d_sat = set_pid_parameters_position.vel_d_sat;
                        onboard_pid_parameters_position.x_i_sat = set_pid_parameters_position.x_i_sat;
                        onboard_pid_parameters_position.y_i_sat = set_pid_parameters_position.y_i_sat;
                        onboard_pid_parameters_position.vel_i_sat = set_pid_parameters_position.vel_i_sat;

                        _onboard_pid_parameters_position = true;
                        pthread_mutex_unlock(&mutexReceive);
                        write_onboard_pid_parameters_position();
                        break;

                    case 185:
                        write_onboard_pid_parameters_position();
                        break;

                    case 186:
                        pthread_mutex_lock(&mutexReceiveReferences);
                        mavlink_msg_set_references_quadrotor_decode(&msgReceive, &set_references_quadrotor);
                        onboard_references_quadrotor.pitch = set_references_quadrotor.pitch;
                        onboard_references_quadrotor.roll = set_references_quadrotor.roll;
                        onboard_references_quadrotor.yaw = set_references_quadrotor.yaw;
                        onboard_references_quadrotor.thrust = set_references_quadrotor.thrust;
                        onboard_references_quadrotor.x = set_references_quadrotor.x;
                        onboard_references_quadrotor.y = set_references_quadrotor.y;
                        onboard_references_quadrotor.z = set_references_quadrotor.z;
                        onboard_references_quadrotor.bitmask = set_references_quadrotor.bitmask;
                        bitmaskAux = set_references_quadrotor.bitmask;

                        _onboard_references_quadrotor = true;
                        pthread_mutex_unlock(&mutexReceiveReferences);

                        if (((bitmaskAux >> 4) & 1)==0) // Custom bitmask. I use this byte to avoid resending the message
                        {
                            write_onboard_references_quadrotor();
                        }
                        break;

                    default:
                        break;
                }
            }
        }
    }
}


// Initial and default values for serveral MAVLink messages
void telemetry::initialMsgValues()
{
    // Bool variables to check if a certain message has been received, initialized to false
    _set_mode = false;
    _mission_set_current = false;
    _mission_ack = false;
    _onboard_pid_parameters_attitude = false;
    _onboard_pid_parameters_position = false;
    _onboard_references_quadrotor = false;

    channel = MAVLINK_COMM_0;

    system_id = 0;
    component_id = MAV_COMP_ID_ALL;

    heartbeat.type = MAV_TYPE_QUADROTOR;
    heartbeat.autopilot = MAV_AUTOPILOT_GENERIC;
    heartbeat.base_mode = MAV_MODE_MANUAL_DISARMED; // Interesting flight modes: MAV_MODE_FLAG_STABILIZE_ENABLED y MAV_MODE_FLAG_GUIDED_ENABLED, mirar mensaje SET_MODE
    heartbeat.custom_mode = 0; // A bitfield for use for autopilot-specific flags
    heartbeat.system_status = MAV_STATE_STANDBY;

    attitude.pitch = 0;
    attitude.roll = 0;
    attitude.yaw = 0;
    attitude.pitchspeed = 0;
    attitude.rollspeed = 0;
    attitude.yawspeed = 0;

    sys_status.onboard_control_sensors_present=  1+2+4+8+16+32+256+4096+8192+16384+131072+262144+524288; // Bitmask showing which onboard controllers and sensors are present
    sys_status.onboard_control_sensors_enabled = 1+2+4+8+16+32+256+4096+8192+16384+131072+262144+524288;
    sys_status.onboard_control_sensors_health = 1+2+4+8+16+32+256+4096+8192+16384+131072+262144+524288;
    sys_status.load = 0;
    sys_status.voltage_battery = 0;
    sys_status.current_battery = -1; // Autopilot does not measure the current
    sys_status.battery_remaining = -1; // Autopilot estimates the remaining battery
    sys_status.drop_rate_comm = 0;
    sys_status.errors_comm = 0;
    sys_status.errors_count1 = 0;
    sys_status.errors_count2 = 0;
    sys_status.errors_count3 = 0;
    sys_status.errors_count4 = 0;

    global_position_int.lat = 0;
    global_position_int.lon = 0;
    global_position_int.alt = 0;
    global_position_int.relative_alt = 0;
    global_position_int.vx = 0;
    global_position_int.vy = 0;
    global_position_int.vz = 0;
    global_position_int.hdg = UINT16_MAX;
}

// Functions to check if a certain message has been received, initialized to false
bool telemetry::b_set_mode()
{
    pthread_mutex_lock(&mutexReceive);
    bool ret = _set_mode;
    pthread_mutex_unlock(&mutexReceive);
    return ret;
}
bool telemetry::b_mission_set_current()
{
    pthread_mutex_lock(&mutexReceive);
    bool  ret = _mission_set_current;
    pthread_mutex_unlock(&mutexReceive);
    return ret;
}
bool telemetry::b_mission_item()
{
    pthread_mutex_lock(&mutexReceive);
    bool ret = _mission_ack;
    pthread_mutex_unlock(&mutexReceive);
    return ret;
}
bool telemetry::b_onboard_pid_parameters_attitude()
{
    pthread_mutex_lock(&mutexReceive);
    bool ret = _onboard_pid_parameters_attitude;
    pthread_mutex_unlock(&mutexReceive);
    return ret;
}
bool telemetry::b_onboard_pid_parameters_position()
{
    pthread_mutex_lock(&mutexReceive);
    bool ret = _onboard_pid_parameters_position;
    pthread_mutex_unlock(&mutexReceive);
    return ret;
}
bool telemetry::b_onboard_references_quadrotor()
{
    pthread_mutex_lock(&mutexReceiveReferences);
    bool ret = _onboard_references_quadrotor;
    pthread_mutex_unlock(&mutexReceiveReferences);
    return ret;
}


// Functions to get received messages
void telemetry::get_set_mode(uint8_t *get_base_mode, uint32_t *get_custom_mode)
{
    pthread_mutex_lock(&mutexReceive);
    *get_base_mode = set_mode.base_mode;
    *get_custom_mode = set_mode.custom_mode;
    _set_mode = false;
    pthread_mutex_unlock(&mutexReceive);
}

void telemetry::get_mission_set_current(uint8_t *get_seq)
{
    pthread_mutex_lock(&mutexReceive);
    *get_seq = mission_set_current.seq;
    _mission_set_current = false;
    pthread_mutex_unlock(&mutexReceive);
}

void telemetry::get_mission_item(std::vector<mavlink_mission_item_t> *get_mission_item)
{
    pthread_mutex_lock(&mutexReceive);
    *get_mission_item = mission_item_copy;
    _mission_ack = false;
    pthread_mutex_unlock(&mutexReceive);
}

void telemetry::get_onboard_pid_parameters_attitude(mavlink_onboard_pid_parameters_attitude_t *get_onboard_pid_parameters_attitude)
{
    pthread_mutex_lock(&mutexReceive);
    memcpy(get_onboard_pid_parameters_attitude, &onboard_pid_parameters_attitude, sizeof(mavlink_onboard_pid_parameters_attitude_t));
    _onboard_pid_parameters_attitude = false;
    pthread_mutex_unlock(&mutexReceive);
}

void telemetry::get_onboard_pid_parameters_position(mavlink_onboard_pid_parameters_position_t *get_onboard_pid_parameters_position)
{
    pthread_mutex_lock(&mutexReceive);
    memcpy(get_onboard_pid_parameters_position, &onboard_pid_parameters_position, sizeof(mavlink_onboard_pid_parameters_position_t));
    _onboard_pid_parameters_position = false;
    pthread_mutex_unlock(&mutexReceive);
}

void telemetry::get_onboard_references_quadrotor(mavlink_onboard_references_quadrotor_t *get_onboard_references_quadrotor)
{
    pthread_mutex_lock(&mutexReceiveReferences);
    memcpy(get_onboard_references_quadrotor, &onboard_references_quadrotor, sizeof(mavlink_onboard_references_quadrotor_t));
    _onboard_references_quadrotor = false;
    pthread_mutex_unlock(&mutexReceiveReferences);
}


// Functions to send messages
void telemetry::write_attitude(float _pitch, float _roll, float _yaw, float _pitchspeed, float _rollspeed, float _yawspeed)
{
    attitude.pitch = _pitch;
    attitude.roll = _roll;
    attitude.yaw = _yaw;
    attitude.pitchspeed = _pitchspeed;
    attitude.rollspeed = _rollspeed;
    attitude.yawspeed = _yawspeed;

    pthread_mutex_lock(&mutexWrite);
    mavlink_msg_attitude_encode(system_id, component_id, &msgSend, &attitude);
    len = mavlink_msg_to_send_buffer(bufWrite, &msgSend);
    write(bufWrite, len);
    pthread_mutex_unlock(&mutexWrite);
}

void telemetry::write_global_position_int(int32_t _lat, int32_t _lon, int32_t _alt, int32_t _relative_alt, int16_t _vx, int16_t _vy, int16_t _vz, uint16_t _hdg)
{
    global_position_int.lat = _lat;
    global_position_int.lon = _lon;
    global_position_int.alt = _alt;
    global_position_int.relative_alt = _relative_alt;
    global_position_int.vx = _vx;
    global_position_int.vy = _vy;
    global_position_int.vz = _vz;
    global_position_int.hdg = _hdg;

    pthread_mutex_lock(&mutexWrite);
    mavlink_msg_global_position_int_encode(system_id, component_id, &msgSend, &global_position_int);
    len = mavlink_msg_to_send_buffer(bufWrite, &msgSend);
    write(bufWrite, len);
    pthread_mutex_unlock(&mutexWrite);
}

void telemetry::write_heartbeat(uint8_t _base_mode, uint32_t _custom_mode, uint8_t _system_status)
{
    heartbeat.base_mode = _base_mode;
    heartbeat.custom_mode = _custom_mode;
    heartbeat.system_status = _system_status;

    pthread_mutex_lock(&mutexWrite);
    mavlink_msg_heartbeat_encode(system_id, component_id, &msgSend, &heartbeat);
    len = mavlink_msg_to_send_buffer(bufWrite, &msgSend);
    write(bufWrite, len);
    pthread_mutex_unlock(&mutexWrite);
}

void telemetry::write_sys_status(uint32_t _onboard_control_sensors_present, uint32_t _onboard_control_sensors_enabled, uint32_t _onboard_control_sensors_health, uint16_t _load, uint16_t _voltage_battery, int16_t _current_battery, int8_t _battery_remaining)
{
    sys_status.onboard_control_sensors_present = _onboard_control_sensors_present;
    sys_status.onboard_control_sensors_enabled = _onboard_control_sensors_enabled;
    sys_status.onboard_control_sensors_health = _onboard_control_sensors_health;
    sys_status.load = _load;
    sys_status.voltage_battery = _voltage_battery;
    sys_status.current_battery = _current_battery;
    sys_status.battery_remaining = _battery_remaining;

    pthread_mutex_lock(&mutexWrite);
    mavlink_msg_sys_status_encode(system_id, component_id, &msgSend, &sys_status);
    len = mavlink_msg_to_send_buffer(bufWrite, &msgSend);
    write(bufWrite, len);
    pthread_mutex_unlock(&mutexWrite);
}

void telemetry::write_mission_current(uint16_t _seq)
{
    // #42 MISSION_CURRENT
    mission_current.seq = _seq;

    pthread_mutex_lock(&mutexWrite);
    mavlink_msg_mission_current_encode(system_id, component_id, &msgSend, &mission_current);
    len = mavlink_msg_to_send_buffer(bufWrite, &msgSend);
    write(bufWrite, len);
    pthread_mutex_unlock(&mutexWrite);
}

void telemetry::write_mission_item_reached(uint16_t _seq)
{
    // #46 MISSION_ITEM_REACHED
    mission_item_reached.seq = _seq;

    pthread_mutex_lock(&mutexWrite);
    mavlink_msg_mission_item_reached_encode(system_id, component_id, &msgSend, &mission_item_reached);
    len = mavlink_msg_to_send_buffer(bufWrite, &msgSend);
    write(bufWrite, len);
    pthread_mutex_unlock(&mutexWrite);
}

void telemetry::write_onboard_references_quadrotor()
{
    // #157 ONBOARD_REFERENCES_QUADROTOR
    pthread_mutex_lock(&mutexWrite);
    mavlink_msg_onboard_references_quadrotor_encode(system_id, component_id, &msgSend, &onboard_references_quadrotor);
    len = mavlink_msg_to_send_buffer(bufWrite, &msgSend);
    write(bufWrite, len);
    pthread_mutex_unlock(&mutexWrite);
}

void telemetry::write_onboard_pid_parameters_position()
{
    // #154 ONBOARD_PID_PARAMETERS_POSITION
    pthread_mutex_lock(&mutexWrite);
    mavlink_msg_onboard_pid_parameters_position_encode(system_id, component_id, &msgSend, &onboard_pid_parameters_position);
    len = mavlink_msg_to_send_buffer(bufWrite, &msgSend);
    write(bufWrite, len);
    pthread_mutex_unlock(&mutexWrite);
}

void telemetry::write_onboard_pid_parameters_attitude()
{
    // #151 ONBOARD_PID_PARAMETERS_ATTITUDE
    pthread_mutex_lock(&mutexWrite);
    mavlink_msg_onboard_pid_parameters_attitude_encode(system_id, component_id, &msgSend, &onboard_pid_parameters_attitude);
    len = mavlink_msg_to_send_buffer(bufWrite, &msgSend);
    write(bufWrite, len);
    pthread_mutex_unlock(&mutexWrite);
}

void telemetry::write_param_value()
{
    // #22 PARAM_VALUE
    // Send empty list to avoid QGroundControl asks for it
    strcpy(param_value.param_id, "No parameters");
    param_value.param_value = 0;
    param_value.param_type = MAV_PARAM_TYPE_UINT8;
    param_value.param_count = 0;
    param_value.param_index = 0;

    pthread_mutex_lock(&mutexWrite);
    mavlink_msg_param_value_encode(system_id, component_id, &msgSend, &param_value);
    len = mavlink_msg_to_send_buffer(bufWrite, &msgSend);
    write(bufWrite, len);
    pthread_mutex_unlock(&mutexWrite);
}

void telemetry::write_mission_count()
{
    // #44 MISSION_COUNT
    mission_count.target_system = system_id;
    mission_count.target_component = component_id;
    mission_count.count = mission_item.size();

    pthread_mutex_lock(&mutexWrite);
    mavlink_msg_mission_count_encode(system_id, component_id, &msgSend, &mission_count);
    len = mavlink_msg_to_send_buffer(bufWrite, &msgSend);
    write(bufWrite, len);
    pthread_mutex_unlock(&mutexWrite);

    if (mission_count.count > 0)
    {
        waitTimeout = true;
        lenPrev = len;
        memcpy(bufWritePrev, bufWrite, len);
        gettimeofday(&tTimeout, NULL); // Start timeout counter
    }
}

void telemetry::write_mission_item()
{
    // #39 MISSION_ITEM
    pthread_mutex_lock(&mutexWrite);
    mavlink_msg_mission_item_encode(system_id, component_id, &msgSend, &mission_item[mission_request.seq]);
    len = mavlink_msg_to_send_buffer(bufWrite, &msgSend);
    write(bufWrite, len);
    pthread_mutex_unlock(&mutexWrite);

    waitTimeout = true;
    lenPrev = len;
    memcpy(bufWritePrev, bufWrite, len);
    gettimeofday(&tTimeout, NULL); // Start timeout counter
}

void telemetry::write_mission_request()
{
    // #40 MISSION_REQUEST
    mission_request.target_system = system_id;
    mission_request.target_component = component_id;

    pthread_mutex_lock(&mutexWrite);
    mavlink_msg_mission_request_encode(system_id, component_id, &msgSend, &mission_request);
    len = mavlink_msg_to_send_buffer(bufWrite, &msgSend);
    write(bufWrite, len);
    pthread_mutex_unlock(&mutexWrite);

    waitTimeout = true;
    lenPrev = len;
    memcpy(bufWritePrev, bufWrite, len);
    gettimeofday(&tTimeout, NULL); // Start timeout counter
}

void telemetry::write_mission_ack()
{
    // #47 MISSION_ACK
    mission_ack.target_system = system_id;
    mission_ack.target_component = component_id;
    mission_ack.type = MAV_MISSION_ACCEPTED;

    pthread_mutex_lock(&mutexWrite);
    mavlink_msg_mission_ack_encode(system_id, component_id, &msgSend, &mission_ack);
    len = mavlink_msg_to_send_buffer(bufWrite, &msgSend);
    write(bufWrite, len);
    pthread_mutex_unlock(&mutexWrite);
}
