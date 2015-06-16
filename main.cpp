/*==============================================================================
    Pablo Albiol http://github.com/PabloAlbiol
==============================================================================*/


#include <iostream>
#include <cstdio>
#include <time.h>
#include <sys/time.h>

#include "telemetry.h"
#include "utilities.h"
#include "MAVLink/pabloag/mavlink.h"

#define PERIODMAIN 10 // Period in ms


bool run = true;

// Control + C to stop the program
void signalHandler( int signum )
{
    std::cout << "Signal " << signum << " received" << std::endl;
    run = false;
}

int main()
{
    telemetry *tele;
    struct timeval tStart, tEnd;
    long int elapTime;
    uint16_t div1 = 0, div2 = 1; // The counter starts at different values to avoid messages to be sent in the same (main) cycle

    // MAVLink related variables
    uint8_t base_mode = MAV_MODE_MANUAL_DISARMED;
    uint32_t custom_mode = 0;
    uint8_t system_status = MAV_STATE_STANDBY;
    float pitch = 0.1, roll = 0.2, yaw = -0.1, pitchspeed = 0.22, rollspeed = 0, yawspeed = 1.25;
    uint8_t seq = 0;
    mavlink_onboard_pid_parameters_attitude_t onboard_pid_parameters_attitude;
    std::vector<mavlink_mission_item_t> mission_item;

    signal(SIGINT, signalHandler);

    tele = new telemetry();
    // Setup telemetry (open the link)
    tele->setupTelemetry();
    // Start telemetry (create the thread)
    tele->startTelemetry();


    while (run)
    {
        gettimeofday(&tStart, NULL);

        /** Check for received messages*/
        if (tele->b_set_mode()) // Check if a message has been received
        {
            tele->get_set_mode(&base_mode, &custom_mode); // If a message has been received, get the message
        }

        if (tele->b_mission_set_current())
        {
            tele->get_mission_set_current(&seq);
            tele->write_mission_current(seq); // Send a message when it's received
        }

        if (tele->b_onboard_pid_parameters_attitude())
        {
            tele->get_onboard_pid_parameters_attitude(&onboard_pid_parameters_attitude);
        }

        if (tele->b_mission_item())
        {
            mission_item.clear();
            tele->get_mission_item(&mission_item);
        }


        /** Messages regularly sent*/
        if (div1 > 20) // Send a heartbeat message regularly (1 time every 20 (main) cycles)
        {
            tele->write_heartbeat(base_mode, custom_mode, system_status);
            div1 = 0;
        }

        if (div2 > 10) // Send an attitude message regularly (1 time every 10 (main) cycles)
        {
            tele->write_attitude(pitch, roll, yaw, pitchspeed, rollspeed, yawspeed);
            div2 = 0;
        }

        div1++;
        div2++;

        gettimeofday(&tEnd, NULL);
        elapTime = timeDiff(&tEnd, &tStart);

        if (elapTime < PERIODMAIN*1000)
        {
            usleep(PERIODMAIN*1000 - elapTime);
        }
        else
        {
            std::cout << "Error Main period" << std::endl;
        }
    }

    delete tele;

    return 0;
}
