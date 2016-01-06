#include "cmdIO.h"
#include "cmdCoreAPI.h"

#include <string.h>
#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;
using namespace DJI::onboardSDK;

bool CA(Script* script __UNUSED, UserData data)
{
        char* inputData = (char*)data;
        char command[100];
        sscanf(inputData, "--%s", command);
        if (strcmp(command, "help") == 0)
        {
                cout << "|------------------DJI onboardSDK - Core API--------------------|" << endl;
                cout << "| --CA <command> <data>                                         |" << endl;
                cout << "| - ac <N/A> activate, load data from files. See also SS load   |" << endl;
                cout << "| - vs <N/A> read flight control version                        |" << endl;
                cout << "| - bf {<channel> <frequency>} read flight control version      |" << endl;
                cout << "|      <channel> 0 ~ 15                                         |" << endl;
                cout << "|      <frequency> 0 1 10 50 100                                |" << endl;
                cout << "|                  other input will hold the latest frequency   |" << endl;
                cout << "| - bd <flag> show broadcast data based on flag.                |" << endl;
                cout << "|      <flag> 0x0001 time                                       |" << endl;
                cout << "|             0x0002 quaternion                                 |" << endl;
                cout << "|             0x0004 acceleration                               |" << endl;
                cout << "|             0x0008 velocity                                   |" << endl;
                cout << "|             0x0010 palstance                                  |" << endl;
                cout << "|             0x0020 possession                                 |" << endl;
                cout << "|             0x0040 magnet                                     |" << endl;
                cout << "|             0x0080 remote controller                          |" << endl;
                cout << "|             0x0100 gimbal                                     |" << endl;
                cout << "|             0x0200 device status                              |" << endl;
                cout << "|             0x0400 battery                                    |" << endl;
                cout << "| - ct <bool> true for obtain control, false for release control|" << endl;
                cout << "| - sy <freq> set synchronized frequency in HZ                  |" << endl;
                cout << "|------------------DJI onboardSDK - Core API--------------------|" << endl;

                __DELETE(data);
                script->addTask(waitInput);
        }
        else
        {
                sscanf(inputData, "--%*s%s", command);
                strcat(command, "CA");
                script->addTask((UserData)command, data);
        }
        return true;
}

bool bdCA(Script* script, UserData data)
{
        stringstream s;

        s << (char*)data;
        char drop[20];
        s >> drop >> drop;
        uint16_t flag;
        s >> flag;
        // cout << hex << flag << endl;

        BroadcastData bd = script->getApi()->getBroadcastData();
        for (int i = 0; i < 11; ++i)
        {
                if (flag & (1 << i))
                {
                    switch (i)
                    {
                            case 0:
                                    cout << bd.timeStamp.time << " " << bd.timeStamp.asr_ts << " "
                                             << bd.timeStamp.sync_flag << endl;
                                    break;
                    }
                }
        }
}

bool bfCA(Script* script, UserData data)
{
        stringstream s;

        s << (char*)data;
        char drop[20];
        s >> drop >> drop;
        uint8_t freq[16];
        while (!s.eof())
        {
                int channel, freqData;
                s >> channel >> freqData;
                switch (freqData)
                {
                        case 0:
                                freq[channel] = BROADCAST_FREQ_0HZ;
                                break;
                        case 1:
                                freq[channel] = BROADCAST_FREQ_1HZ;
                                break;
                        case 10:
                                freq[channel] = BROADCAST_FREQ_10HZ;
                                break;
                        case 50:
                                freq[channel] = BROADCAST_FREQ_50HZ;
                                break;
                        case 100:
                                freq[channel] = BROADCAST_FREQ_100HZ;
                                break;
                        default:
                                freq[channel] = BROADCAST_FREQ_HOLD;
                                break;
                }
        }

        script->getApi()->setBroadcastFreq(freq);

        __DELETE(data);
        script->addTask(waitInput);

        return true;
}

bool acCA(Script* script, UserData data)
{
        script->getApi()->activate(&script->adata);

        __DELETE(data);
        script->addTask(waitInput);
        return true;
}

bool vsCA(Script* script, UserData data)
{
        script->getApi()->getVersion();

        __DELETE(data);
        script->addTask(waitInput);
        return true;
}
