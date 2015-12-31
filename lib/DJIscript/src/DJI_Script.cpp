#include "DJI_Script.h"

using namespace DJI::onboardSDK;

Script::Script(CoreAPI *controlAPI)
{
    api = controlAPI;

    virtualRC = new VirtualRC(api);
    hotpoint = new HotPoint(api);
    waypoint = new WayPoint(api);
    flight = new Flight(api);
    camera = new Camera(api);
    follow = new Follow(api);
}

void Script::run()
{

}

TaskList::TaskList(Task t, UserData Data, time_t Timeout, TaskList *Pre, TaskList *Next)
{

    TaskList *insert = 0;
    TaskList *tail = 0;
    task = t;
    data = Data;
    timeout = Timeout;
    this->next = Next;
    if (Pre != 0)
    {
        if (Pre->next != 0)
        {
            insert = Pre->next;
        }
        Pre->next = this;
    }
    tail = this->next;
    while (tail->next != 0)
    {
        tail = tail->next;
    }
    tail->next = insert;
}

TaskList::TaskList(const TaskList &)
{

}
