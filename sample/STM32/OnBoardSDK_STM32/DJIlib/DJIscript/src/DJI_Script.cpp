#include "DJI_Script.h"
#include <iostream>
#include <string.h>

#include <iostream>

#ifdef __linux__
#include <sys/select.h>

int kbhit()
{
  struct timeval tv;
  fd_set read_fd;

  /* Do not wait at all, not even a microsecond */
  tv.tv_sec=0;
  tv.tv_usec=0;

  /* Must be done first to initialize read_fd */
  FD_ZERO(&read_fd);

  /* Makes select() ask if input is ready:
   * 0 is the file descriptor for stdin    */
  FD_SET(0,&read_fd);

  /* The first parameter is the number of the
   * largest file descriptor to check + 1. */
  if(select(1, &read_fd,NULL, /*No writes*/NULL, /*No exceptions*/&tv) == -1)
    return 0;  /* An error occured */

  /*  read_fd now holds a bit map of files that are
   * readable. We test the entry for the standard
   * input (file 0). */
  
if(FD_ISSET(0,&read_fd))
    /* Character pending on stdin */
    return 1;

  /* no characters were pending */
  return 0;
}
#elif _WIN32
#include <conio.h>
#endif

using namespace DJI;
using namespace DJI::onboardSDK;
using namespace std;

Script::Script(CoreAPI *controlAPI, TaskSetItem *set, size_t SetSize)
{
    api = controlAPI;

    virtualRC = new VirtualRC(api);
    hotpoint = new HotPoint(api);
    waypoint = new WayPoint(api);
    flight = new Flight(api);
    camera = new Camera(api);
    follow = new Follow(api);

    taskTree = new TaskList(Script::emptyTask);

    taskSet = set;
    setSize = SetSize;
}

void Script::addTaskList(TaskList *list, TaskList *pre)
{
    if (pre)
        pre->insert(list);
    else
    {
        if (taskTree == 0)
            taskTree = new TaskList(Script::emptyTask);
        taskTree->tail()->setNext(list);
    }
}

void Script::addTask(Task t, UserData Data, int Repeat, time_ms Timeout)
{
    TaskList *list = new TaskList(t, Data, Repeat,Timeout);
    addTaskList(list);
}

bool Script::addTask(UserData name, UserData Data, time_ms Timeout)
{
    TaskSetItem t = match(name);
    if (t.task)
        addTaskList(new TaskList(t.task, Data, t.repeat, Timeout));
    else
        return false;
    return true;
}

void Script::run()
{
    cout << endl;
    while (taskTree != 0)
    {
        taskTree->run(this);
        TaskList *destroy = taskTree;
        taskTree = taskTree->getNext();
        if (destroy)
            delete destroy;
    }
    cout << endl;
}

TaskSetItem Script::match(UserData name)
{
    TaskSetItem empty;
    empty.task = 0;
    char *data = (char *)name;
    for (unsigned int i = 0; i < setSize; ++i)
        if (strcmp(data, (char *)taskSet[i].name) == 0)
            return taskSet[i];
    return empty;
}

bool Script::quitCurrent()
{
    if (kbhit())
        return true;
    return false;
}

void Script::run(Script *script) { script->run(); }

bool Script::emptyTask(Script *script __UNUSED, UserData data __UNUSED) { return true; }
WayPoint *Script::getWaypoint() const
{
    return waypoint;
}

void Script::setWaypoint(WayPoint *value)
{
    waypoint = value;
}

Camera *Script::getCamera() const
{
    return camera;
}

void Script::setCamera(Camera *value)
{
    camera = value;
}

VirtualRC *Script::getVirtualRC() const { return virtualRC; }

void Script::setVirtualRC(VirtualRC *value) { virtualRC = value; }

HotPoint *Script::getHotpoint() const { return hotpoint; }

void Script::setHotpoint(HotPoint *value) { hotpoint = value; }

Follow *Script::getFollow() const { return follow; }

void Script::setFollow(Follow *value) { follow = value; }

Flight *Script::getFlight() const { return flight; }

void Script::setFlight(Flight *value) { flight = value; }

CoreAPI *Script::getApi() const { return api; }

void Script::setApi(CoreAPI *value) { api = value; }

TaskList::TaskList(Task t, UserData Data, int Repeat, time_ms Timeout, TaskList *Pre,
                   TaskList *Next)
{
    task = t;
    data = Data;
    timeout = Timeout;
    start = 0;
    next = 0;
    repeat = Repeat;

    tail()->next = Next;
    if (Pre)
        Pre->insert(this);
}

TaskList::TaskList(const TaskList &) {}

void TaskList::run(Script *s)
{
    if (task)
        do
        {
            if (repeat > 0)
                repeat--;
            task(s, data);
            if (s->quitCurrent())
                break;
        } while (repeat);
}
void TaskList::setTask(const Task &value) { task = value; }
void TaskList::setNext(TaskList *value) { next = value; }
Task TaskList::getTask() const { return task; }
TaskList *TaskList::getNext() const { return next; }

TaskList *TaskList::tail()
{
    TaskList *ans = this;
    while (ans->next != 0) ans = ans->next;
    return ans;
}

void TaskList::insert(TaskList *list)
{
    list->tail()->next = this->next;
    this->next = list;
}
