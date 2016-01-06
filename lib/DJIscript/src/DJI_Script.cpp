#include "DJI_Script.h"
#include <iostream>
#include <string.h>

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

void Script::addTask(Task t, UserData Data, time_t Timeout)
{
    TaskList *list = new TaskList(t, Data, Timeout);
    addTaskList(list);
}

bool Script::addTask(UserData name, UserData Data, time_t Timeout)
{
    Task t = match(name);
    if (t)
        addTaskList(new TaskList(t, Data, Timeout));
    else
        return false;
    return true;
}

void Script::run()
{
    //API_LOG(api->getDriver(), STATUS_LOG, "Start");
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
    //API_LOG(api->getDriver(), STATUS_LOG, "Finish");
}

Task Script::match(UserData name)
{
    char *data = (char *)name;
    for (unsigned int i = 0; i < setSize; ++i)
        if (strcmp(data, (char *)taskSet[i].name) == 0)
            return taskSet[i].task;
    return 0;
}

void Script::run(Script *script) { script->run(); }

bool Script::emptyTask(Script *script __UNUSED, UserData data __UNUSED) { return true; }
CoreAPI *Script::getApi() const
{
    return api;
}

void Script::setApi(CoreAPI *value)
{
    api = value;
}


TaskList::TaskList(Task t, UserData Data, time_t Timeout, TaskList *Pre, TaskList *Next)
{
    task = t;
    data = Data;
    timeout = Timeout;
    start = 0;
    next = 0;

    tail()->next = Next;
    if (Pre)
        Pre->insert(this);
}

TaskList::TaskList(const TaskList &) {}

void TaskList::run(Script *s)
{
    if (task)
        task(s, data);
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
