/*! @file DJI_Script.cpp
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  DJI Script: Core interface for DJI Onboard SDK command line example
 *  
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#include "DJI_Script.h"
#include <iostream>
#include <string.h>

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
  TaskList *list = new TaskList(t, Data, Repeat, Timeout);
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
//  if (kbhit())
//    return true;
  return false;
}

void Script::run(Script *script) { script->run(); }

bool Script::emptyTask(Script *script __UNUSED, UserData data __UNUSED)
{
  //! @todo implement
  return true;
}

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
      //! @todo better algorithm
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
