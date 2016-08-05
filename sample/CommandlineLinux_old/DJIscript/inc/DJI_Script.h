/*! @file DJI_Script.h
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  DJI Script: Core interface for DJI Onboard SDK command line example
 *  
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#ifndef DJI_SCRIPT_H
#define DJI_SCRIPT_H

#include <DJI_API.h>
//! @todo replace Flight and VRC to DJI-Drone
#include <DJI_Flight.h>
#include <DJI_Camera.h>
#include <DJI_Follow.h>
#include <DJI_HotPoint.h>
#include <DJI_WayPoint.h>
#include <DJI_VirtualRC.h>

#define TASK_ITEM(x, r)                          \
  {                                              \
    (UserData) #x, x, r                          \
  }
namespace DJI
{
namespace onboardSDK
{
class Script;
class Interpreter;
typedef bool (*Task)(DJI::onboardSDK::Script *, DJI::UserData);

//! The TaskList class creates a linked list of Tasks.
class TaskList
{
  public:
  TaskList(Task t = 0, UserData Data = 0, int Repeat = 0, time_ms Timeout = 0,
      TaskList *Pre = 0, TaskList *Next = 0);

  void run(Script *s);

  TaskList *getNext() const;
  TaskList *tail();
  void insert(TaskList *list);
  void setNext(TaskList *value);

  Task getTask() const;
  void setTask(const Task &value);

  private:
  TaskList(const TaskList &); //! @note TaskList can not be copied

  private:
  Task task;
  time_ms start; //! @note for time management and allocation
  time_ms timeout;
  UserData data;
  TaskList *next;
  int repeat;
};

typedef struct TaskSetItem
{
  UserData name;
  Task task;
  int repeat;
} TaskSetItem;

//! The Script class provides a framework for interactive operation of the Onboard SDK. 
class Script
{

  public:
  Script(CoreAPI *controlAPI = 0, TaskSetItem *set = 0, size_t SetSize = 0);

  void addTaskList(TaskList *list, TaskList *pre = 0);
  void addTask(Task t, UserData Data = 0, int Repeat = 0, time_ms Timeout = 0);
  bool addTask(UserData name, UserData Data = 0, time_ms Timeout = 0);

  void If(Task condition, TaskList *True = 0, TaskList *False = 0);
  void Loop(Task condition, TaskList *Loop);

  //! @note run must poll in a independent thread.
  void run();

  virtual TaskSetItem match(UserData name);
  virtual bool quitCurrent();

  public:
  static void run(Script *script);

  static TaskList *addIf(Task condition, TaskList *True = 0, TaskList *False = 0);
  static TaskList *addLoop(Task condition, TaskList *Loop);

  static bool emptyTask(Script *script, UserData data __UNUSED);

  public:
  ActivateData adata;

  void setApi(CoreAPI *value) { api = value; }
  void setFlight(Flight *value) { flight = value; }
  void setFollow(Follow *value) { follow = value; }
  void setCamera(Camera *value) { camera = value; }
  void setHotpoint(HotPoint *value) { hotpoint = value; }
  void setVirtualRC(VirtualRC *value) { virtualRC = value; }
  void setWaypoint(WayPoint *value) { waypoint = value; }

  CoreAPI *getApi() const { return api; }
  Flight *getFlight() const { return flight; }
  Follow *getFollow() const { return follow; }
  HotPoint *getHotpoint() const { return hotpoint; }
  VirtualRC *getVirtualRC() const { return virtualRC; }
  Camera *getCamera() const { return camera; }
  WayPoint *getWaypoint() const { return waypoint; }

  private:
  TaskList *taskTree;
  TaskSetItem *taskSet;
  size_t setSize;
  CoreAPI *api;

  VirtualRC *virtualRC;
  HotPoint *hotpoint;
  WayPoint *waypoint;
  Flight *flight;
  Camera *camera;
  Follow *follow;
};
} // namespace onboardSDK
} // namespace DJI

#endif // DJI_SCRIPT_H
