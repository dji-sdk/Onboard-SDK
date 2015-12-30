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

namespace DJI
{
namespace onboardSDK
{
class Script;
class Interpreter;
typedef bool (*Task)(Script *, UserData);

class TaskList
{
  public:
    TaskList(Task t = 0, UserData data = 0, time_t timeout = 0, TaskList *Pre = 0,
             TaskList *Next = 0);

  private:
    Task task;
    time_t timeout;
    UserData data;
    TaskList *next;
};

class Script
{
  public:
    Script(CoreAPI *controlAPI = 0);

    void addTaskList(TaskList *list, TaskList *pre);
    void If(Task condition, TaskList *True = 0, TaskList *False = 0);
    void Loop(Task condition, TaskList *Loop);

  public:
    static TaskList *addIf(Task condition, TaskList *True = 0, TaskList *False = 0);
    static TaskList *addLoop(Task condition, TaskList *Loop);

  private:
    TaskList *taskTree;
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
