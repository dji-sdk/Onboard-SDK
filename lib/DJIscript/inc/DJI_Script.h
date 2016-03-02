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

#define TASK_ITEM(x, r)                                                                        \
    {                                                                                          \
        (UserData) #x, x, r                                                                    \
    }
namespace DJI
{
namespace onboardSDK
{
class Script;
class Interpreter;
typedef bool (*Task)(DJI::onboardSDK::Script *, DJI::onboardSDK::UserData);

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

class Script
{
    /*! @note
     * class Script only offers a platform frame for scriptional flight control,
     * like command line.
     * */
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

    void setApi(CoreAPI *value);
    void setFlight(Flight *value);
    void setFollow(Follow *value);
    void setCamera(Camera *value);
    void setHotpoint(HotPoint *value);
    void setVirtualRC(VirtualRC *value);
    void setWaypoint(WayPoint *value);

    CoreAPI *getApi() const;
    Flight *getFlight() const;
    Follow *getFollow() const;
    HotPoint *getHotpoint() const;
    VirtualRC *getVirtualRC() const;
    Camera *getCamera() const;
    WayPoint *getWaypoint() const;

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
