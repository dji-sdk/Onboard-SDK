/*! @file cmdSettings.cpp
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  Script Settings command for DJI Onboard SDK Command line example
 *  
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#include "cmdIO.h"
#include "cmdSettings.h"

#include <string.h>
#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;
using namespace DJI;
using namespace DJI::onboardSDK;

bool SS(Script* script, UserData data)
{
  char* inputData = (char*)data;
  char command[100];
  sscanf(inputData, "--%s", command);
  if (strcmp(command, "help") == 0)
  {
    cout << "|------------------DJI onboardSDK - Script Settings--------------|" << endl;
    cout << "|  Usage:                                                        |" << endl;
    cout << "|  --SS <command> <data>                                         |" << endl;
    cout << "|  Currently, only one command is supported:                     |" << endl;
    cout << "|  - load <file name> load and print id and key                  |" << endl;
    cout << "|------------------DJI onboardSDK - Script Settings--------------|" << endl;

    script->addTask(waitInput);
    script->addTask(waitInput);
  }
  else
  {
    if (sscanf(inputData, "--%*s%s", command))
    {
      strcat(command, "SS");
      script->addTask((UserData)command, data);
    }
    else
      script->addTask(addTask);
  }
  return true;
}

bool loadSS(Script* script, UserData data)
{
  char* inputData = (char*)data;
  char command[100];
  char line[1024];
  static char key[50];

  bool set_ID, set_KEY;

  if (sscanf(inputData, "--%*s%*s%s", command))
    cout << "Path: " << command << endl;
  else
    memcpy(command, "settings.ini", 13);
  ifstream read(command);

  if (read.is_open())
    while (!read.eof())
    {
      read.getline(line, 1024);
      if (*line != 0) //! @note sscanf have features on empty buffer.
      {
        int id;
        if (sscanf(line, "ID:%d", &id))
        {
          script->adata.ID = id;
          set_ID = true;
        }
        script->adata.reserved = 2;
        script->adata.version = SDK_VERSION;
        if (sscanf(line, "KEY:%s", key))
        {
          script->adata.encKey = key;
          set_KEY = true;
        }
      }
      cout << line << endl;
    }
  else
    cout << "can not open file" << endl;
  read.close();
  __DELETE(data);

  if (!set_KEY)
  {
    cout << "There's a problem with your Key. Make sure the format is correct." << endl;
    script->addTask(waitInput);
    return false;  
  }
  if (!set_ID)
  {
    cout << "There's a problem with your ID. Make sure the format is correct." << endl;
    script->addTask(waitInput);
    return false;  
  }
  script->addTask(waitInput);
  return true;
}

bool idSS(Script* script, UserData data)
{
  //! @todo idss
  cout << "not available yet." << endl;
  __DELETE(data);
  script->addTask(waitInput);
  return true;
}

bool keySS(Script* script, UserData data)
{
  //! @todo keySS
  cout << "not available yet." << endl;
  __DELETE(data);
  script->addTask(waitInput);
  return true;
}

bool saveSS(Script* script, UserData data)
{
  //! @todo saveSS
  cout << "not available yet." << endl;
  __DELETE(data);
  script->addTask(waitInput);
  return true;
}

bool spSS(Script* script, UserData data)
{
  //! @todo spSS
  cout << "not available yet." << endl;
  __DELETE(data);
  script->addTask(waitInput);
  return true;
}
