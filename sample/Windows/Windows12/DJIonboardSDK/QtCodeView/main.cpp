#include <iostream>
#include "D:\github\onboard-sdk\samples\Windows\Windows12\DJIonboardSDK\DJIonboardSDK\WindowsHardDriver.h"
using namespace std;

void fun(int port)
{
    cout<<"get!"<<port;
}

int main()
{
    _asynCom com;
    com.setBuf(2048,2048);
    com.open(3,115200);
    //com.restart();
    cout << "Hello World!" << endl;
    while(1)
    {

    }
    return 0;
}

