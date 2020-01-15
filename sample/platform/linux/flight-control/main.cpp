/*! @file flight-control/main.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  main for Flight Control API usage in a Linux environment.
 *  Provides a number of helpful additions to core API calls,
 *  especially for position control, attitude control, takeoff,
 *  landing.
 *
 *  @Copyright (c) 2016-2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */
 //////////////////////////////////////////////////
#include <iostream>
#include <wiringPi.h>

#define dis 20


////////////////////////////////////////////////
#include "flight_control_sample.hpp"
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;






//////////////////////////////////////////////////////////////구조
// Ultra sonic
class dist {
	private :
		float s, e;
	public :
    float result;
		float measures(float i);
		float measuree(float j);
		int calc();
};

float dist::measures(float i) {
	s = i;
}

float dist::measuree(float j) {
	e = j;
}

int dist::calc() {
	result = (e - s) / 58;
	//std::cout << "distance(cm) : " << result << std::endl;
  return result;
}
/////////////////////////////////////////////////////////////////

//ultra sodinc pin def
#define Trig1 0
#define Echo1 1
#define Trig2 2
#define Echo2 3
#define Trig3 4
#define Echo3 5
#define Trig4 6
#define Echo4 26

int check(float avg[]);

/*! main
 *
 */
 
 
int main(int argc, char** argv)
{
  
  int cnt = 0;
  int cnt_more = 0;
  int center = 0;
  char c;   //avoiding case
  
  // Initialize variables
  int functionTimeout = 1;
  
  int x, y ,z;
  int tx =0 , ty=0 , tz=0;
  
  int result1, result2, result3, result4;
  dist d1, d2, d3, d4;

  
  float avg[12] = {0, }; 
  
  
//////////
	wiringPiSetup();

//ultra 1 -> Forward
	pinMode(Trig1, OUTPUT);
	pinMode(Echo1, INPUT);
//ultra 2 -> Back
	pinMode(Trig2, OUTPUT);
	pinMode(Echo2, INPUT);
//ultra 3 -> Left
	pinMode(Trig3, OUTPUT);
	pinMode(Echo3, INPUT);   
//ultra 4 -> Right
	pinMode(Trig4, OUTPUT);
	pinMode(Echo4, INPUT);

  
  // Setup OSDK.
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle*   vehicle = linuxEnvironment.getVehicle();
  
  
                             
                             
  if (vehicle == NULL)
  {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  // Obtain Control Authority
  vehicle->obtainCtrlAuthority(functionTimeout);

  // Display interactive prompt
  std::cout
    << "| Available commands:                                            |"
    << std::endl;
  std::cout
    << "| [a] Monitored Takeoff + Landing                                |"
    << std::endl;
  std::cout
    << "| [b] Monitored Takeoff + Position Control + Landing             |"
    << std::endl;
  char inputChar;
  std::cin >> inputChar;

  switch (inputChar)
  {
    //Telemetry::GPSInfo gps;
    //Telemetry::Vector3f localOffset;
    //Telemetry::TypeMap<TOPIC_GPS_FUSED>::type globalPosition;
    
    Telemetry::GlobalPosition globalPosition;
    
    globalPosition = vehicle->broadcast->getGlobalPosition();
    
    
    
  
    
    
    case 'a':
      monitoredTakeoff(vehicle);
      //monitoredLanding(vehicle);
      break;
    case 'b':
      monitoredTakeoff(vehicle);

    int cnt;
    cnt = 0;
    
    
     
// ultrasonic sensor    
      while(1) {
        
      while(cnt < 12){
		      
      d1re:
      //ultrasonic seonsor_1, calcuating distance    
      digitalWrite(Trig1,0);
      digitalWrite(Trig1,1);
      delayMicroseconds(10);
      digitalWrite(Trig1,0);
      while(digitalRead(Echo1) == 0)
        d1.measures(micros());
      while(digitalRead(Echo1) == 1)
        d1.measuree(micros());
        
      //std::cout<<"1st Sensor : ";
      result1 = d1.calc();
      
      if(result1 < 1000){
		  avg[cnt] = result1;
		  cnt++;
	  }
	    else{
		    goto d1re;
	  }
      

      d2re:
      //ultrasonic seonsor_2, calcuating distance
      digitalWrite(Trig2,0);
      digitalWrite(Trig2,1);
      delayMicroseconds(10);
      digitalWrite(Trig2,0);
      while(digitalRead(Echo2) == 0)
        d2.measures(micros());
      while(digitalRead(Echo2) == 1)
        d2.measuree(micros());
        
      //std::cout<<"2nd Sensor : ";
      result2 = d2.calc();
      
      if(result2 < 1000){
		  avg[cnt] = result2;
		  cnt++;
	  }
	  else{
		  goto d2re;
	  }
      
      d3re:
      //ultrasonic seonsor_3, calcuating distance
      digitalWrite(Trig3,0);
      digitalWrite(Trig3,1);
      delayMicroseconds(10);
      digitalWrite(Trig3,0);
      while(digitalRead(Echo3) == 0)
        d3.measures(micros());
      while(digitalRead(Echo3) == 1)
        d3.measuree(micros());
        
      //std::cout<<"3rd Sensor : ";
      result3 = d3.calc();
      
      
      if(result3 < 1000){
		  avg[cnt] = result3;
		  cnt++;
	  }
	  else{
		  goto d3re;
	  }
	  
	  d4re:
      //ultrasonic seonsor_4, calcuating distance
      digitalWrite(Trig4,0);
      digitalWrite(Trig4,1);
      delayMicroseconds(10);
      digitalWrite(Trig4,0);
      while(digitalRead(Echo4) == 0)
        d4.measures(micros());
      while(digitalRead(Echo4) == 1)
        d4.measuree(micros());
        
      //::cout<<"4th Sensor : ";
      result4 = d4.calc();
      
      if(result4 < 1000){
		  avg[cnt] = result4;
		  cnt++;
	  }
	  else{
		  goto d4re;
	  }
	  
      
	} //while(cnt < 12) end
      cnt = 0;
    
      
//      delay(20);
      
      std::cout<<"\n\n";    
      
      //moveByPositionOffset(vehicle, x, y, z, yaw)
      
//     if(result1 < 50)
//        moveByPositionOffset(vehicle, 5, 0, 0, 0);    // x, y, z, yaw
     int judge = check(avg);
      //std::cout << judge <<std::endl;
        
        
// avoiding switch case

      switch (judge)
       {
        case 1 : std::cout<<"case : "<<judge<<std::endl; moveByPositionOffset(vehicle, -15, 0, 0, 0);     break; //0001
        case 2 : std::cout<<"case : "<<judge<<std::endl; moveByPositionOffset(vehicle, 15, 0, 0, 0);      break; //0010
        case 4 : std::cout<<"case : "<<judge<<std::endl; moveByPositionOffset(vehicle, 0, 15, 0, 0);      break; //0100
        case 8 : std::cout<<"case : "<<judge<<std::endl; moveByPositionOffset(vehicle, 0, -15, 0, 0);     break; //1000 
        
        case 3 : std::cout<<"case : "<<judge<<std::endl; moveByPositionOffset(vehicle, 0, 15, 0, 0);      break; //0011
        case 6 : std::cout<<"case : "<<judge<<std::endl; moveByPositionOffset(vehicle, 15, 15, 0, 0);      break; //0110
        case 12 : std::cout<<"case : "<<judge<<std::endl; moveByPositionOffset(vehicle, 15, 0, 0, 0);     break; //1100
        case 5 : std::cout<<"case : "<<judge<<std::endl; moveByPositionOffset(vehicle, -15, 15, 0, 0);     break; //0101
        case 10 : std::cout<<"case : "<<judge<<std::endl; moveByPositionOffset(vehicle, 15, -15, 0, 0);    break; //1010
        case 9 : std::cout<<"case : "<<judge<<std::endl; moveByPositionOffset(vehicle, -15, -15, 0, 0);    break; //1001
        
        case 7 : std::cout<<"case : "<<judge<<std::endl; moveByPositionOffset(vehicle, 0, 15, 0, 0);     break; //0111
        case 14 : std::cout<<"case : "<<judge<<std::endl; moveByPositionOffset(vehicle, 15, 0, 0, 0);     break; //1110
        case 11 : std::cout<<"case : "<<judge<<std::endl; moveByPositionOffset(vehicle, 0, -15, 0, 0);     break; //1011
        case 13 : std::cout<<"case : "<<judge<<std::endl; moveByPositionOffset(vehicle, -15, 0, 0, 0);    break; //1101
   
        case 15 : std::cout<<"case : "<<judge<<std::endl; moveByPositionOffset(vehicle, 0, 0, 15, 0);     break; //1111
        default : moveByPositionOffset(vehicle, 0, 0, 0, 0);
      }
  
    
    
      
      //delay(10);
      
      cnt_more++;
      
        if (cnt_more == 20){
          std::cout << " more? " << std::endl;
          char o;
          std :: cin >> o;
          
          if ( o == 'c')
            break;
          else
            cnt_more = 0;
        }
   
        
      }   //while(1) end
      
      
// ultrasonic sensor end
      

      /* moveByPositionOffset(vehicle, 0, 0, 0, 0);
      moveByPositionOffset(vehicle, 0, 0, 0, 0);
      moveByPositionOffset(vehicle, 0, 0, 0, 0);
      moveByPositionOffset(vehicle, 0, 0, 0, 0);*/
    
      /*while(cnt < 10){
       if(std::abs(localOffset.x) > x)
        {
          moveByPositionOffset(vehicle, -1, 0, 0, 0);
          tx -= x;
        }
        else if(std::abs(localOffset.x)< x)
        {
          moveByPositionOffset(vehicle, 1, 0, 0, 0);
          tx -= x;
        } */
        /*if(localOffset.x != x)
        { center = x-localOffset.x;
          moveByPositionOffset(vehicle, center
          * , 0, 0, 0);
        }
        std::cout
          <<cnt<<", "<< std::abs(localOffset.x) <<std::endl;
        cnt++;
      }*/
                
      
      moveByPositionOffset(vehicle, tx, ty, tz, 0); 
      monitoredLanding(vehicle);
      break;
      
    default:
      break;
  }

  return 0;
}
/*
char avoiding()
{
  float calc_avg(dist d);
  char c;
  dist d1, d2, d3, d4;
  // avoiding
    delay(100);
     if(calc_avg(d1)<dis + 10) {
        c = 'A';
        if(calc_avg(d2)<dis) {
          c = 'E';
          if(calc_avg(d3)<dis) {
            c = 'K';
            if(calc_avg(d4)<dis) {
              c = 'O';
            }
          }
          else if(calc_avg(d4)<dis) {
            c = 'M';
          }
        }
          
        else if(calc_avg(d3)<dis){
          c = 'H';
          if(d4.result<dis){
            c = 'N';
          }
          else if(calc_avg(d2)<dis){
          c = 'K';
          }
        }
        else if(calc_avg(d4)<dis){
          c = 'J';
        	if(calc_avg(d2)<dis){
            c = 'L';
            if(calc_avg(d3)<dis){
              c = 'O';
            }
          }
          else if(calc_avg(d3)<dis){
            c = 'N';
            if(calc_avg(d2)<dis){
              c = 'O';
            }
          }
        }
      } 

      else if(calc_avg(d2)<dis - 10){
        c = 'B';
        if(calc_avg(d3)<dis){
          c = 'F';
          if(calc_avg(d4)<dis){
            c = 'L';
          }
        }
        else if(calc_avg(d4)<dis){
          c = 'I';
        }
      }

      else if(calc_avg(d3)<dis){
        c = 'C';
        if(calc_avg(d4)<dis){
          c = 'G';
        }
      }

      else if(calc_avg(d4)<dis){
        c = 'D';
      }

      else
      	c='P';
        
      return c;
}
*/
/*

float calc_avg(dist d)
{
  int cnt=0;
  float avg[5] = {0,};
  float avg_d = 0;
  
  if(d.result < 1000){
    if(cnt==5) {
      avg_d = (avg[0] + avg[1] + avg[2] + avg[3] + avg[4])/5;
      
      cnt = 0;
    }
    else
    {
      avg[cnt]=d.result;
      cnt++;
    }
  }
  
  return avg_d;
}

*/

int check(float avg[])
{
  int re =0;
  
  float avg_d[4] = {0, };
  
  for(int i = 0; i < 4; i++){
    
    avg_d[i] = (avg[i+0] + avg[i+4] + avg[i+8])/3;
    
  }
  
  for(int i = 0; i < 4; i++)
  {
    std::cout <<i+1<<"'s avg_distance(cm) : " << avg_d[i] << std::endl;
  }
  
  for(int i = 0; i < 4; i++){
    if(avg_d[i] < dis){
      switch(i){
        case 0 : re = re + 1; break;
        case 1 : re = re + 2; break;
        case 2 : re = re + 4; break;
        case 3 : re = re + 8; break;
      }
    }
    else re = re + 0;
  }
  
  return re; 
    
}
