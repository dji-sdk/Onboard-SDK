// Copyright 2013 Velodyne Acoustics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkVelodyneHDLPositionReader.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "vtkVelodyneHDLPositionReader.h"

#include "vtkPacketFileReader.h"
#include "vtkPacketFileWriter.h"
#include "vtkVelodyneTransformInterpolator.h"

#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkDataArray.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkMath.h>
#include <vtkMath.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyLine.h>
#include <vtkSmartPointer.h>
#include <vtkStreamingDemandDrivenPipeline.h>
#include <vtkTransform.h>
#include <vtkTupleInterpolator.h>
#include <vtkUnsignedCharArray.h>
#include <vtkUnsignedIntArray.h>
#include <vtkUnsignedShortArray.h>

#include <vtk_libproj4.h>

#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <algorithm>
#include <map>
#include <sstream>

#include <cmath>

#ifdef _MSC_VER
# include <boost/cstdint.hpp>
typedef boost::uint8_t uint8_t;
#else
# include <stdint.h>
#endif

namespace
{
struct PositionPacket
{
  unsigned int gpsTimestamp;
  short gyro[3];
  short temp[3];
  short accelx[3];
  short accely[3];
  char sentance[73];
};
}

//-----------------------------------------------------------------------------
class vtkVelodyneHDLPositionReader::vtkInternal
{
public:

  vtkInternal()
  {
    this->Reader = 0;
    this->UTMZone = -1;
    this->Offset[0] = 0.0;
    this->Offset[1] = 0.0;
    this->Offset[2] = 0.0;
  }

  int ProcessHDLPacket(const unsigned char *data, unsigned int bytes, PositionPacket& position);
  std::vector<std::string> ParseSentance(const std::string& sentance);

  void InterpolateGPS(vtkPoints* points, vtkDataArray* gpsTime, vtkDataArray* times, vtkDataArray* heading);

  vtkPacketFileReader* Reader;
  int UTMZone;
  std::string UTMString;
  double Offset[3];

  vtkNew<vtkVelodyneTransformInterpolator> Interp;
};

namespace
{
  const unsigned short BIT_12_MASK = 0x0fff;
  const unsigned short REMAINDER_12_MASK = 0x07ff;
  const unsigned short SIGN_12_MASK = 0x0800;

  const double GYRO_SCALE = 0.09766; // deg / s
  const double TEMP_SCALE = 0.1453; // C
  const double TEMP_OFFSET = 25.0; // C
  const double ACCEL_SCALE = 0.001221; // G
}

//-----------------------------------------------------------------------------
int vtkVelodyneHDLPositionReader::vtkInternal::ProcessHDLPacket(const unsigned char *data, unsigned int bytes, PositionPacket& position)
{
  if(bytes != 512)
    {
    return 0;
    }

  for (int i = 0; i < 14; ++i)
    {
    if (data[i] != 0)
      {
      std::cerr << "unexpected data in first zeros block\n";
      return 0;
      }
    }

  for(int i = 0; i < 3; ++i)
    {
    memcpy(position.gyro + i, data + 14 + i*8, 2);
    memcpy(position.temp + i, data + 14 + i*8 + 2, 2);
    memcpy(position.accelx + i, data + 14 + i*8 + 4, 2);
    memcpy(position.accely + i, data + 14 + i*8 + 6, 2);
    }

  for(int i = 0; i < 3; ++i)
    {
    // Selector only least significant 12 bits
    position.gyro[i] &= BIT_12_MASK;
    position.temp[i] &= BIT_12_MASK;
    position.accelx[i] &= BIT_12_MASK;
    position.accely[i] &= BIT_12_MASK;

    // Perform 12 bit twos complement
    position.gyro[i] = -2048*((position.gyro[i] & SIGN_12_MASK) >> 11) +
      (position.gyro[i] & REMAINDER_12_MASK);
    position.temp[i] = -2048*((position.temp[i] & SIGN_12_MASK) >> 11) +
      (position.temp[i] & REMAINDER_12_MASK);
    position.accelx[i] = -2048*((position.accelx[i] & SIGN_12_MASK) >> 11) +
      (position.accelx[i] & REMAINDER_12_MASK);
    position.accely[i] = -2048*((position.accely[i] & SIGN_12_MASK) >> 11) +
      (position.accely[i] & REMAINDER_12_MASK);
    }

  memcpy(&position.gpsTimestamp, data + 14 + 8+8+8 + 160, 4);

  std::copy(data + 14 + 8 + 8 + 8 + 160 + 4 + 4,
            data + 14 + 8 + 8 + 8 + 160 + 4 + 4 + 72,
            position.sentance);
  position.sentance[72] = '\0';

  return 1;
}

//-----------------------------------------------------------------------------
std::vector<std::string> vtkVelodyneHDLPositionReader::vtkInternal::ParseSentance(const std::string& sentance)
{
  std::stringstream sstr(sentance);
  std::string token;

  std::vector<std::string> result;

  while(std::getline(sstr, token, ','))
    {
    result.push_back(token);
    }

  return result;
}

//-----------------------------------------------------------------------------
vtkVelodyneTransformInterpolator*
vtkVelodyneHDLPositionReader::GetInterpolator()
{
  return this->Internal->Interp.GetPointer();
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLPositionReader::vtkInternal::InterpolateGPS(vtkPoints* points, vtkDataArray* gpsTime, vtkDataArray* times, vtkDataArray* headings)
{
  vtkNew<vtkTupleInterpolator> headingInterpolator;

  // assert(gpsTime is sorted)
  assert(points->GetNumberOfPoints() == times->GetNumberOfTuples());

  this->Interp->SetInterpolationTypeToLinear();
  this->Interp->Initialize();
  headingInterpolator->SetInterpolationTypeToLinear();
  headingInterpolator->SetNumberOfComponents(2);

  double timeOffset = 0.0;

  assert(times->GetNumberOfTuples() == gpsTime->GetNumberOfTuples());
  unsigned int lastGPS = 0;
  for (vtkIdType i = 0, k = times->GetNumberOfTuples(); i < k; ++i)
    {
    const unsigned int currGPS = gpsTime->GetTuple1(i);
    if (currGPS != lastGPS)
      {
      if (currGPS < lastGPS)
        {
        // time of day has wrapped; increment time offset
        timeOffset += 24.0 * 3600.0;
        }
      lastGPS = currGPS;

      // Compute time in seconds from decimal-encoded time
      const int hours = currGPS / 10000;
      const int minutes = (currGPS / 100) % 100;
      const int seconds = currGPS % 100;
      const double convertedtime = ((3600* hours) + (60.0 * minutes) + seconds) + timeOffset;

      // Get position and heading
      double pos[3];
      points->GetPoint(i, pos);

      const double heading = headings->GetTuple1(i);

      // Compute transform
      vtkNew<vtkTransform> transform;
      transform->PostMultiply();
      transform->RotateZ(-heading/* - this->BaseYaw*/);
      // transform->RotateY(-this->BaseRoll);
      // transform->RotateX(-this->BasePitch);
      transform->Translate(pos);

      this->Interp->AddTransform(convertedtime, transform.GetPointer());

      // Compute heading vector for interpolation
      double ha = heading * DEG_TO_RAD;
      double hv[2] = { cos(ha), sin(ha) };
      headingInterpolator->AddTuple(convertedtime, hv);
      }
    }
}

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkVelodyneHDLPositionReader);

//-----------------------------------------------------------------------------
vtkVelodyneHDLPositionReader::vtkVelodyneHDLPositionReader()
{
  this->Internal = new vtkInternal;
  this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(1);
}

//-----------------------------------------------------------------------------
vtkVelodyneHDLPositionReader::~vtkVelodyneHDLPositionReader()
{
  delete this->Internal;
}

//-----------------------------------------------------------------------------
const std::string& vtkVelodyneHDLPositionReader::GetFileName()
{
  return this->FileName;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLPositionReader::SetFileName(const std::string& filename)
{
  if (filename == this->FileName)
    {
    return;
    }

  this->FileName = filename;
  this->Modified();
}

namespace
{

int LatLongToZone(double lat, double lon)
{
  double longTemp = (lon+180)-static_cast<int>((lon+180)/360)*360-180;

  double latRad = lat * DEG_TO_RAD;
  double lonRad = lon * DEG_TO_RAD;

  int zone = static_cast<int>((longTemp + 180) / 6) + 1;
  if(lat >= 56.0 && lat < 64.0 && longTemp >= 3.0 && longTemp < 12.0)
    {
    zone = 32;
    }

  if(lat >= 72.0 && lat < 84)
    {
    if(longTemp >= 0.0 && longTemp < 9.0)
      {
      zone = 31;
      }
    else if(longTemp >= 9.0 && longTemp < 21.0)
      {
      zone = 33;
      }
    else if(longTemp >= 21.0 && longTemp < 33.0)
      {
      zone = 35;
      }
    else if(longTemp >= 33.0 && longTemp < 42.0)
      {
      zone = 37;
      }
    }

  return zone;
}

}

//-----------------------------------------------------------------------------
int vtkVelodyneHDLPositionReader::RequestData(vtkInformation *request,
                              vtkInformationVector **inputVector,
                              vtkInformationVector *outputVector)
{
  vtkPolyData *output = vtkPolyData::GetData(outputVector);
  vtkInformation *info = outputVector->GetInformationObject(0);

  if (!this->FileName.length())
    {
    vtkErrorMacro("FileName has not been set.");
    return 0;
    }

  this->Internal->UTMZone = -1;

  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
  vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New();
  vtkSmartPointer<vtkIdList> polyIds = polyLine->GetPointIds();

  // Data arrays
  vtkSmartPointer<vtkDoubleArray> lats = vtkSmartPointer<vtkDoubleArray>::New();
  lats->SetName("lat");
  vtkSmartPointer<vtkDoubleArray> lons = vtkSmartPointer<vtkDoubleArray>::New();
  lons->SetName("lon");

  vtkSmartPointer<vtkDoubleArray> times = vtkSmartPointer<vtkDoubleArray>::New();
  times->SetName("time");

  vtkSmartPointer<vtkDoubleArray> gpsTime = vtkSmartPointer<vtkDoubleArray>::New();
  gpsTime->SetName("gpstime");

  typedef std::map< std::string, vtkSmartPointer<vtkDoubleArray> > VecMap;
  VecMap dataVectors;
  dataVectors.insert(std::make_pair("gyro1", vtkSmartPointer<vtkDoubleArray>::New()));
  dataVectors.insert(std::make_pair("gyro2", vtkSmartPointer<vtkDoubleArray>::New()));
  dataVectors.insert(std::make_pair("gyro3", vtkSmartPointer<vtkDoubleArray>::New()));
  dataVectors.insert(std::make_pair("temp1", vtkSmartPointer<vtkDoubleArray>::New()));
  dataVectors.insert(std::make_pair("temp2", vtkSmartPointer<vtkDoubleArray>::New()));
  dataVectors.insert(std::make_pair("temp3", vtkSmartPointer<vtkDoubleArray>::New()));
  dataVectors.insert(std::make_pair("accel1x", vtkSmartPointer<vtkDoubleArray>::New()));
  dataVectors.insert(std::make_pair("accel1y", vtkSmartPointer<vtkDoubleArray>::New()));
  dataVectors.insert(std::make_pair("accel2x", vtkSmartPointer<vtkDoubleArray>::New()));
  dataVectors.insert(std::make_pair("accel2y", vtkSmartPointer<vtkDoubleArray>::New()));
  dataVectors.insert(std::make_pair("accel3x", vtkSmartPointer<vtkDoubleArray>::New()));
  dataVectors.insert(std::make_pair("accel3y", vtkSmartPointer<vtkDoubleArray>::New()));
  dataVectors.insert(std::make_pair("heading", vtkSmartPointer<vtkDoubleArray>::New()));
  for(VecMap::iterator it = dataVectors.begin(); it != dataVectors.end(); ++it)
    {
    it->second->SetName(it->first.c_str());
    }

  points->Allocate(5000, 5000);
  cells->Allocate(5000, 5000);
  lats->Allocate(5000, 5000);
  lons->Allocate(5000, 5000);
  gpsTime->Allocate(5000, 5000);

  const unsigned char* data;
  unsigned int dataLength;
  double timeSinceStart;

  PROJ *pj_utm = NULL;
  // PROJ *pj_latlong;
  // const char *latlonargs[4] = {"+proj=longlat", "+ellps=WGS84",
  //                              "+datum=WGS84",  "+no_defs" };

  // // Modern compilers and old proj4 api are at war here
  // pj_latlong = proj_init(4, const_cast<char**>(latlonargs));
  // assert(pj_latlong);

  this->Open();
  vtkIdType pointcount = 0;

  while(this->Internal->Reader->NextPacket(data, dataLength, timeSinceStart))
    {
    PositionPacket position;
    if(this->Internal->ProcessHDLPacket(data, dataLength, position))
      {
      std::vector<std::string> words = this->Internal->ParseSentance(position.sentance);

      double gpsUpdateTime;
      double latDegGPRMC;
      double lonDegGPRMC;
      double heading;

      if(words.size() != 13 &&
         words.size() != 14)
        {
        gpsUpdateTime = position.gpsTimestamp;
        lonDegGPRMC = 0.0;
        latDegGPRMC = 0.0;
        heading = 0.0;
        }
      else
        {
        gpsUpdateTime = atof(words[1].c_str());
        latDegGPRMC = atof(words[3].c_str());
        lonDegGPRMC = atof(words[5].c_str());
        heading = atof(words[8].c_str());
        }

      double latDegDec = floor(latDegGPRMC / 100);
      double latDegMin = 100 * ((latDegGPRMC / 100) - latDegDec);
      double lat = latDegDec + latDegMin /  60.0;

      if(words.size() > 5 && words[4][0] == 'S')
        {
        lat = -lat;
        }

      double lonDegDec = floor(lonDegGPRMC / 100);
      double lonDegMin = 100 * ((lonDegGPRMC / 100) - lonDegDec);
      double lon = lonDegDec + lonDegMin /  60.0;

      if(words.size() > 7 && words[6][0] == 'W')
        {
        lon = -lon;
        }

      if(this->Internal->UTMZone < 0)
        {
        assert(!pj_utm);
        this->Internal->UTMZone = LatLongToZone(lat, lon);
        std::vector<const char*> utmparams;
        utmparams.push_back("+proj=utm");
        std::stringstream zone;
        zone << "+zone=" << this->Internal->UTMZone;
        this->Internal->UTMString = zone.str();
        // WARNING: Dont let the string stream pass out of scope until
        // we finish initialization
        utmparams.push_back(this->Internal->UTMString.c_str());
        if(lat < 0)
          {
          utmparams.push_back("+south");
          }

        utmparams.push_back("+ellps=WGS84");
        utmparams.push_back("+units=m");
        utmparams.push_back("+no_defs");
        pj_utm = proj_init(utmparams.size(), const_cast<char**>(&(utmparams[0])));
        }
      else
        {
        assert(pj_utm);
        }

      // Need to convert decimal to minutes

      // PROJ_XY xy;
      // xy.x = lon * DEG_TO_RAD;
      // xy.y = lat * DEG_TO_RAD;

      PROJ_LP lp;
      lp.phi = lat * DEG_TO_RAD;
      lp.lam = lon * DEG_TO_RAD;

      PROJ_XY xy;
      xy = proj_fwd( lp, pj_utm);

      double x = xy.x;
      double y = xy.y;
      double z = 0;

      if(pointcount == 0)
        {
        this->Internal->Offset[0] = x;
        this->Internal->Offset[1] = y;
        }

      x -= this->Internal->Offset[0];
      y -= this->Internal->Offset[1];

      points->InsertNextPoint(x, y, z);
      lats->InsertNextValue(lat);
      lons->InsertNextValue(lon);
      gpsTime->InsertNextValue(gpsUpdateTime);
      polyIds->InsertNextId(pointcount);

      times->InsertNextValue(position.gpsTimestamp);

      dataVectors["gyro1"]->InsertNextValue(position.gyro[0] * GYRO_SCALE);
      dataVectors["gyro2"]->InsertNextValue(position.gyro[1] * GYRO_SCALE);
      dataVectors["gyro3"]->InsertNextValue(position.gyro[2] * GYRO_SCALE);
      dataVectors["temp1"]->InsertNextValue(position.temp[0] * TEMP_SCALE + TEMP_OFFSET);
      dataVectors["temp2"]->InsertNextValue(position.temp[1] * TEMP_SCALE + TEMP_OFFSET);
      dataVectors["temp3"]->InsertNextValue(position.temp[2] * TEMP_SCALE + TEMP_OFFSET);
      dataVectors["accel1x"]->InsertNextValue(position.accelx[0] * ACCEL_SCALE);
      dataVectors["accel2x"]->InsertNextValue(position.accelx[1] * ACCEL_SCALE);
      dataVectors["accel3x"]->InsertNextValue(position.accelx[2] * ACCEL_SCALE);
      dataVectors["accel1y"]->InsertNextValue(position.accely[0] * ACCEL_SCALE);
      dataVectors["accel2y"]->InsertNextValue(position.accely[1] * ACCEL_SCALE);
      dataVectors["accel3y"]->InsertNextValue(position.accely[2] * ACCEL_SCALE);
      dataVectors["heading"]->InsertNextValue(heading);

      pointcount++;
      }
    }
  this->Close();

  cells->InsertNextCell(polyLine);

  // Optionally interpolate the GPS values... note that we assume that the
  // first GPS point is not 0,0 if we have valid GPS data; otherwise we assume
  // that the GPS data is garbage and ignore it
  if (lats->GetNumberOfTuples() && lons->GetNumberOfTuples() &&
      (lats->GetValue(0) != 0.0 || lons->GetValue(0) != 0.0))
    {
    this->Internal->InterpolateGPS(points, gpsTime, times,
                                   dataVectors["heading"]);
    }

  output->SetPoints(points);
  output->SetLines(cells);
  output->GetPointData()->AddArray(lats);
  output->GetPointData()->AddArray(lons);
  output->GetPointData()->AddArray(gpsTime);
  output->GetPointData()->AddArray(times);
  for(VecMap::iterator it = dataVectors.begin(); it != dataVectors.end(); ++it)
    {
    output->GetPointData()->AddArray(it->second);
    }

  if(pj_utm)
    {
    proj_free(pj_utm);
    }

  return 1;
}

//-----------------------------------------------------------------------------
int vtkVelodyneHDLPositionReader::RequestInformation(vtkInformation *request,
                                     vtkInformationVector **inputVector,
                                     vtkInformationVector *outputVector)
{
  return this->Superclass::RequestInformation(request, inputVector, outputVector);
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLPositionReader::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "FileName: " << this->FileName << endl;
}

//-----------------------------------------------------------------------------
int vtkVelodyneHDLPositionReader::CanReadFile(const char *fname)
{
  return 1;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLPositionReader::Open()
{
  this->Close();
  this->Internal->Reader = new vtkPacketFileReader;
  if (!this->Internal->Reader->Open(this->FileName))
    {
    vtkErrorMacro("Failed to open packet file: " << this->FileName << endl << this->Internal->Reader->GetLastError());
    this->Close();
    }
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLPositionReader::Close()
{
  delete this->Internal->Reader;
  this->Internal->Reader = 0;
}
