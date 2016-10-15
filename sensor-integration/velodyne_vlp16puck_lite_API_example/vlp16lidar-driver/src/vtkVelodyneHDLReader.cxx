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
  Module:    vtkVelodyneHDLReader.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/*========================================================================*
Modifications copyright (C) 2016  DJI.  All rights reserved.
==========================================================================*/

#include "vtkVelodyneHDLReader.h"

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
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkStreamingDemandDrivenPipeline.h>
#include <vtkUnsignedIntArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkUnsignedShortArray.h>

#include <vtkTransform.h>

#include <sstream>
#include <algorithm>
#include <cmath>

#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/filesystem.hpp>

#ifdef _MSC_VER
# include <boost/cstdint.hpp>
typedef boost::uint8_t uint8_t;
#else
# include <stdint.h>
#endif


namespace
{

#define HDL_Grabber_toRadians(x) ((x) * vtkMath::Pi() / 180.0)

const int HDL_NUM_ROT_ANGLES = 36001;
const int HDL_LASER_PER_FIRING = 32;
const int HDL_MAX_NUM_LASERS = 64;
const int HDL_FIRING_PER_PKT = 12;

enum HDLBlock
{
  BLOCK_0_TO_31 = 0xeeff,
  BLOCK_32_TO_63 = 0xddff
};

#pragma pack(push, 1)
typedef struct HDLLaserReturn
{
  unsigned short distance;
  unsigned char intensity;
} HDLLaserReturn;

struct HDLFiringData
{
  unsigned short blockIdentifier;
  unsigned short rotationalPosition;
  HDLLaserReturn laserReturns[HDL_LASER_PER_FIRING];
};

struct HDLDataPacket
{
  HDLFiringData firingData[HDL_FIRING_PER_PKT];
  unsigned int gpsTimestamp;
  unsigned char blank1;
  unsigned char blank2;
};

struct HDLLaserCorrection
{
  double azimuthCorrection;
  double verticalCorrection;
  double distanceCorrection;
  double verticalOffsetCorrection;
  double horizontalOffsetCorrection;
  double sinVertCorrection;
  double cosVertCorrection;
  double sinVertOffsetCorrection;
  double cosVertOffsetCorrection;
};

struct HDLRGB
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
};
#pragma pack(pop)

//-----------------------------------------------------------------------------
int MapFlags(unsigned int flags, unsigned int low, unsigned int high)
{
  return (flags & low ? -1 : flags & high ? 1 : 0);
}

//-----------------------------------------------------------------------------
int MapDistanceFlag(unsigned int flags)
{
  return MapFlags(flags & vtkVelodyneHDLReader::DUAL_DISTANCE_MASK,
                  vtkVelodyneHDLReader::DUAL_DISTANCE_NEAR,
                  vtkVelodyneHDLReader::DUAL_DISTANCE_FAR);
}

//-----------------------------------------------------------------------------
int MapIntensityFlag(unsigned int flags)
{
  return MapFlags(flags & vtkVelodyneHDLReader::DUAL_INTENSITY_MASK,
                  vtkVelodyneHDLReader::DUAL_INTENSITY_LOW,
                  vtkVelodyneHDLReader::DUAL_INTENSITY_HIGH);
}

//-----------------------------------------------------------------------------
  double HDL32AdjustTimeStamp(int firingblock,
                              int dsr)
{
  return (firingblock * 46.08) + (dsr * 1.152);
}

//-----------------------------------------------------------------------------
double VLP16AdjustTimeStamp(int firingblock,
                            int dsr,
                            int firingwithinblock)
{
  return (firingblock * 110.592) + (dsr * 2.304) + (firingwithinblock * 55.296);
}

}

//-----------------------------------------------------------------------------
class vtkVelodyneHDLReader::vtkInternal
{
public:

  vtkInternal()
  {
    this->Skip = 0;
    this->LastAzimuth = -1;
    this->LastTimestamp = std::numeric_limits<unsigned int>::max();
    this->TimeAdjust = std::numeric_limits<double>::quiet_NaN();
    this->Reader = 0;
    this->SplitCounter = 0;
    this->NumberOfTrailingFrames = 0;
    this->ApplyTransform = 0;
    this->PointsSkip = 0;
    this->CropReturns = false;
    this->CropInside = false;
    this->CropRegion[0] = this->CropRegion[1] = 0.0;
    this->CropRegion[2] = this->CropRegion[3] = 0.0;
    this->CropRegion[4] = this->CropRegion[5] = 0.0;
    this->CorrectionsInitialized = false;

    std::fill(this->LastPointId, this->LastPointId + HDL_MAX_NUM_LASERS, -1);

    this->LaserSelection.resize(64, true);
    this->DualReturnFilter = 0;
    this->IsDualReturnData = false;
    this->IsHDL64Data = false;

    this->Init();
  }

  ~vtkInternal()
  {
  }

  std::vector<vtkSmartPointer<vtkPolyData> > Datasets;
  vtkSmartPointer<vtkPolyData> CurrentDataset;

  vtkNew<vtkTransform> SensorTransform;
  vtkSmartPointer<vtkVelodyneTransformInterpolator> Interp;

  vtkSmartPointer<vtkPoints> Points;
  vtkSmartPointer<vtkPoints> Points1P;

  vtkSmartPointer<vtkUnsignedCharArray> Intensity;
  vtkSmartPointer<vtkUnsignedCharArray> LaserId;
  vtkSmartPointer<vtkUnsignedShortArray> Azimuth;
  vtkSmartPointer<vtkDoubleArray> Distance;
  vtkSmartPointer<vtkDoubleArray> Timestamp;
  vtkSmartPointer<vtkUnsignedIntArray> RawTime;
  vtkSmartPointer<vtkIntArray> IntensityFlag;
  vtkSmartPointer<vtkIntArray> DistanceFlag;
  vtkSmartPointer<vtkUnsignedIntArray> Flags;

  bool IsDualReturnData;
  bool IsHDL64Data;

  int LastAzimuth;
  unsigned int LastTimestamp;
  double TimeAdjust;
  vtkIdType LastPointId[HDL_MAX_NUM_LASERS];
  vtkIdType FirstPointIdThisReturn;

  std::vector<fpos_t> FilePositions;
  std::vector<int> Skips;
  int Skip;
  vtkPacketFileReader* Reader;

  int SplitCounter;

  // Parameters ready by calibration
  std::vector<double> cos_lookup_table_;
  std::vector<double> sin_lookup_table_;
  HDLLaserCorrection laser_corrections_[HDL_MAX_NUM_LASERS];
  int CalibrationReportedNumLasers;
  bool CorrectionsInitialized;

  // User configurable parameters
  int NumberOfTrailingFrames;
  int ApplyTransform;
  int PointsSkip;

  bool CropReturns;
  bool CropInside;
  double CropRegion[6];

  std::vector<bool> LaserSelection;
  unsigned int DualReturnFilter;

  void SplitFrame(bool force=false);
  vtkSmartPointer<vtkPolyData> CreateData(vtkIdType numberOfPoints);
  vtkSmartPointer<vtkCellArray> NewVertexCells(vtkIdType numberOfVerts);

  void Init();
  void InitTables();
  void LoadCorrectionsFile(const std::string& filename);

  void ProcessHDLPacket(unsigned char *data, std::size_t bytesReceived);

  double ComputeTimestamp(unsigned int tohTime);
  void ComputeOrientation(double timestamp, vtkTransform* geotransform);

  // Process the laser return from the firing data
  // firingData - one of HDL_FIRING_PER_PKT from the packet
  // hdl64offset - either 0 or 32 to support 64-laser systems
  // firingBlock - block of packet for firing [0-11]
  // azimuthDiff - average azimuth change between firings
  // timestamp - the timestamp of the packet
  // geotransform - georeferencing transform
  void ProcessFiring(HDLFiringData* firingData,
                     int hdl65offset,
                     int firingBlock,
                     int azimuthDiff,
                     double timestamp,
                     unsigned int rawtime,
                     vtkTransform* geotransform);

  void PushFiringData(const unsigned char laserId,
                      const unsigned char rawLaserId,
                      unsigned short azimuth,
                      const double timestamp,
                      const unsigned int rawtime,
                      const HDLLaserReturn* laserReturn,
                      const HDLLaserCorrection* correction,
                      vtkTransform* geotransform,
                      bool dualReturn);

};

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkVelodyneHDLReader);

//-----------------------------------------------------------------------------
vtkVelodyneHDLReader::vtkVelodyneHDLReader()
{
  this->Internal = new vtkInternal;
  this->UnloadData();
  this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(1);


}

//-----------------------------------------------------------------------------
vtkVelodyneHDLReader::~vtkVelodyneHDLReader()
{
  delete this->Internal;
}

//-----------------------------------------------------------------------------
const std::string& vtkVelodyneHDLReader::GetFileName()
{
  return this->FileName;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetApplyTransform(int apply)
{
  if(apply != this->Internal->ApplyTransform)
    {
    this->Modified();
    }
  this->Internal->ApplyTransform = apply;
}

//-----------------------------------------------------------------------------
int vtkVelodyneHDLReader::GetApplyTransform()
{
  return this->Internal->ApplyTransform;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetSensorTransform(vtkTransform* transform)
{
  if (transform)
    {
    this->Internal->SensorTransform->SetMatrix(transform->GetMatrix());
    }
  else
    {
    this->Internal->SensorTransform->Identity();
    }
  this->Modified();
}

//-----------------------------------------------------------------------------
vtkVelodyneTransformInterpolator* vtkVelodyneHDLReader::GetInterpolator() const
{
  return this->Internal->Interp;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetInterpolator(
  vtkVelodyneTransformInterpolator* interpolator)
{
  this->Internal->Interp = interpolator;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetFileName(const std::string& filename)
{
  if (filename == this->FileName)
    {
    return;
    }

  this->FileName = filename;
  this->Internal->FilePositions.clear();
  this->Internal->Skips.clear();
  this->UnloadData();
  this->Modified();
}

//-----------------------------------------------------------------------------
const std::string& vtkVelodyneHDLReader::GetCorrectionsFile()
{
  return this->CorrectionsFile;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetLaserSelection(int x00, int x01, int x02, int x03, int x04, int x05, int x06, int x07,
                                        int x08, int x09, int x10, int x11, int x12, int x13, int x14, int x15,
                                        int x16, int x17, int x18, int x19, int x20, int x21, int x22, int x23,
                                        int x24, int x25, int x26, int x27, int x28, int x29, int x30, int x31,
                                        int x32, int x33, int x34, int x35, int x36, int x37, int x38, int x39,
                                        int x40, int x41, int x42, int x43, int x44, int x45, int x46, int x47,
                                        int x48, int x49, int x50, int x51, int x52, int x53, int x54, int x55,
                                        int x56, int x57, int x58, int x59, int x60, int x61, int x62, int x63)
{
  int mask[64] = {x00, x01, x02, x03, x04, x05, x06, x07,
                  x08, x09, x10, x11, x12, x13, x14, x15,
                  x16, x17, x18, x19, x20, x21, x22, x23,
                  x24, x25, x26, x27, x28, x29, x30, x31,
                  x32, x33, x34, x35, x36, x37, x38, x39,
                  x40, x41, x42, x43, x44, x45, x46, x47,
                  x48, x49, x50, x51, x52, x53, x54, x55,
                  x56, x57, x58, x59, x60, x61, x62, x63};
  this->SetLaserSelection(mask);
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetLaserSelection(int LaserSelection[64])
{
  for(int i = 0; i < 64; ++i)
    {
    this->Internal->LaserSelection[i] = LaserSelection[i];
    }
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::GetLaserSelection(int LaserSelection[64])
{
  for(int i = 0; i < 64; ++i)
    {
    LaserSelection[i] = this->Internal->LaserSelection[i];
    }
}

//-----------------------------------------------------------------------------
unsigned int vtkVelodyneHDLReader::GetDualReturnFilter() const
{
  return this->Internal->DualReturnFilter;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetDualReturnFilter(unsigned int filter)
{
  if (this->Internal->DualReturnFilter != filter)
    {
    this->Internal->DualReturnFilter = filter;
    this->Modified();
    }
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::GetVerticalCorrections(double VerticalCorrections[64])
{
  for(int i = 0; i < 64; ++i)
    {
    VerticalCorrections[i] = this->Internal->laser_corrections_[i].verticalCorrection;
    }
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetDummyProperty(int vtkNotUsed(dummy))
{
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetPointsSkip(int pr)
{
  this->Internal->PointsSkip = pr;
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetNumberOfTrailingFrames(int numTrailing)
{
  assert(numTrailing >= 0);
  this->Internal->NumberOfTrailingFrames = numTrailing;
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetCropReturns(int crop)
{
  if (!this->Internal->CropReturns == !!crop)
    {
    this->Internal->CropReturns = !!crop;
    this->Modified();
    }
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetCropInside(int crop)
{
  if (!this->Internal->CropInside == !!crop)
    {
    this->Internal->CropInside = !!crop;
    this->Modified();
    }
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetCropRegion(double region[6])
{
  std::copy(region, region + 6, this->Internal->CropRegion);
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetCropRegion(
  double xl, double xu, double yl, double yu, double zl, double zu)
{
  this->Internal->CropRegion[0] = xl;
  this->Internal->CropRegion[1] = xu;
  this->Internal->CropRegion[2] = yl;
  this->Internal->CropRegion[3] = yu;
  this->Internal->CropRegion[4] = zl;
  this->Internal->CropRegion[5] = zu;
  this->Modified();
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetCorrectionsFile(const std::string& correctionsFile)
{
/*
  if (correctionsFile == this->CorrectionsFile)
    {
    return;
    }

  if (!boost::filesystem::exists(correctionsFile) ||
      boost::filesystem::is_directory(correctionsFile))
    {
    vtkErrorMacro("Invalid sensor configuration file" << correctionsFile);
    return;
    }
 */
  this->Internal->LoadCorrectionsFile(correctionsFile);

  this->CorrectionsFile = correctionsFile;
  this->UnloadData();
  this->Modified();


}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::UnloadData()
{
  std::fill(this->Internal->LastPointId, this->Internal->LastPointId + HDL_MAX_NUM_LASERS, -1);
  this->Internal->LastAzimuth = -1;
  this->Internal->LastTimestamp = std::numeric_limits<unsigned int>::max();
  this->Internal->TimeAdjust = std::numeric_limits<double>::quiet_NaN();

  this->Internal->IsDualReturnData = false;
  this->Internal->IsHDL64Data = false;
  this->Internal->Datasets.clear();
  this->Internal->CurrentDataset = this->Internal->CreateData(0);
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::SetTimestepInformation(vtkInformation *info)
{
  const size_t numberOfTimesteps = this->Internal->FilePositions.size();
  std::vector<double> timesteps;
  for (size_t i = 0; i < numberOfTimesteps; ++i)
    {
    timesteps.push_back(i);
    }

  if (numberOfTimesteps)
    {
    double timeRange[2] = {timesteps.front(), timesteps.back()};
    info->Set(vtkStreamingDemandDrivenPipeline::TIME_STEPS(), &timesteps.front(), timesteps.size());
    info->Set(vtkStreamingDemandDrivenPipeline::TIME_RANGE(), timeRange, 2);
    }
  else
    {
    info->Remove(vtkStreamingDemandDrivenPipeline::TIME_STEPS());
    info->Remove(vtkStreamingDemandDrivenPipeline::TIME_RANGE());
    }
}

//-----------------------------------------------------------------------------
int vtkVelodyneHDLReader::RequestData(vtkInformation *request,
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

  if (!this->Internal->CorrectionsInitialized)
    {
    vtkErrorMacro("Corrections have not been set");
    return 0;
    }

  int timestep = 0;
  if (info->Has(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP()))
    {
    double timeRequest = info->Get(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP());
    timestep = static_cast<int>(floor(timeRequest+0.5));
    }

  if (timestep < 0 || timestep >= this->GetNumberOfFrames())
    {
    vtkErrorMacro("Cannot meet timestep request: " << timestep << ".  Have " << this->GetNumberOfFrames() << " datasets.");
    output->ShallowCopy(this->Internal->CreateData(0));
    return 0;
    }

  this->Open();

  if(this->Internal->NumberOfTrailingFrames > 0)
    {
    output->ShallowCopy(this->GetFrameRange(timestep - this->Internal->NumberOfTrailingFrames,
                                            this->Internal->NumberOfTrailingFrames));
    }
  else
    {
    output->ShallowCopy(this->GetFrame(timestep));
    }

  this->Close();
  return 1;
}

//-----------------------------------------------------------------------------
int vtkVelodyneHDLReader::RequestInformation(vtkInformation *request,
                                     vtkInformationVector **inputVector,
                                     vtkInformationVector *outputVector)
{
  if (this->FileName.length() && !this->Internal->FilePositions.size())
    {
    this->ReadFrameInformation();
    }

  vtkInformation *info = outputVector->GetInformationObject(0);
  this->SetTimestepInformation(info);
  return 1;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "FileName: " << this->FileName << endl;
  os << indent << "CorrectionsFile: " << this->CorrectionsFile << endl;
}

//-----------------------------------------------------------------------------
int vtkVelodyneHDLReader::CanReadFile(const char *fname)
{
  return 1;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::ProcessHDLPacket(unsigned char *data, unsigned int bytesReceived)
{
  this->Internal->ProcessHDLPacket(data, bytesReceived);
}

//-----------------------------------------------------------------------------
std::vector<vtkSmartPointer<vtkPolyData> >& vtkVelodyneHDLReader::GetDatasets()
{
  return this->Internal->Datasets;
}

//-----------------------------------------------------------------------------
int vtkVelodyneHDLReader::GetNumberOfFrames()
{
  return this->Internal->FilePositions.size();;
}

//-----------------------------------------------------------------------------
int vtkVelodyneHDLReader::GetNumberOfChannels()
{
  return this->Internal->CalibrationReportedNumLasers;
}

vtkSmartPointer<vtkPolyData> vtkVelodyneHDLReader::GetVtkPoints()
{
   this->Internal->CurrentDataset->Reset();
   this->Internal->CurrentDataset->SetPoints(this->Internal->Points1P);

   return this->Internal->CurrentDataset;

}


//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::Open()
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
void vtkVelodyneHDLReader::Close()
{
  delete this->Internal->Reader;
  this->Internal->Reader = 0;
}


//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::DumpFrames(int startFrame, int endFrame, const std::string& filename)
{
  if (!this->Internal->Reader)
    {
    vtkErrorMacro("DumpFrames() called but packet file reader is not open.");
    return;
    }

  vtkPacketFileWriter writer;
  if (!writer.Open(filename))
    {
    vtkErrorMacro("Failed to open packet file for writing: " << filename);
    return;
    }

  pcap_pkthdr* header = 0;
  const unsigned char* data = 0;
  unsigned int dataLength = 0;
  double timeSinceStart = 0;

  unsigned int lastAzimuth = 0;
  int currentFrame = startFrame;

  this->Internal->Reader->SetFilePosition(&this->Internal->FilePositions[startFrame]);
  int skip = this->Internal->Skips[startFrame];

  while (this->Internal->Reader->NextPacket(data, dataLength, timeSinceStart, &header) &&
         currentFrame <= endFrame)
    {
    if (dataLength == (1206 + 42) ||
        dataLength == (512 + 42))
      {
      writer.WritePacket(header, const_cast<unsigned char*>(data));
      }

    // dont check for frame counts if it was a GPS packet
    if(dataLength != (1206 + 42))
      {
      continue;
      }

    // Check if we cycled a frame and decrement
    const HDLDataPacket* dataPacket = reinterpret_cast<const HDLDataPacket *>(data + 42);

    for (int i = skip; i < HDL_FIRING_PER_PKT; ++i)
      {
      HDLFiringData firingData = dataPacket->firingData[i];

      if (firingData.rotationalPosition != 0 && firingData.rotationalPosition < lastAzimuth)
        {
        currentFrame++;
        if(currentFrame > endFrame)
          {
          break;
          }
        }
      lastAzimuth = firingData.rotationalPosition;
      }
    skip = 0;
    }

  writer.Close();
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> vtkVelodyneHDLReader::GetFrameRange(int startFrame, int numberOfFrames)
{
  this->UnloadData();
  if (!this->Internal->Reader)
    {
    vtkErrorMacro("GetFrame() called but packet file reader is not open.");
    return 0;
    }
  if (!this->Internal->CorrectionsInitialized)
    {
    vtkErrorMacro("Corrections have not been set");
    return 0;
    }

  const unsigned char* data = 0;
  unsigned int dataLength = 0;
  double timeSinceStart = 0;

  if(startFrame < 0)
    {
    numberOfFrames -= startFrame;
    startFrame = 0;
    }
  assert(numberOfFrames > 0);

  this->Internal->Reader->SetFilePosition(&this->Internal->FilePositions[startFrame]);
  this->Internal->Skip = this->Internal->Skips[startFrame];

  this->Internal->SplitCounter = numberOfFrames;

  while (this->Internal->Reader->NextPacket(data, dataLength, timeSinceStart))
    {
    this->ProcessHDLPacket(const_cast<unsigned char*>(data), dataLength);

    if (this->Internal->Datasets.size())
      {
      this->Internal->SplitCounter = 0;
      return this->Internal->Datasets.back();
      }
    }

  this->Internal->SplitFrame(true);
  this->Internal->SplitCounter = 0;
  return this->Internal->Datasets.back();
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> vtkVelodyneHDLReader::GetFrame(int frameNumber)
{
  this->UnloadData();
  if (!this->Internal->Reader)
    {
    vtkErrorMacro("GetFrame() called but packet file reader is not open.");
    return 0;
    }
  if (!this->Internal->CorrectionsInitialized)
    {
    vtkErrorMacro("Corrections have not been set");
    return 0;
    }

  assert(this->Internal->FilePositions.size() == this->Internal->Skips.size());
  if(frameNumber < 0 || frameNumber > this->Internal->FilePositions.size())
    {
    vtkErrorMacro("Invalid frame requested");
    return 0;
    }

  const unsigned char* data = 0;
  unsigned int dataLength = 0;
  double timeSinceStart = 0;


  this->Internal->Reader->SetFilePosition(&this->Internal->FilePositions[frameNumber]);
  this->Internal->Skip = this->Internal->Skips[frameNumber];

  while (this->Internal->Reader->NextPacket(data, dataLength, timeSinceStart))
    {
    this->ProcessHDLPacket(const_cast<unsigned char*>(data), dataLength);

    if (this->Internal->Datasets.size())
      {
      return this->Internal->Datasets.back();
      }
    }

  this->Internal->SplitFrame();
  return this->Internal->Datasets.back();
}

namespace
{
  template <typename T>
  vtkSmartPointer<T> CreateDataArray(const char* name, vtkIdType np, vtkPolyData* pd)
  {
    vtkSmartPointer<T> array = vtkSmartPointer<T>::New();
    array->Allocate(60000);
    array->SetName(name);
    array->SetNumberOfTuples(np);

    if (pd)
      {
      pd->GetPointData()->AddArray(array);
      }

    return array;
  }
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> vtkVelodyneHDLReader::vtkInternal::CreateData(vtkIdType numberOfPoints)
{
  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();

  // points
  vtkNew<vtkPoints> points;


  points->SetDataTypeToFloat();
  points->Allocate(60000);
  points->SetNumberOfPoints(numberOfPoints);
  points->GetData()->SetName("Points_m_XYZ");
  polyData->SetPoints(points.GetPointer());
  polyData->SetVerts(NewVertexCells(numberOfPoints));

  // intensity
  this->Points = points.GetPointer();
  this->Intensity = CreateDataArray<vtkUnsignedCharArray>("intensity", numberOfPoints, polyData);
  this->LaserId = CreateDataArray<vtkUnsignedCharArray>("laser_id", numberOfPoints, polyData);
  this->Azimuth = CreateDataArray<vtkUnsignedShortArray>("azimuth", numberOfPoints, polyData);
  this->Distance = CreateDataArray<vtkDoubleArray>("distance_m", numberOfPoints, polyData);
  this->Timestamp = CreateDataArray<vtkDoubleArray>("adjustedtime", numberOfPoints, polyData);
  this->RawTime = CreateDataArray<vtkUnsignedIntArray>("timestamp", numberOfPoints, polyData);
  this->DistanceFlag = CreateDataArray<vtkIntArray>("dual_distance", numberOfPoints, 0);
  this->IntensityFlag = CreateDataArray<vtkIntArray>("dual_intensity", numberOfPoints, 0);
  this->Flags = CreateDataArray<vtkUnsignedIntArray>("dual_flags", numberOfPoints, 0);

  if (this->IsDualReturnData)
    {
    polyData->GetPointData()->AddArray(this->DistanceFlag.GetPointer());
    polyData->GetPointData()->AddArray(this->IntensityFlag.GetPointer());
    }

  return polyData;
}

//----------------------------------------------------------------------------
vtkSmartPointer<vtkCellArray> vtkVelodyneHDLReader::vtkInternal::NewVertexCells(vtkIdType numberOfVerts)
{
  vtkNew<vtkIdTypeArray> cells;
  cells->SetNumberOfValues(numberOfVerts*2);
  vtkIdType* ids = cells->GetPointer(0);
  for (vtkIdType i = 0; i < numberOfVerts; ++i)
    {
    ids[i*2] = 1;
    ids[i*2+1] = i;
    }

  vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();
  cellArray->SetCells(numberOfVerts, cells.GetPointer());
  return cellArray;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::vtkInternal::PushFiringData(const unsigned char laserId,
                                                       const unsigned char rawLaserId,
                                                       unsigned short azimuth,
                                                       const double timestamp,
                                                       const unsigned int rawtime,
                                                       const HDLLaserReturn* laserReturn,
                                                       const HDLLaserCorrection* correction,
                                                       vtkTransform* geotransform,
                                                       const bool dualReturn)
{
  azimuth %= 36000;
  const vtkIdType thisPointId = this->Points->GetNumberOfPoints();
  const short intensity = laserReturn->intensity;

  double cosAzimuth, sinAzimuth;
  if (correction->azimuthCorrection == 0)
  {
    cosAzimuth = this->cos_lookup_table_[azimuth];
    sinAzimuth = this->sin_lookup_table_[azimuth];
  }
  else
  {
    double azimuthInRadians = HDL_Grabber_toRadians((static_cast<double> (azimuth) / 100.0) - correction->azimuthCorrection);
    cosAzimuth = std::cos (azimuthInRadians);
    sinAzimuth = std::sin (azimuthInRadians);
  }

  double distanceM = laserReturn->distance * 0.002 + correction->distanceCorrection;
  double xyDistance = distanceM * correction->cosVertCorrection;

  // Compute raw position
  double pos[3] =
    {
    xyDistance * sinAzimuth - correction->horizontalOffsetCorrection * cosAzimuth,
    xyDistance * cosAzimuth + correction->horizontalOffsetCorrection * sinAzimuth,
    distanceM * correction->sinVertCorrection + correction->verticalOffsetCorrection
    };

  // Apply sensor transform
  this->SensorTransform->InternalTransformPoint(pos, pos);

  // Test if point is cropped
  if (this->CropReturns)
    {
    bool pointOutsideOfBox = pos[0] >= this->CropRegion[0] && pos[0] <= this->CropRegion[1] &&
      pos[1] >= this->CropRegion[2] && pos[1] <= this->CropRegion[3] &&
      pos[2] >= this->CropRegion[4] && pos[2] <= this->CropRegion[5];
    if ((pointOutsideOfBox && !this->CropInside) ||
        (!pointOutsideOfBox && this->CropInside))
      {
      return;
      }
    }

  // Do not add any data before here as this might short-circuit
  if (dualReturn)
    {
    const vtkIdType dualPointId = this->LastPointId[rawLaserId];
    if (dualPointId < this->FirstPointIdThisReturn)
      {
      // No matching point from first set (skipped?)
      this->Flags->InsertNextValue(DUAL_DOUBLED);
      this->DistanceFlag->InsertNextValue(0);
      this->IntensityFlag->InsertNextValue(0);
      }
    else
      {
      const short dualIntensity = this->Intensity->GetValue(dualPointId);
      const double dualDistance = this->Distance->GetValue(dualPointId);
      unsigned int firstFlags = this->Flags->GetValue(dualPointId);
      unsigned int secondFlags = 0;


      if (dualDistance == distanceM && intensity == dualIntensity)
      {
        // ignore duplicate point and leave first with original flags
        return;
      }

      if (dualIntensity < intensity)
        {
        firstFlags &= ~DUAL_INTENSITY_HIGH;
        secondFlags |= DUAL_INTENSITY_HIGH;
        }
      else
        {
        firstFlags &= ~DUAL_INTENSITY_LOW;
        secondFlags |= DUAL_INTENSITY_LOW;
        }

      if (dualDistance < distanceM)
        {
        firstFlags &= ~DUAL_DISTANCE_FAR;
        secondFlags |= DUAL_DISTANCE_FAR;
        }
      else
        {
        firstFlags &= ~DUAL_DISTANCE_NEAR;
        secondFlags |= DUAL_DISTANCE_NEAR;
        }

      // We will output only one point so return out of this
      if (this->DualReturnFilter)
        {
        if (!(secondFlags & this->DualReturnFilter))
          {
          // second return does not match filter; skip
          this->Flags->SetValue(dualPointId, firstFlags);
          this->DistanceFlag->SetValue(dualPointId, MapDistanceFlag(firstFlags));
          this->IntensityFlag->SetValue(dualPointId, MapIntensityFlag(firstFlags));
          return;
          }
        if (!(firstFlags & this->DualReturnFilter))
          {
          // first return does not match filter; replace with second return
          this->Points->SetPoint(dualPointId, pos);
          this->Distance->SetValue(dualPointId, distanceM);
          this->Intensity->SetValue(dualPointId, laserReturn->intensity);
          this->Timestamp->SetValue(dualPointId, timestamp);
          this->RawTime->SetValue(dualPointId, rawtime);
          this->Flags->SetValue(dualPointId, secondFlags);
          this->DistanceFlag->SetValue(dualPointId, MapDistanceFlag(secondFlags));
          this->IntensityFlag->SetValue(dualPointId, MapIntensityFlag(secondFlags));
          return;
          }
        }

      this->Flags->SetValue(dualPointId, firstFlags);
      this->DistanceFlag->SetValue(dualPointId, MapDistanceFlag(firstFlags));
      this->IntensityFlag->SetValue(dualPointId, MapIntensityFlag(firstFlags));
      this->Flags->InsertNextValue(secondFlags);
      this->DistanceFlag->InsertNextValue(MapDistanceFlag(secondFlags));
      this->IntensityFlag->InsertNextValue(MapIntensityFlag(secondFlags));
      }
    }
  else
    {
    this->Flags->InsertNextValue(DUAL_DOUBLED);
    this->DistanceFlag->InsertNextValue(0);
    this->IntensityFlag->InsertNextValue(0);
    }

  // Apply geoposition transform
  geotransform->InternalTransformPoint(pos, pos);
  this->Points->InsertNextPoint(pos);
  this->Points1P->InsertNextPoint(pos);

  this->Azimuth->InsertNextValue(azimuth);
  this->Intensity->InsertNextValue(laserReturn->intensity);
  this->LaserId->InsertNextValue(laserId);
  this->Timestamp->InsertNextValue(timestamp);
  this->RawTime->InsertNextValue(rawtime);
  this->Distance->InsertNextValue(distanceM);
  this->LastPointId[rawLaserId] = thisPointId;

}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::vtkInternal::InitTables()
{
  if (cos_lookup_table_.size() == 0 || sin_lookup_table_.size() == 0)
    {
    cos_lookup_table_.resize(HDL_NUM_ROT_ANGLES);
    sin_lookup_table_.resize(HDL_NUM_ROT_ANGLES);
    for (unsigned int i = 0; i < HDL_NUM_ROT_ANGLES; i++)
      {
      double rad = HDL_Grabber_toRadians(i / 100.0);
      cos_lookup_table_[i] = std::cos(rad);
      sin_lookup_table_[i] = std::sin(rad);
      }
    }
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::vtkInternal::LoadCorrectionsFile(const std::string& correctionsFile)
{
  boost::property_tree::ptree pt;
  try
    {
    read_xml(correctionsFile, pt, boost::property_tree::xml_parser::trim_whitespace);
    }
  catch (boost::exception const&)
    {
    vtkGenericWarningMacro("LoadCorrectionsFile: error reading calibration file: " << correctionsFile);
    return;
    }

  int enabledCount = 0;
  BOOST_FOREACH (boost::property_tree::ptree::value_type &v, pt.get_child("boost_serialization.DB.enabled_"))
    {
    std::stringstream ss;
    if(v.first == "item")
      {
      ss << v.second.data();
      int test = 0;
      ss >> test;
      if(!ss.fail() && test == 1)
        {
        enabledCount++;
        }
      }
    }
  this->CalibrationReportedNumLasers = enabledCount;

  BOOST_FOREACH (boost::property_tree::ptree::value_type &v, pt.get_child("boost_serialization.DB.points_"))
    {
    if (v.first == "item")
      {
      boost::property_tree::ptree points = v.second;
      BOOST_FOREACH (boost::property_tree::ptree::value_type &px, points)
        {
        if (px.first == "px")
          {
          boost::property_tree::ptree calibrationData = px.second;
          int index = -1;
          double azimuth = 0;
          double vertCorrection = 0;
          double distCorrection = 0;
          double vertOffsetCorrection = 0;
          double horizOffsetCorrection = 0;

          BOOST_FOREACH (boost::property_tree::ptree::value_type &item, calibrationData)
            {
            if (item.first == "id_")
              index = atoi(item.second.data().c_str());
            if (item.first == "rotCorrection_")
              azimuth = atof(item.second.data().c_str());
            if (item.first == "vertCorrection_")
              vertCorrection = atof(item.second.data().c_str());
            if (item.first == "distCorrection_")
              distCorrection = atof(item.second.data().c_str());
            if (item.first == "vertOffsetCorrection_")
              vertOffsetCorrection = atof(item.second.data().c_str());
            if (item.first == "horizOffsetCorrection_")
              horizOffsetCorrection = atof(item.second.data().c_str());
            }
          if (index != -1)
            {
            laser_corrections_[index].azimuthCorrection = azimuth;
            laser_corrections_[index].verticalCorrection = vertCorrection;
            laser_corrections_[index].distanceCorrection = distCorrection / 100.0;
            laser_corrections_[index].verticalOffsetCorrection = vertOffsetCorrection / 100.0;
            laser_corrections_[index].horizontalOffsetCorrection = horizOffsetCorrection / 100.0;

            laser_corrections_[index].cosVertCorrection = std::cos (HDL_Grabber_toRadians(laser_corrections_[index].verticalCorrection));
            laser_corrections_[index].sinVertCorrection = std::sin (HDL_Grabber_toRadians(laser_corrections_[index].verticalCorrection));
            }
          }
        }
      }
    }

  for (int i = 0; i < HDL_MAX_NUM_LASERS; i++)
    {
    HDLLaserCorrection correction = laser_corrections_[i];
    laser_corrections_[i].sinVertOffsetCorrection = correction.verticalOffsetCorrection
                                       * correction.sinVertCorrection;
    laser_corrections_[i].cosVertOffsetCorrection = correction.verticalOffsetCorrection
                                       * correction.cosVertCorrection;
    }
  this->CorrectionsInitialized = true;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::vtkInternal::Init()
{
  this->InitTables();
  this->SensorTransform->Identity();

  vtkNew<vtkPoints> pointsTst;

  pointsTst->SetDataTypeToDouble();
  pointsTst->Allocate(3000);
  pointsTst->GetData()->SetName("Points_m_XYZ");

  this->Points1P = pointsTst.GetPointer();

}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::vtkInternal::SplitFrame(bool force)
{
  if(this->SplitCounter > 0 && !force)
    {
    this->SplitCounter--;
    return;
    }

  for (size_t n = 0; n < HDL_MAX_NUM_LASERS; ++n)
    {
    this->LastPointId[n] = -1;
    }

  this->CurrentDataset->SetVerts(this->NewVertexCells(this->CurrentDataset->GetNumberOfPoints()));
  this->Datasets.push_back(this->CurrentDataset);
  this->CurrentDataset = this->CreateData(0);
}

//-----------------------------------------------------------------------------
double vtkVelodyneHDLReader::vtkInternal::ComputeTimestamp(
  unsigned int tohTime)
{
  static const double hourInMilliseconds = 3600.0 * 1e6;

  if (tohTime < this->LastTimestamp)
    {
    if (!vtkMath::IsFinite(this->TimeAdjust))
      {
      // First adjustment; must compute adjustment number
      if (this->Interp && this->Interp->GetNumberOfTransforms())
        {
        const double ts = static_cast<double>(tohTime) * 1e-6;
        const double hours = (this->Interp->GetMinimumT() - ts) / 3600.0;
        this->TimeAdjust = vtkMath::Round(hours) * hourInMilliseconds;
        }
      else
        {
        // Ought to warn about this, but happens when applogic is checking that
        // we can read the file :-(
        this->TimeAdjust = 0;
        }
      }
    else
      {
      // Hour has wrapped; add an hour to the update adjustment value
      this->TimeAdjust += hourInMilliseconds;
      }
    }

  this->LastTimestamp = tohTime;
  return static_cast<double>(tohTime) + this->TimeAdjust;
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::vtkInternal::ComputeOrientation(
  double timestamp, vtkTransform* geotransform)
{
  if(this->ApplyTransform && this->Interp && this->Interp->GetNumberOfTransforms())
    {
    // NOTE: We store time in milliseconds, but the interpolator uses seconds,
    //       so we need to adjust here
    const double t = timestamp * 1e-6;
    this->Interp->InterpolateTransform(t, geotransform);
    }
  else
    {
    geotransform->Identity();
    }
}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::vtkInternal::ProcessFiring(HDLFiringData* firingData,
                                                      int hdl64offset,
                                                      int firingBlock,
                                                      int azimuthDiff,
                                                      double timestamp,
                                                      unsigned int rawtime,
                                                      vtkTransform* geotransform)
{
  const bool dual = (this->LastAzimuth == firingData->rotationalPosition) &&
    (!this->IsHDL64Data);

  if (!dual)
    {
    this->FirstPointIdThisReturn = this->Points->GetNumberOfPoints();
    }

  if (dual && !this->IsDualReturnData)
    {
    this->IsDualReturnData = true;
    this->CurrentDataset->GetPointData()->AddArray(this->DistanceFlag.GetPointer());
    this->CurrentDataset->GetPointData()->AddArray(this->IntensityFlag.GetPointer());
    }

  for (int dsr = 0; dsr < HDL_LASER_PER_FIRING; dsr++)
    {
    unsigned char rawLaserId = static_cast<unsigned char>(dsr + hdl64offset);
    unsigned char laserId = rawLaserId;
    unsigned short azimuth = firingData->rotationalPosition;

    // Detect VLP-16 data and adjust laser id if necessary
    int firingWithinBlock = 0;

    if(this->CalibrationReportedNumLasers == 16)
      {
      assert(hdl64offset == 0);
      if(laserId >= 16)
        {
        laserId -= 16;
        firingWithinBlock = 1;
        }
      }

    // Interpolate azimuth
    double timestampadjustment = 0.0;
    double blockdsr0 = 0.0;
    double nextblockdsr0 = 1.0;
    if(this->CalibrationReportedNumLasers == 32)
      {
      timestampadjustment = HDL32AdjustTimeStamp(firingBlock, dsr);
      nextblockdsr0 = HDL32AdjustTimeStamp(firingBlock+1,0);
      blockdsr0 = HDL32AdjustTimeStamp(firingBlock,0);
      }
    else if(this->CalibrationReportedNumLasers == 16)
      {
      timestampadjustment = VLP16AdjustTimeStamp(firingBlock, laserId, firingWithinBlock);
      nextblockdsr0 = VLP16AdjustTimeStamp(firingBlock+1,0,0);
      blockdsr0 = VLP16AdjustTimeStamp(firingBlock,0,0);
      }
    int azimuthadjustment = vtkMath::Round(azimuthDiff * ((timestampadjustment - blockdsr0) / (nextblockdsr0 - blockdsr0)));
    timestampadjustment = vtkMath::Round(timestampadjustment);

    if (firingData->laserReturns[dsr].distance != 0.0 && this->LaserSelection[laserId])
      {
         this->PushFiringData(laserId,
                           rawLaserId,
                           azimuth + azimuthadjustment,
                           timestamp + timestampadjustment,
                           rawtime + static_cast<unsigned int>(timestampadjustment),
                           &(firingData->laserReturns[dsr]),
                           &(laser_corrections_[dsr + hdl64offset]),
                           geotransform,
                           dual);

      }

    }

}

//-----------------------------------------------------------------------------
void vtkVelodyneHDLReader::vtkInternal::ProcessHDLPacket(unsigned char *data, std::size_t bytesReceived)
{
  if (bytesReceived != 1206)
    {
    return;
    }

  this->Points1P->Reset();

  HDLDataPacket* dataPacket = reinterpret_cast<HDLDataPacket *>(data);

  vtkNew<vtkTransform> geotransform;
  const unsigned int rawtime = dataPacket->gpsTimestamp;
  const double timestamp = this->ComputeTimestamp(dataPacket->gpsTimestamp);
  this->ComputeOrientation(timestamp, geotransform.GetPointer());

  // Update the transforms here and then call internal
  // transform
  this->SensorTransform->Update();
  geotransform->Update();

  int firingBlock = this->Skip;
  this->Skip = 0;

  std::vector<int> diffs (HDL_FIRING_PER_PKT - 1);
  for(int i = 0; i < HDL_FIRING_PER_PKT - 1; ++i)
    {
    int localDiff = (36000 + dataPacket->firingData[i+1].rotationalPosition -
                     dataPacket->firingData[i].rotationalPosition) % 36000;
    diffs[i] = localDiff;
    }
  std::nth_element(diffs.begin(),
                   diffs.begin() + HDL_FIRING_PER_PKT/2,
                   diffs.end());
  int azimuthDiff = diffs[HDL_FIRING_PER_PKT/2];
  assert(azimuthDiff >= 0);

  for (; firingBlock < HDL_FIRING_PER_PKT; ++firingBlock)
    {
    HDLFiringData* firingData = &(dataPacket->firingData[firingBlock]);
    int hdl64offset = (firingData->blockIdentifier == BLOCK_0_TO_31) ? 0 : 32;
    this->IsHDL64Data |= (hdl64offset > 0);

    if (firingData->rotationalPosition < this->LastAzimuth)
      {
      this->SplitFrame();
      }

    // Skip this firing every PointSkip
    if(this->PointsSkip == 0 || firingBlock % (this->PointsSkip + 1) == 0)
      {
      this->ProcessFiring(firingData,
                          hdl64offset,
                          firingBlock,
                          azimuthDiff,
                          timestamp,
                          rawtime,
                          geotransform.GetPointer());
      }

    this->LastAzimuth = firingData->rotationalPosition;
    }

}

//-----------------------------------------------------------------------------
int vtkVelodyneHDLReader::ReadFrameInformation()
{
  vtkPacketFileReader reader;
  if (!reader.Open(this->FileName))
    {
    vtkErrorMacro("Failed to open packet file: " << this->FileName << endl << reader.GetLastError());
    return 0;
    }

  const unsigned char* data = 0;
  unsigned int dataLength = 0;
  double timeSinceStart = 0;

  unsigned int lastAzimuth = 0;
  unsigned int lastTimestamp = 0;

  std::vector<fpos_t> filePositions;
  std::vector<int> skips;

  fpos_t lastFilePosition;
  reader.GetFilePosition(&lastFilePosition);


  filePositions.push_back(lastFilePosition);
  skips.push_back(0);

  while (reader.NextPacket(data, dataLength, timeSinceStart))
    {

    if (dataLength != 1206)
      {
      continue;
      }

    const HDLDataPacket* dataPacket = reinterpret_cast<const HDLDataPacket *>(data);

//    unsigned int timeDiff = dataPacket->gpsTimestamp - lastTimestamp;
//    if (timeDiff > 600 && lastTimestamp != 0)
//      {
//      printf("missed %d packets\n",  static_cast<int>(floor((timeDiff/553.0) + 0.5)));
//      }

    for (int i = 0; i < HDL_FIRING_PER_PKT; ++i)
      {
      HDLFiringData firingData = dataPacket->firingData[i];

      if (firingData.rotationalPosition < lastAzimuth)
        {
        filePositions.push_back(lastFilePosition);
        skips.push_back(i);
        this->UpdateProgress(0.0);
        }

      lastAzimuth = firingData.rotationalPosition;
      }

    lastTimestamp = dataPacket->gpsTimestamp;
    reader.GetFilePosition(&lastFilePosition);
    }

  this->Internal->FilePositions = filePositions;
  this->Internal->Skips = skips;
  return this->GetNumberOfFrames();
}
