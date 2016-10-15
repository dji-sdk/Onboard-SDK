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
// .NAME vtkVelodyneHDLReader - class for reading Velodyne HDL data
// .Section Description
//
/*========================================================================*
Modifications copyright (C) 2016  DJI.  All rights reserved.
==========================================================================*/

#ifndef _vtkVelodyneHDLReader_h
#define _vtkVelodyneHDLReader_h

#include <vtkPolyDataAlgorithm.h>
#include <vtkSmartPointer.h>
#include <string>

class vtkTransform;
class vtkVelodyneTransformInterpolator;

class VTK_EXPORT vtkVelodyneHDLReader : public vtkPolyDataAlgorithm
{
public:
  enum DualFlag {
    DUAL_DISTANCE_NEAR = 0x1,   // point with lesser distance
    DUAL_DISTANCE_FAR = 0x2,    // point with greater distance
    DUAL_INTENSITY_HIGH = 0x4,  // point with lesser intensity
    DUAL_INTENSITY_LOW = 0x8,   // point with greater intensity
    DUAL_DOUBLED = 0xf,         // point is single return
    DUAL_DISTANCE_MASK = 0x3,
    DUAL_INTENSITY_MASK = 0xc,
  };

public:
  static vtkVelodyneHDLReader *New();

  vtkTypeMacro(vtkVelodyneHDLReader, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  //Description:
  //
  const std::string& GetFileName();
  void SetFileName(const std::string& filename);

  //Description:
  //
  const std::string& GetCorrectionsFile();
  void SetCorrectionsFile(const std::string& correctionsFile);

  //Description:
  //
  int CanReadFile(const char* fname);

  // Property functions

  // Description:
  // Number of frames behind current frame to read.  Zero indicates only
  // show the current frame.  Negative numbers are invalid.
  void SetNumberOfTrailingFrames(int numberTrailing);

  // Description:
  // TODO: This is not friendly but I dont have a better way to pass 64 values to a filter in
  // paraview
  void SetLaserSelection(int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int,
                    int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int,
                    int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int,
                    int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int);
  void SetLaserSelection(int LaserSelection[64]);

  void GetLaserSelection(int LaserSelection[64]);

  void GetVerticalCorrections(double LaserAngles[64]);

  unsigned int GetDualReturnFilter() const;
  void SetDualReturnFilter(unsigned int);

  // A trick to workaround failure to wrap LaserSelection
  void SetDummyProperty(int);

  void SetPointsSkip(int);

  void SetCropReturns(int);
  void SetCropInside(int);
  void SetCropRegion(double[6]);
  void SetCropRegion(double, double, double, double, double, double);

  int GetNumberOfChannels();

  // I/O and processing functions
  void Open();
  void Close();
  int ReadFrameInformation();
  int GetNumberOfFrames();
  vtkSmartPointer<vtkPolyData> GetFrame(int frameNumber);
  vtkSmartPointer<vtkPolyData> GetFrameRange(int frameNumber, int numberOfFrames);

  void DumpFrames(int startFrame, int endFrame, const std::string& filename);

  void ProcessHDLPacket(unsigned char *data, unsigned int bytesReceived);
  std::vector<vtkSmartPointer<vtkPolyData> >& GetDatasets();

  vtkSmartPointer<vtkPolyData> GetVtkPoints();

  // Transform related functions

  vtkVelodyneTransformInterpolator* GetInterpolator() const;
  void SetInterpolator(vtkVelodyneTransformInterpolator* interpolator);

  void SetSensorTransform(vtkTransform*);

  int GetApplyTransform();
  void SetApplyTransform(int apply);

protected:
  vtkVelodyneHDLReader();
  ~vtkVelodyneHDLReader();

  int RequestInformation(vtkInformation *,
                         vtkInformationVector **,
                         vtkInformationVector *);

  int RequestData(vtkInformation *,
                  vtkInformationVector **,
                  vtkInformationVector *);


  void UnloadData();
  void SetTimestepInformation(vtkInformation *info);

  std::string CorrectionsFile;
  std::string FileName;

  class vtkInternal;
  vtkInternal* Internal;

private:

  vtkVelodyneHDLReader(const vtkVelodyneHDLReader&);
  void operator = (const vtkVelodyneHDLReader&);


};
#endif
