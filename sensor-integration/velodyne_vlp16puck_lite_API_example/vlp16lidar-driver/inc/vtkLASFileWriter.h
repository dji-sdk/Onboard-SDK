// Copyright 2014 Velodyne Acoustics, Inc.
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
  Module:    vtkLASFileWriter.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkLASFileWriter -
// .SECTION Description
//
/*========================================================================*
Modifications copyright (C) 2016  DJI.  All rights reserved.
==========================================================================*/

#ifndef __vtkLASFileWriter_h
#define __vtkLASFileWriter_h

#include <vtkSystemIncludes.h>

class vtkPolyData;

class VTK_EXPORT vtkLASFileWriter
{
public:
  vtkLASFileWriter();
  ~vtkLASFileWriter();

  void vtkLASFileName(const char* filename);
  void vtkLASFileClose(void);
  void SetTimeRange(double min, double max);
  void SetOrigin(int gcs, double easting, double northing, double height);
  void SetGeoConversion(int in, int out);
  void SetPrecision(double neTol, double hTol = 1e-3);

  void UpdateMetaData(vtkPolyData* data);
  void FlushMetaData();

  void WriteFrame(vtkPolyData* data);

protected:
  class vtkInternal;

  vtkInternal* Internal;
};

#endif
