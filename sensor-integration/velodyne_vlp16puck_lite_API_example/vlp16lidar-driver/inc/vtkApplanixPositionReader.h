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
  Module:    vtkApplanixPositionReader.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkApplanixPositionReader - class for reading Applanix data
// .Section Description
//

#ifndef _vtkApplanixPositionReader_h
#define _vtkApplanixPositionReader_h

#include <vtkPolyDataAlgorithm.h>

class vtkVelodyneTransformInterpolator;

class VTK_EXPORT vtkApplanixPositionReader : public vtkPolyDataAlgorithm
{
public:
  static vtkApplanixPositionReader* New();
  vtkTypeMacro(vtkApplanixPositionReader, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  //Description:
  vtkSetStringMacro(FileName);
  vtkGetStringMacro(FileName);

  // Description:
  // Set/Get the "base" yaw orientation of the GPS sensor. The value is
  // subtracted from the reported yaw to get actual yaw.
  vtkSetMacro(BaseYaw, double);
  vtkGetMacro(BaseYaw, double);

  // Description:
  // Set/Get the "base" roll orientation of the GPS sensor. The value is
  // subtracted from the reported roll to get actual roll.
  vtkSetMacro(BaseRoll, double);
  vtkGetMacro(BaseRoll, double);

  // Description:
  // Set/Get the "base" pitch orientation of the GPS sensor. The value is
  // subtracted from the reported pitch to get actual pitch.
  vtkSetMacro(BasePitch, double);
  vtkGetMacro(BasePitch, double);

  // Description:
  // Set/Get the time offset adjustment to apply to the time values. This is
  // meant to correct the offset between GPS time (as provided by the Applanix
  // sensor) and UTC time (as used by the Velodyne sensor).
  vtkSetMacro(TimeOffset, double);
  vtkGetMacro(TimeOffset, double);

  //Description:
  int CanReadFile(const char* fname) { return 1; }

  //Description:
  vtkVelodyneTransformInterpolator* GetInterpolator() const;

protected:
  vtkApplanixPositionReader();
  virtual ~vtkApplanixPositionReader();

  virtual int RequestData(vtkInformation*,
                          vtkInformationVector**,
                          vtkInformationVector*);

  char* FileName;

  double BaseYaw;
  double BaseRoll;
  double BasePitch;

  double TimeOffset;

  class vtkInternal;
  vtkInternal* Internal;

private:
  vtkApplanixPositionReader(const vtkApplanixPositionReader&);
  void operator=(const vtkApplanixPositionReader&);

};
#endif
