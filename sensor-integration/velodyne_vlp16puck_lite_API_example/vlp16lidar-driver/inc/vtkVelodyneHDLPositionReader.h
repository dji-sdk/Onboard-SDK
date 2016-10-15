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
// .NAME vtkVelodyneHDLPositionReader - class for reading Velodyne HDL data
// .Section Description
//

#ifndef _vtkVelodyneHDLPositionReader_h
#define _vtkVelodyneHDLPositionReader_h

#include <vtkPolyDataAlgorithm.h>
#include <vtkSmartPointer.h>
#include <string>

class vtkVelodyneTransformInterpolator;

class VTK_EXPORT vtkVelodyneHDLPositionReader : public vtkPolyDataAlgorithm
{
public:
  static vtkVelodyneHDLPositionReader *New();
  vtkTypeMacro(vtkVelodyneHDLPositionReader, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  //Description:
  //
  const std::string& GetFileName();
  void SetFileName(const std::string& filename);

  //Description:
  //
  int CanReadFile(const char* fname);

  vtkVelodyneTransformInterpolator* GetInterpolator();

protected:

  vtkVelodyneHDLPositionReader();
  virtual ~vtkVelodyneHDLPositionReader();

  virtual int RequestInformation(vtkInformation *,
                         vtkInformationVector **,
                         vtkInformationVector *);

  virtual int RequestData(vtkInformation *,
                  vtkInformationVector **,
                  vtkInformationVector *);

  void Open();
  void Close();

  std::string FileName;

  class vtkInternal;
  vtkInternal* Internal;

private:

  vtkVelodyneHDLPositionReader(const vtkVelodyneHDLPositionReader&);
  void operator = (const vtkVelodyneHDLPositionReader&);

};
#endif
