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
  Module:    vtkVelodyneOffsetFilter.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkWrappedTupleInterpolator.h"

#include <vtkObjectFactory.h>

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkWrappedTupleInterpolator);

//----------------------------------------------------------------------------
vtkWrappedTupleInterpolator::vtkWrappedTupleInterpolator()
{
}

//----------------------------------------------------------------------------
vtkWrappedTupleInterpolator::~vtkWrappedTupleInterpolator()
{
}

//----------------------------------------------------------------------------
void vtkWrappedTupleInterpolator::InterpolateTuple1(double t, double tuple[1])
{
  this->InterpolateTuple(t, tuple);
}

//----------------------------------------------------------------------------
void vtkWrappedTupleInterpolator::InterpolateTuple2(double t, double tuple[2])
{
  this->InterpolateTuple(t, tuple);
}

//----------------------------------------------------------------------------
void vtkWrappedTupleInterpolator::InterpolateTuple3(double t, double tuple[3])
{
  this->InterpolateTuple(t, tuple);
}

//----------------------------------------------------------------------------
void vtkWrappedTupleInterpolator::InterpolateTuple4(double t, double tuple[4])
{
  this->InterpolateTuple(t, tuple);
}

//----------------------------------------------------------------------------
void vtkWrappedTupleInterpolator::InterpolateTuple5(double t, double tuple[5])
{
  this->InterpolateTuple(t, tuple);
}

//----------------------------------------------------------------------------
void vtkWrappedTupleInterpolator::InterpolateTuple9(double t, double tuple[9])
{
  this->InterpolateTuple(t, tuple);
}

//----------------------------------------------------------------------------
void vtkWrappedTupleInterpolator::InterpolateTuple16(double t, double tuple[16])
{
  this->InterpolateTuple(t, tuple);
}

//----------------------------------------------------------------------------
void vtkWrappedTupleInterpolator::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}
