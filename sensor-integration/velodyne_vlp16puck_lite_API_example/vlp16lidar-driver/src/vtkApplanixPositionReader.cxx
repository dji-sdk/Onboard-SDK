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

#include "vtkApplanixPositionReader.h"

#include "vtkVelodyneTransformInterpolator.h"

#include <vtkCellArray.h>
#include <vtkDoubleArray.h>
#include <vtkInformationVector.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkPointData.h>
#include <vtkPolyLine.h>
#include <vtkTransform.h>
// #include <vtkTransformInterpolator.h>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <map>

#define DATA_ARRAY(name) \
  vtkNew<vtkDoubleArray> name##Data; name##Data->SetName(#name)

namespace
{
typedef std::map<std::string, size_t> FieldIndexMap;
typedef std::map<size_t, vtkDoubleArray*> FieldDataMap;
}

//-----------------------------------------------------------------------------
class vtkApplanixPositionReader::vtkInternal
{
public:
  void SetMapping(const std::string& fieldName, vtkNew<vtkDoubleArray>& array);

  vtkNew<vtkVelodyneTransformInterpolator> Interpolator;

  FieldIndexMap Fields;
  FieldDataMap FieldMapping;
};

//-----------------------------------------------------------------------------
void vtkApplanixPositionReader::vtkInternal::SetMapping(
  const std::string& fieldName, vtkNew<vtkDoubleArray>& array)
{
  FieldIndexMap::iterator field = this->Fields.find(fieldName);
  if (field != this->Fields.end())
    {
    this->FieldMapping.insert(
      std::make_pair(field->second, array.GetPointer()));
    }
}

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkApplanixPositionReader);

//-----------------------------------------------------------------------------
vtkApplanixPositionReader::vtkApplanixPositionReader()
{
  this->Internal = new vtkInternal;
  this->FileName = 0;
  this->BaseYaw = 0.0;
  this->BaseRoll = 0.0;
  this->BasePitch = 0.0;
  this->TimeOffset = 16.0; // correct for at least 2012-Jul - 2015-May

  this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(1);
}

//-----------------------------------------------------------------------------
vtkApplanixPositionReader::~vtkApplanixPositionReader()
{
  delete this->FileName;
  delete this->Internal;
}

//-----------------------------------------------------------------------------
vtkVelodyneTransformInterpolator* vtkApplanixPositionReader::GetInterpolator() const
{
  return this->Internal->Interpolator.GetPointer();
}

//-----------------------------------------------------------------------------
int vtkApplanixPositionReader::RequestData(
  vtkInformation *request,
  vtkInformationVector **inputVector,
  vtkInformationVector *outputVector)
{
  vtkPolyData *output = vtkPolyData::GetData(outputVector);
  vtkInformation *info = outputVector->GetInformationObject(0);

  if (!this->FileName || !*this->FileName)
    {
    vtkErrorMacro("FileName has not been set");
    return VTK_ERROR;
    }

  vtkNew<vtkPoints> points;
  vtkNew<vtkCellArray> cells;
  vtkNew<vtkPolyLine> polyLine;
  vtkIdList* polyIds = polyLine->GetPointIds();

  // Data arrays
  DATA_ARRAY(time);
  DATA_ARRAY(distance);
  DATA_ARRAY(easting);
  DATA_ARRAY(northing);
  DATA_ARRAY(height);
  DATA_ARRAY(lat);
  DATA_ARRAY(lon);
  DATA_ARRAY(roll);
  DATA_ARRAY(pitch);
  DATA_ARRAY(heading);
  vtkNew<vtkIntArray> zoneData;
  zoneData->SetName("zone");

  // Open data file
  std::ifstream f(this->FileName);
  if (!f.good())
    {
    vtkErrorMacro("Failed to open input file \"" << this->FileName << "\"");
    return VTK_ERROR;
    }

  std::string line;
  std::string lastLine;

  // Read header
  size_t numFields = 0;
  while (std::getline(f, line))
    {
    boost::algorithm::trim(line);
    if (line.empty())
      {
      continue;
      }

    if (boost::starts_with(line, "central meridian"))
      {
      std::vector<std::string> parts;
      boost::algorithm::split(parts, line, boost::is_any_of(" "),
                              boost::algorithm::token_compress_on);

      zoneData->InsertNextValue(
        static_cast<int>(186 + boost::lexical_cast<double>(parts[3])) / 6);
      }

    if (line[0] == '(')
      {
      // Set up field index mapping
      std::vector<std::string> fields;
      boost::algorithm::split(fields, lastLine, boost::is_any_of(","),
                              boost::algorithm::token_compress_on);

      numFields = fields.size();
      for (size_t n = 0; n < numFields; ++n)
        {
        boost::algorithm::trim(fields[n]);
        this->Internal->Fields.insert(std::make_pair(fields[n], n));
        }

      // Done with header
      break;
      }

    lastLine = line;
    }

  // Set up data array mapping
  this->Internal->SetMapping("TIME", timeData);
  this->Internal->SetMapping("DISTANCE", distanceData);
  this->Internal->SetMapping("EASTING", eastingData);
  this->Internal->SetMapping("NORTHING", northingData);
  this->Internal->SetMapping("ELLIPSOID HEIGHT", heightData);
  this->Internal->SetMapping("LATITUDE", latData);
  this->Internal->SetMapping("LONGITUDE", lonData);
  this->Internal->SetMapping("ROLL", rollData);
  this->Internal->SetMapping("PITCH", pitchData);
  this->Internal->SetMapping("HEADING", headingData);

  // Read data
  vtkIdType count = 0;
  while (std::getline(f, line))
    {
    boost::algorithm::trim(line);
    if (line.empty())
      {
      continue;
      }

    // Split into fields
    std::vector<std::string> fields;
    boost::algorithm::split(fields, line, boost::is_any_of(" "),
                            boost::algorithm::token_compress_on);

    if (fields.size() < numFields)
      {
      vtkWarningMacro(
        "Line '" << line
        << "' has only " << fields.size() << "fields "
        << "(expected " << numFields << ")");
      continue;
      }

    // Assign values to data arrays
    for (FieldDataMap::iterator iter = this->Internal->FieldMapping.begin();
         iter != this->Internal->FieldMapping.end(); ++iter)
      {
      const double value = boost::lexical_cast<double>(fields[iter->first]);
      iter->second->InsertNextValue(value);
      }

    ++count;
    }

  // Verify position information
  if (timeData->GetNumberOfTuples() != count ||
      eastingData->GetNumberOfTuples() != count ||
      northingData->GetNumberOfTuples() != count ||
      heightData->GetNumberOfTuples() != count ||
      rollData->GetNumberOfTuples() != count ||
      pitchData->GetNumberOfTuples() != count ||
      headingData->GetNumberOfTuples() != count)
    {
    vtkErrorMacro(
      "Failed to extract points: one or more position fields has fewer values"
      " than the number of readable records in the input file");
    return VTK_ERROR;
    }

  // Build polyline and transform interpolator
  points->Allocate(count);
  polyIds->Allocate(count);

  this->Internal->Interpolator->SetInterpolationTypeToLinear();
  this->Internal->Interpolator->Initialize();

  double pos[3] = { 0.0, 0.0, 0.0 };
  double firstPos[3];
  for (vtkIdType n = 0; n < count; ++n)
    {
    if (n == 0)
      {
      firstPos[0] = eastingData->GetValue(n);
      firstPos[1] = northingData->GetValue(n);
      firstPos[2] = heightData->GetValue(n);
      }
    else
      {
      pos[0] = eastingData->GetValue(n) - firstPos[0];
      pos[1] = northingData->GetValue(n) - firstPos[1];
      pos[2] = heightData->GetValue(n) - firstPos[2];
      }

    points->InsertNextPoint(pos);
    polyIds->InsertNextId(n);

    vtkNew<vtkTransform> transform;
    transform->PostMultiply();
    transform->RotateZ(-headingData->GetValue(n) - this->BaseYaw);
    transform->RotateY(rollData->GetValue(n) - this->BaseRoll);
    transform->RotateX(pitchData->GetValue(n) - this->BasePitch);
    transform->Translate(pos);

    const double timestamp = timeData->GetValue(n) - this->TimeOffset;
    this->Internal->Interpolator->AddTransform(timestamp,
                                               transform.GetPointer());
    }

  cells->InsertNextCell(polyLine.GetPointer());

  // Fill output port
  output->SetPoints(points.GetPointer());
  output->SetLines(cells.GetPointer());

  output->GetFieldData()->AddArray(zoneData.GetPointer());
  for (FieldDataMap::iterator iter = this->Internal->FieldMapping.begin();
       iter != this->Internal->FieldMapping.end(); ++iter)
    {
    output->GetPointData()->AddArray(iter->second);
    }

  return VTK_OK;
}

//-----------------------------------------------------------------------------
void vtkApplanixPositionReader::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "FileName: " << this->FileName << endl;
}
