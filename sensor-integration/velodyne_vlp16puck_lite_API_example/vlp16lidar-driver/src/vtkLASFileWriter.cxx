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

/*========================================================================*
Modifications copyright (C) 2016  DJI.  All rights reserved.
==========================================================================*/


#include "vtkLASFileWriter.h"

#include <vtkPointData.h>
#include <vtkPolyData.h>

#include <vtk_libproj4.h>

#include <liblas/liblas.hpp>

#include <eigen3/Eigen/Dense>

#ifndef PJ_VERSION // 4.8 or later
#include <cassert>
#endif


namespace
{

#ifdef PJ_VERSION // 4.8 or later

//-----------------------------------------------------------------------------
projPJ CreateProj(int epsg)
{
  std::ostringstream ss;
  ss << "+init=epsg:" << epsg;
  return pj_init_plus(ss.str().c_str());
}

//-----------------------------------------------------------------------------
Eigen::Vector3d ConvertGcs(
  Eigen::Vector3d p, projPJ inProj, projPJ outProj)
{
  if (pj_is_latlong(inProj))
    {
    p[0] *= DEG_TO_RAD;
    p[1] *= DEG_TO_RAD;
    }

  double* const data = p.data();
  pj_transform(inProj, outProj, 1, 1, data + 0, data + 1, data + 2);

  if (pj_is_latlong(outProj))
    {
    p[0] *= RAD_TO_DEG;
    p[1] *= RAD_TO_DEG;
    }

  return p;
}

#else

//-----------------------------------------------------------------------------
PROJ* CreateProj(int utmZone, bool south)
{
  std::ostringstream ss;
  ss << "+zone=" << utmZone;

  char buffer[65] = { 0 };
  strncpy(buffer, ss.str().c_str(), 64);

  std::vector<const char*> utmparams;
  utmparams.push_back("+proj=utm");
  utmparams.push_back("+ellps=WGS84");
  utmparams.push_back("+units=m");
  utmparams.push_back("+no_defs");
  utmparams.push_back(buffer);
  if (south)
    {
    utmparams.push_back("+south");
    }

  return proj_init(utmparams.size(), const_cast<char**>(&(utmparams[0])));
}

//-----------------------------------------------------------------------------
Eigen::Vector3d InvertProj(Eigen::Vector3d in, PROJ* proj)
{
  // This "lovely little gem" makes an awful lot of assumptions about the input
  // (some flavor of XY) and output (internal LP, which we assume / hope is
  // WGS'84) GCS's and what operations are "interesting" (i.e. the assumption
  // that the Z component does not need to be considered and can be passed
  // through unaltered). Unfortunately, it's the best we can do with PROJ 4.7
  // until VTK can be updated to use 4.8. Fortunately, given how we're being
  // used, our input really ought to always be UTM.
  PROJ_XY xy;
  xy.x = in[0];
  xy.y = in[1];

  const PROJ_LP lp = proj_inv(xy, proj);
  return Eigen::Vector3d(lp.lam * RAD_TO_DEG, lp.phi * RAD_TO_DEG, in[2]);
}

#endif

}

//-----------------------------------------------------------------------------
class vtkLASFileWriter::vtkInternal
{
public:
  void Close();

  std::ofstream Stream;
  liblas::Writer* Writer;

  double MinTime;
  double MaxTime;
  Eigen::Vector3d Origin;

  size_t npoints;
  double MinPt[3];
  double MaxPt[3];

#ifdef PJ_VERSION // 4.8 or later
  projPJ InProj;
  projPJ OutProj;
#else
  PROJ* Proj;
#endif
  int OutGcs;
};

//-----------------------------------------------------------------------------
void vtkLASFileWriter::vtkInternal::Close()
{
  delete this->Writer;
  this->Writer = 0;
  this->Stream.close();
}

vtkLASFileWriter::vtkLASFileWriter()
    : Internal(new vtkInternal)
{
    this->Internal->MinTime = -std::numeric_limits<double>::infinity();
    this->Internal->MaxTime = +std::numeric_limits<double>::infinity();

  #ifdef PJ_VERSION // 4.8 or later
    this->Internal->InProj = 0;
    this->Internal->OutProj = 0;
  #else
    this->Internal->Proj = 0;
  #endif
    this->Internal->OutGcs = -1;

    this->Internal->npoints = 0;

    for(int i = 0; i < 3; ++i)
      {
      this->Internal->MaxPt[i] = -std::numeric_limits<double>::max();
      this->Internal->MinPt[i] = std::numeric_limits<double>::max();
      }

}

//-----------------------------------------------------------------------------
void vtkLASFileWriter::vtkLASFileName(const char* filename)
{

  this->Internal->Stream.open(
    filename, std::ios::out | std::ios::trunc | std::ios::binary);

  liblas::Header header;
  header.SetSoftwareId("DJIonboardSDK");
  header.SetDataFormatId(liblas::ePointFormat1);
  header.SetScale(1e-3, 1e-3, 1e-3);
  header.SetMin(-100.0, -100.0, -100.0);
  header.SetMax(100.0, 100.0, 100.0);
  this->Internal->Writer = new liblas::Writer(this->Internal->Stream, header);

}

//-----------------------------------------------------------------------------
vtkLASFileWriter::~vtkLASFileWriter()
{
  this->Internal->Close();

#ifdef PJ_VERSION // 4.8 or later
  pj_free(this->Internal->InProj);
  pj_free(this->Internal->OutProj);
#else
  proj_free(this->Internal->Proj);
#endif

  delete this->Internal;
}

//-----------------------------------------------------------------------------
void vtkLASFileWriter::SetTimeRange(double min, double max)
{
  this->Internal->MinTime = min;
  this->Internal->MaxTime = max;
}

//-----------------------------------------------------------------------------
void vtkLASFileWriter::SetOrigin(
  int gcs, double easting, double northing, double height)
{
  // Set internal UTM offset
  Eigen::Vector3d origin(easting, northing, height);
  this->Internal->Origin = origin;

  // Convert offset to output GCS, if a geoconversion is set up
#ifdef PJ_VERSION // 4.8 or later
  if (this->Internal->OutProj)
    {
    origin =
      ConvertGcs(origin, this->Internal->InProj, this->Internal->OutProj);
    gcs = this->Internal->OutGcs;
    }
#else
  if (this->Internal->Proj)
    {
    origin = InvertProj(origin, this->Internal->Proj);
    gcs = this->Internal->OutGcs;
    }
#endif

  // Update header
  liblas::Header header = this->Internal->Writer->GetHeader();

  header.SetOffset(origin[0], origin[1], origin[2]);

  try
    {
    liblas::SpatialReference srs;
    std::ostringstream ss;
    ss << "EPSG:" << gcs;
    srs.SetFromUserInput(ss.str());

    header.SetSRS(srs);
    }
  catch (std::logic_error)
    {
    std::cerr << "failed to set SRS (logic)" << std::endl;
    }
  catch (std::runtime_error)
    {
    std::cerr << "failed to set SRS" << std::endl;
    }

  this->Internal->Writer->SetHeader(header);
  this->Internal->Writer->WriteHeader();
}

//-----------------------------------------------------------------------------
void vtkLASFileWriter::SetGeoConversion(int in, int out)
{
#ifdef PJ_VERSION // 4.8 or later
  pj_free(this->Internal->InProj);
  pj_free(this->Internal->OutProj);

  this->Internal->InProj = CreateProj(in);
  this->Internal->OutProj = CreateProj(out);
#else
  // The PROJ 4.7 API makes it near impossible to do generic transforms, hence
  // InvertProj (see also comments there) is full of assumptions. Assert some
  // of those assumptions here.
  assert((in > 32600 && in < 32661) || (in > 32700 && in < 32761));
  assert(out == 4326);

  proj_free(this->Internal->Proj);
  this->Internal->Proj = CreateProj(in % 100, in > 32700);
#endif

  this->Internal->OutGcs = out;
}

//-----------------------------------------------------------------------------
void vtkLASFileWriter::SetPrecision(double neTol, double hTol)
{
  liblas::Header header = this->Internal->Writer->GetHeader();
  header.SetScale(neTol, neTol, hTol);
  this->Internal->Writer->SetHeader(header);
  this->Internal->Writer->WriteHeader();
}

//-----------------------------------------------------------------------------
void vtkLASFileWriter::WriteFrame(vtkPolyData* data)
{
  vtkPoints* const points = data->GetPoints();
  vtkDataArray* const intensityData =
    data->GetPointData()->GetArray("intensity");
  vtkDataArray* const laserIdData =
    data->GetPointData()->GetArray("laser_id");
  vtkDataArray* const timestampData =
    data->GetPointData()->GetArray("timestamp");

  const vtkIdType numPoints = points->GetNumberOfPoints();
  for (vtkIdType n = 0; n < numPoints; ++n)
    {
    const double time = timestampData->GetComponent(n, 0) * 1e-6;
    if (time >= this->Internal->MinTime && time <= this->Internal->MaxTime)
      {
      Eigen::Vector3d pos;
      points->GetPoint(n, pos.data());
      pos += this->Internal->Origin;

#ifdef PJ_VERSION // 4.8 or later
      if (this->Internal->OutProj)
        {
        pos = ConvertGcs(pos, this->Internal->InProj, this->Internal->OutProj);
        }
#else
      if (this->Internal->Proj)
        {
        pos = InvertProj(pos, this->Internal->Proj);
        }
#endif

      liblas::Point p(&this->Internal->Writer->GetHeader());
      p.SetCoordinates(pos[0], pos[1], pos[2]);
      p.SetIntensity(static_cast<uint16_t>(intensityData->GetComponent(n, 0)));
      p.SetReturnNumber(1);
      p.SetNumberOfReturns(1);
      p.SetUserData(static_cast<uint8_t>(laserIdData->GetComponent(n, 0)));
      p.SetTime(time);

      this->Internal->Writer->WritePoint(p);
      }
    }
}

//-----------------------------------------------------------------------------
void vtkLASFileWriter::FlushMetaData()
{
  liblas::Header header = this->Internal->Writer->GetHeader();

  header.SetPointRecordsByReturnCount(0, this->Internal->npoints);
  header.SetMin(this->Internal->MinPt[0], this->Internal->MinPt[1], this->Internal->MinPt[2]);
  header.SetMax(this->Internal->MaxPt[0], this->Internal->MaxPt[1], this->Internal->MaxPt[2]);

  this->Internal->Writer->SetHeader(header);
  this->Internal->Writer->WriteHeader();
}

//-----------------------------------------------------------------------------
void vtkLASFileWriter::UpdateMetaData(vtkPolyData* data)
{
  vtkPoints* const points = data->GetPoints();
  vtkDataArray* const timestampData =
    data->GetPointData()->GetArray("timestamp");

  const vtkIdType numPoints = points->GetNumberOfPoints();
  for (vtkIdType n = 0; n < numPoints; ++n)
    {
    const double time = timestampData->GetComponent(n, 0) * 1e-6;
    if (time >= this->Internal->MinTime && time <= this->Internal->MaxTime)
      {
      Eigen::Vector3d pos;
      points->GetPoint(n, pos.data());
      pos += this->Internal->Origin;

#ifdef PJ_VERSION // 4.8 or later
      if (this->Internal->OutProj)
        {
        pos = ConvertGcs(pos, this->Internal->InProj, this->Internal->OutProj);
        }
#else
      if (this->Internal->Proj)
        {
        pos = InvertProj(pos, this->Internal->Proj);
        }
#endif

      this->Internal->npoints++;

      for(int i = 0; i < 3; ++i)
        {
        if(pos[i] > this->Internal->MaxPt[i])
          {
          this->Internal->MaxPt[i] = pos[i];
          }
        if(pos[i] < this->Internal->MinPt[i])
          {
          this->Internal->MinPt[i] = pos[i];
          }
        }

      }
    }
}

//-----------------------------------------------------------------------------
void vtkLASFileWriter::vtkLASFileClose(void)
{
  this->Internal->Close();

}



