/*! @file dji_version.hpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief
 *  Drone/SDK Version definition for DJI onboardSDK library
 *
 *  @note Since OSDK 3.2.2 (Feb 2017), versioning is handled by the SDK.
 *  You can use the Version::FW macro to target your code towards specific
 * platforms/firmware.
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

#ifndef DJI_VERSION_H
#define DJI_VERSION_H

#include "dji_command.hpp"
#include <cstdint>

namespace DJI
{
namespace OSDK
{

class Version
{
public:
  typedef uint32_t FirmWare;

  constexpr static const char* M100 = "M100";
  constexpr static const char* N3   = "N3";
  constexpr static const char* A3   = "A3";
  constexpr static const char* M210 = "PM410";
  constexpr static const char* M600 = "PM820";

  typedef struct VersionData
  {
    uint16_t version_ack;
    uint32_t version_crc;
    char     hw_serial_num[16];
    char     hwVersion[12]; //! Current longest product code: pm820v3pro
    FirmWare fwVersion;

    //! Legacy member
    char version_name[32];
  } VersionData;

public:
  static const FirmWare FW(uint8_t a, uint8_t b, uint8_t c, uint8_t d);

public:
  //! @todo clean up
  // clang-format off

  // Matrice 100 Firmware version supported by DJI OSDK Core library
  static const FirmWare M100_31           = 0x03010A00;//Version::FW(3, 1, 10,  0  );
  static const FirmWare A3_31             = 0x03016400;//Version::FW(3, 1, 100, 0  );
  static const FirmWare A3_32             = 0x03026400;//Version::FW(3, 2, 100, 0  );

  // clang-format on
}; // class version

/*!
 * @brief Define FW version that supports "mandatory" CMD_SET
 *
 * @details All supported products implement predefined CMD_SET list
 *
 * Supported products: M100, M210, M600, A3, N3
 */
const Version::FirmWare mandatoryVersionBase = (Version::FW(3, 1, 10, 0));

/*!
 * @brief Define FW version that supports "extended" CMD_SET
 *
 * @details Limited products support predefined CMD_SET list
 *
 * Supported products: M210, A3, N3
 */
const Version::FirmWare extendedVersionBase = (Version::FW(3, 2, 36, 6));

/*!
 * @brief Define FW version constant for 3.3.x firmware branch
 * @details Only the A3 and the N3 support the 3.3.x firmware branch.
 * @note Not to be confused with the OSDK version 3.3.x; firmware versions follow a different numbering and cadence.
 */
const Version::FirmWare versionBase33 = (Version::FW(3,3,0,0));

/*!
 * @brief Define CMD_SET support matrix
 */
typedef struct CMD_SETSupportMatrix
{
  uint8_t           cmdSet;
  Version::FirmWare fwVersion;
} CMD_SETSupportMatrix;
} // namespace DJI
} // namespace OSDK

#endif // DJI_VERSION_H
