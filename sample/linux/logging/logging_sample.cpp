/*! @file logging_sample.cpp
 *  @version 3.4
 *  @date Sep 15 2017
 *
 *  @brief
 *  Logging API usage in a Linux environment.
 *  Shows example usage of various logging APIs and controls.
 *
 *  @Copyright (c) 2017 DJI
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

#include "logging_sample.hpp"

bool
dynamicLoggingControlExample()
{
  // First set of logs: we stick with the default logging configuration.
  // Status enabled, Error enabled, debug disabled
  DSTATUS("This status log will be printed.");
  DERROR("This error log will be printed.");
  DDEBUG("This debug log will NOT be printed.");

  // Second set of logs: let's make some changes to the configuration.
  // Status disabled, Error enabled, debug enabled
  DSTATUS("Disabling status logging now...");
  DJI::OSDK::Log::instance().disableStatusLogging();
  DJI::OSDK::Log::instance().enableDebugLogging();
  DDEBUG("Enabled debug logging now...");

  DSTATUS("This status log will NOT be printed.");
  DERROR("This error log will be printed.");
  DDEBUG("This debug log will be printed.");

  // Finally, let's define our own logging channel
  int CUSTOM_LOG = 4; // 4 is a channel - use it for grouping together messages
                      // on a related topic
  DLOG(CUSTOM_LOG)("Here is a custom logging stream!");
  DLOG(CUSTOM_LOG)
  ("All logs are regular expressions of the form \"\\n%%s/%%d @ %%s, L%%d: \"");
  DLOG(CUSTOM_LOG)
  ("You can parse all logs and extract information by log name, channel, "
   "function name, or line number\n");

  DJI::OSDK::Log::instance().enableStatusLogging();
  DSTATUS("Re-enabled Status logging...");
  DSTATUS("Logs start with a newline character - so if different logs are "
          "printed in succession, you don't need to put \\n at the end of each "
          "log.");
  DSTATUS("Make sure to put one a the end of your last log!");
  DSTATUS("Also, you can use printf-style string formatting in logs: for "
          "example, %s has channel number %d",
          "CUSTOM_LOG", CUSTOM_LOG);
  DDEBUG(
    "That's it! Look into the source code to see what didn't get printed.\n");
}