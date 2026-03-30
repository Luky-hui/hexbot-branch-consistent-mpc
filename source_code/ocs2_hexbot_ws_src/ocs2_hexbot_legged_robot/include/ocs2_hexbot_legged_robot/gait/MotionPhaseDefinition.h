/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <algorithm>
#include <array>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

#include "ocs2_hexbot_legged_robot/common/Types.h"

namespace ocs2 {
namespace legged_robot {

enum ModeNumber : size_t {  // {RR, RM, RF, LR, LM, LF}
  FLY = 0,
  RR = 1u << 0,
  RM = 1u << 1,
  RF = 1u << 2,
  LR = 1u << 3,
  LM = 1u << 4,
  LF = 1u << 5,
  TRIPOD_A = RR | RF | LM,
  TRIPOD_B = RM | LR | LF,
  STANCE = RR | RM | RF | LR | LM | LF,
};

inline std::array<std::string, 6> orderedLegNames() {
  return {"RR", "RM", "RF", "LR", "LM", "LF"};
}

inline contact_flag_t modeNumber2StanceLeg(const size_t& modeNumber) {
  contact_flag_t stanceLegs{};
  for (size_t i = 0; i < stanceLegs.size(); ++i) {
    stanceLegs[i] = ((modeNumber >> i) & 0x1u) != 0u;
  }
  return stanceLegs;
}

inline size_t stanceLeg2ModeNumber(const contact_flag_t& stanceLegs) {
  size_t modeNumber = 0;
  for (size_t i = 0; i < stanceLegs.size(); ++i) {
    if (stanceLegs[i]) {
      modeNumber |= (1u << i);
    }
  }
  return modeNumber;
}

inline std::string modeNumber2String(const size_t& modeNumber) {
  static const std::map<size_t, std::string> specialNames = {
      {FLY, "FLY"},
      {TRIPOD_A, "RR_RF_LM"},
      {TRIPOD_B, "RM_LR_LF"},
      {STANCE, "STANCE"},
  };
  const auto special = specialNames.find(modeNumber);
  if (special != specialNames.end()) {
    return special->second;
  }

  const auto legNames = orderedLegNames();
  const auto stanceLegs = modeNumber2StanceLeg(modeNumber);
  std::ostringstream stream;
  bool first = true;
  for (size_t i = 0; i < stanceLegs.size(); ++i) {
    if (!stanceLegs[i]) {
      continue;
    }
    if (!first) {
      stream << "_";
    }
    stream << legNames[i];
    first = false;
  }
  return first ? "FLY" : stream.str();
}

inline size_t string2ModeNumber(const std::string& modeString) {
  static const std::map<std::string, size_t> specialNames = {
      {"FLY", FLY},
      {"STANCE", STANCE},
      {"TRIPOD_A", TRIPOD_A},
      {"TRIPOD_B", TRIPOD_B},
      {"RR_RF_LM", TRIPOD_A},
      {"RM_LR_LF", TRIPOD_B},
  };

  const auto special = specialNames.find(modeString);
  if (special != specialNames.end()) {
    return special->second;
  }

  contact_flag_t stanceLegs{};
  const auto legNames = orderedLegNames();
  std::stringstream stream(modeString);
  std::string token;
  while (std::getline(stream, token, '_')) {
    const auto it = std::find(legNames.begin(), legNames.end(), token);
    if (it == legNames.end()) {
      throw std::runtime_error("[string2ModeNumber] Unknown Hexbot gait token: " + token);
    }
    stanceLegs[static_cast<size_t>(std::distance(legNames.begin(), it))] = true;
  }
  return stanceLeg2ModeNumber(stanceLegs);
}

}  // namespace legged_robot
}  // end of namespace ocs2
