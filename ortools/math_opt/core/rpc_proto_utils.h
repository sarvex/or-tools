// Copyright 2010-2022 Google LLC
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

#ifndef OR_TOOLS_MATH_OPT_CORE_RPC_PROTO_UTILS_H_
#define OR_TOOLS_MATH_OPT_CORE_RPC_PROTO_UTILS_H_

#include <string>

#include "ortools/math_opt/rpc.pb.h"

namespace operations_research::math_opt {

// Returns a string corresponding to the request case in camel case. If the
// value is unknown, returns a "unknown<n>" where n is its integer value. If it
// is MESSAGE_NOT_SET, returns "not_set".
std::string StreamSolveRequestMessageCaseToString(
    StreamSolveRequest::MessageCase message_case);

// Returns a string corresponding to the response case in camel case. If the
// value is unknown, returns a "unknown<n>" where n is its integer value. If it
// is MESSAGE_NOT_SET, returns "not_set".
std::string StreamSolveResponseMessageCaseToString(
    StreamSolveResponse::MessageCase message_case);

}  // namespace operations_research::math_opt

#endif  // OR_TOOLS_MATH_OPT_CORE_RPC_PROTO_UTILS_H_
