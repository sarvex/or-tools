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

#include "ortools/math_opt/core/rpc_proto_utils.h"

#include <string>

#include "absl/strings/str_cat.h"
#include "ortools/math_opt/rpc.pb.h"

namespace operations_research::math_opt {

std::string StreamSolveRequestMessageCaseToString(
    const StreamSolveRequest::MessageCase message_case) {
  switch (message_case) {
    case StreamSolveRequest::MESSAGE_NOT_SET:
      return "not_set";
    case StreamSolveRequest::kInit:
      return "init";
    case StreamSolveRequest::kUpdate:
      return "update";
    case StreamSolveRequest::kSolve:
      return "solve";
    case StreamSolveRequest::kCallbackResponse:
      return "callback_response";
    case StreamSolveRequest::kInterrupt:
      return "interrupt";
    default:
      return absl::StrCat("unknown<", message_case, ">");
  }
}

std::string StreamSolveResponseMessageCaseToString(
    const StreamSolveResponse::MessageCase message_case) {
  switch (message_case) {
    case StreamSolveResponse::MESSAGE_NOT_SET:
      return "not_set";
    case StreamSolveResponse::kInitResponse:
      return "init_response";
    case StreamSolveResponse::kUpdateResponse:
      return "update_response";
    case StreamSolveResponse::kSolveResponse:
      return "solve_response";
    case StreamSolveResponse::kCallback:
      return "callback";
    case StreamSolveResponse::kInterruptResponse:
      return "interrupt_response";
    default:
      return absl::StrCat("unknown<", message_case, ">");
  }
}

}  // namespace operations_research::math_opt
