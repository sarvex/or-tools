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

#ifndef OR_TOOLS_MATH_OPT_CPP_REMOTE_SOLVE_H_
#define OR_TOOLS_MATH_OPT_CPP_REMOTE_SOLVE_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "net/eventmanager/eventmanager_default.h"
#include "net/eventmanager/eventmanager_interface.h"
#include "net/rpc/rpc.h"
#include "ortools/math_opt/core/solve_interrupter.h"
#include "ortools/math_opt/cpp/model.h"
#include "ortools/math_opt/cpp/model_solve_parameters.h"
#include "ortools/math_opt/cpp/solve_arguments.h"
#include "ortools/math_opt/cpp/solve_result.h"
#include "ortools/math_opt/rpc.stubby.h"
#include "production/rpc/stubs/public/canonical_stub_options.h"

namespace operations_research::math_opt {

// Default options for an Extensible Stub to connect to SolveServer (go/uoss)
// for math_opt::SolveService RPCs, see SolveServerStub() for details.
production::rpc::stubs::CanonicalStubOptions DefaultStubOptions();

absl::StatusOr<production::rpc::stubs::CanonicalStubOptions>
StubOptionsFromFlags();

// This function returns an "Extensible Stub" (go/canonicalstub) to connect to
// SolveServer (go/uoss) for math_opt::SolveService RPCs.
//
// The timeout argument is used to call WaitUntilReachableWithTimeout(). A value
// of timeout <= zero bypasses this call (it is then the user's responsibility).
// Callers who are creating multiple stubs should use timeout of zero and then
// manually call WaitUntilReachableWithTimeout() on each stub after all stubs
// have been created so that the waiting can be done in "parallel". For details
// see go/channel-reachability.
absl::StatusOr<std::unique_ptr<SolveService>> SolveServerStub(
    const production::rpc::stubs::CanonicalStubOptions& options =
        DefaultStubOptions(),
    absl::Duration timeout = absl::Minutes(2));

struct SolveResultAndLogs {
  SolveResult solve_result;

  // The logs from the underlying solver, if supported.
  std::vector<std::string> messages;
};

struct RemoteSolveArgs {
  SolveParameters parameters;
  ModelSolveParameters model_parameters;
};

// Solves the input model remotely on the server from stub.
//
// See also ::operations_research::math_opt::Solve() to solve locally, and for
// more detailed documentation.
//
// NOTE: the rpc object is mutable even though RemoteSolveArgs is const.
absl::StatusOr<SolveResultAndLogs> RemoteSolve(
    SolveService& stub, const Model& model, SolverType solver_type,
    const RemoteSolveArgs& args = {});

}  // namespace operations_research::math_opt

#endif  // OR_TOOLS_MATH_OPT_CPP_REMOTE_SOLVE_H_
