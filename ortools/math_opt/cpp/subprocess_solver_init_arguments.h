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

#ifndef OR_TOOLS_MATH_OPT_CPP_SUBPROCESS_SOLVER_INIT_ARGUMENTS_H_
#define OR_TOOLS_MATH_OPT_CPP_SUBPROCESS_SOLVER_INIT_ARGUMENTS_H_

#include "ortools/math_opt/core/solve_interrupter.h"  // IWYU pragma: export
#include "ortools/math_opt/cpp/streamable_solver_init_arguments.h"  // IWYU pragma: export
#include "ortools/math_opt/subprocess/core/sandboxing.h"  // IWYU pragma: export

namespace operations_research::math_opt {

// Arguments passed to SubprocessSolve() and SubprocessIncrementalSolver::New()
// to control the instantiation of the subprocess solver.
struct SubprocessSolverInitArguments {
  // An optional interrupter that cancels any pending and future interactions
  // with a SubprocessSolve() or SubprocessIncrementalSolver (by
  // terminating/killing the underlying subprocess).
  //
  // Pending and future calls to SubprocessSolve(),
  // SubprocessIncrementalSolver's Solve() and Update() will return an
  // absl::CancelledError().
  //
  // The SubprocessIncrementalSolver does not take ownership of the interrupter,
  // and the interrupter object must stay alive for the whole lifetime of the
  // solver.
  //
  // The differences between this parameter and SolveArguments::interrupter are:
  //   * SolveArguments::interrupter does not cancel the underlying solver; the
  //     solver finishes early and the best found solution before the
  //     interruption is returned.
  //   * SolveArguments::interrupter may not have an immediate effect; it sends
  //     a signal to the underlying solver that it should stop, but the solver
  //     may continue the computation for a certain amount of time before it
  //     checks this signal.
  //   * SolveArguments::interrupter is specific to a Solve() while canceller is
  //     for the SubprocessIncrementalSolver itself; since the cancellation
  //     involves killing the subprocess, it is not usable afterwards.
  //
  // This interrupter should be used in cases where the solve result is not
  // important anymore, for example in a server if the request that triggered
  // the solve is cancelled.
  //
  // TODO(b/192485712): add here or in SubprocessSolver a way to trigger
  // cancellation on fiber cancellation as it would make sense in most cases.
  SolveInterrupter* canceller = nullptr;

  // The sandboxing mode for binaries containing solvers that requires it
  // (e.g. Gurobi which is a closed-source third-party library).
  //
  // SubprocessSolve() or SubprocessIncrementalSolver::New() returns a failing
  // Status when `sandboxing` is Sandboxing::kNo and the combination of binary
  // and environment requires sandboxing (e.g. running a binary that contains
  // Gurobi on Forge or Borg).
  //
  // By default we always use sandboxing for binaries that support it even
  // when not running on Forge or Borg. This can be an issue for solvers that
  // require a local license (e.g. Gurobi) since it won't be accessible from
  // the sandbox. Hence a typical use of this setting is when using
  // SubprocessSolver on a machine with a local installation of Gurobi.
  Sandboxing sandboxing = Sandboxing::kIfSupported;

  // All parameters that can be stored in a proto and exchange with other
  // processes.
  StreamableSolverInitArguments streamable;

  // TODO(b/192485712): add an enum to select subprocess solving or in-process
  // solving here (with default value being subprocess solving) which could be
  // useful for debugging and which will make sense for RPC as well (maybe we
  // should use the same enum for RPC as well and have a three value enum; at
  // the expense of being a bit confusing for non-RPC use-case).
};

}  // namespace operations_research::math_opt

#endif  // OR_TOOLS_MATH_OPT_CPP_SUBPROCESS_SOLVER_INIT_ARGUMENTS_H_
