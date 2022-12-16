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

#ifndef OR_TOOLS_MATH_OPT_CPP_SUBPROCESS_SOLVE_H_
#define OR_TOOLS_MATH_OPT_CPP_SUBPROCESS_SOLVE_H_

#include <memory>

#include "absl/status/statusor.h"
#include "ortools/math_opt/core/solve_interrupter.h"  // IWYU pragma: export
#include "ortools/math_opt/cpp/model.h"
#include "ortools/math_opt/cpp/parameters.h"       // IWYU pragma: export
#include "ortools/math_opt/cpp/solve_arguments.h"  // IWYU pragma: export
#include "ortools/math_opt/cpp/solve_result.h"     // IWYU pragma: export
#include "ortools/math_opt/cpp/subprocess_solver_init_arguments.h"  // IWYU pragma: export
#include "ortools/math_opt/cpp/update_result.h"   // IWYU pragma: export
#include "ortools/math_opt/cpp/update_tracker.h"  // IWYU pragma: export
#include "ortools/math_opt/storage/model_storage.h"
#include "ortools/math_opt/subprocess/core/subprocess_solver.h"  // IWYU pragma: export

namespace operations_research::math_opt {

// Solves the input model in a subprocess.
//
// This is similar to Solve() but instead of solving in-process it solves in a
// subprocess.
//
// One important difference is that this function supports both interruption
// (with `solve_args.interrupter`) and cancellation. The subprocess is killed
// and this function returns absl::CancelledError. This can be done by setting
// `init_args.canceller`.
absl::StatusOr<SolveResult> SubprocessSolve(
    const Model& model, SolverType solver_type,
    const SolveArguments& solve_args = {},
    const SubprocessSolverInitArguments& init_args = {});

// Incremental solve of a model in a subprocess.
//
// This is similar to IncrementalSolver but instead of solving in-process it
// solves in a subprocess.
//
// One important difference is that this the Update() and Solve() support
// cancellation. The subprocess is killed and this function returns
// absl::CancelledError. This can be done by setting `init_args.canceller` (see
// documentation above).
class SubprocessIncrementalSolver {
 public:
  // Creates a new incremental solve for the given model. It may returns an
  // error if the parameters are invalid (for example if the selected solver is
  // not linked in the binary).
  //
  // The returned instance registers on the Model to keep track of updates (see
  // IncrementalSolver class documentation for details).
  static absl::StatusOr<std::unique_ptr<SubprocessIncrementalSolver>> New(
      Model* model, SolverType solver_type,
      SubprocessSolverInitArguments arguments = {});

  // Updates the underlying solver with latest model changes and runs the solve.
  //
  // See IncrementalSolver::Solve() for details.
  absl::StatusOr<SolveResult> Solve(const SolveArguments& arguments = {});

  // Updates the model to solve.
  //
  // This is an advanced API, most users should use Solve() above that does the
  // update and before calling the solver. Calling this function is only useful
  // for users that want to access to update data or users that need to use
  // SolveWithoutUpdate() (which should not be common).
  //
  // See IncrementalSolver::Update() for details.
  absl::StatusOr<UpdateResult> Update();

  // Same as Solve() but does not update the underlying solver with the latest
  // changes to the model.
  //
  // This is an advanced API, most users should use Solve().
  absl::StatusOr<SolveResult> SolveWithoutUpdate(
      const SolveArguments& arguments = {}) const;

 private:
  SubprocessIncrementalSolver(
      SolverType solver_type, SubprocessSolverInitArguments init_args,
      std::shared_ptr<SolveInterrupter> local_canceller,
      std::unique_ptr<const ScopedSolveInterrupterCallback> user_canceller_cb,
      const ModelStorage* expected_storage,
      std::unique_ptr<UpdateTracker> update_tracker,
      std::unique_ptr<SubprocessSolver> solver);

  const SolverType solver_type_;
  const SubprocessSolverInitArguments init_args_;
  std::shared_ptr<SolveInterrupter> local_canceller_;
  std::unique_ptr<const ScopedSolveInterrupterCallback> user_canceller_cb_;
  const ModelStorage* const expected_storage_;
  const std::unique_ptr<UpdateTracker> update_tracker_;
  std::unique_ptr<SubprocessSolver> subprocess_solver_;
};

}  // namespace operations_research::math_opt

#endif  // OR_TOOLS_MATH_OPT_CPP_SUBPROCESS_SOLVE_H_
