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

// This source if fairly similar to solve.cc. The key differences are:
// * We use SubprocessSolver instead of Solver.
// * We use SubprocessSolverInitArguments which:
//   - It does not support non-streamable arguments (like a pointer on Gurobi
//     environment).
//   - It has an additional `canceller` interrupter to cancel the solve.
// * When a callback fails, we kill the subprocess instead of requiring a
//   cooperative interruption.
#include "ortools/math_opt/cpp/subprocess_solve.h"

#include <functional>
#include <memory>
#include <optional>
#include <utility>

#include "absl/container/flat_hash_set.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "ortools/base/check.h"
#include "ortools/base/status_macros.h"
#include "ortools/math_opt/cpp/math_opt.h"
#include "ortools/util/status_macros.h"

namespace operations_research::math_opt {
namespace {

absl::StatusOr<SolveResult> CallSolve(
    SubprocessSolver& solver, const ModelStorage* const expected_storage,
    const SolveArguments& arguments, SolveInterrupter& local_canceller) {
  RETURN_IF_ERROR(arguments.CheckModelStorageAndCallback(expected_storage));

  Solver::Callback cb = nullptr;
  absl::Mutex mutex;
  absl::Status cb_status;  // Guarded by `mutex`.
  if (arguments.callback != nullptr) {
    cb = [&](const CallbackDataProto& callback_data_proto) {
      const CallbackData data(expected_storage, callback_data_proto);
      const CallbackResult result = arguments.callback(data);
      if (const absl::Status status =
              result.CheckModelStorage(expected_storage);
          !status.ok()) {
        // Note that we use util::StatusBuilder() here as util::Annotate() is
        // not available in open-source code.
        util::StatusBuilder builder(status);
        builder << "invalid CallbackResult returned by user callback";

        const absl::MutexLock lock(&mutex);
        cb_status.Update(builder);

        // Trigger subprocess cancellation.
        local_canceller.Interrupt();
        return CallbackResultProto{};
      }
      return result.Proto();
    };
  }

  const absl::StatusOr<SolveResultProto> solve_result_proto = solver.Solve(
      {.parameters = arguments.parameters.Proto(),
       .model_parameters = arguments.model_parameters.Proto(),
       .message_callback = arguments.message_callback,
       .callback_registration = arguments.callback_registration.Proto(),
       .user_cb = std::move(cb),
       .interrupter = arguments.interrupter});

  // solver.Solve() returns an error on cancellation by local_canceller but in
  // that case we want to ignore this error and return status generated in the
  // callback instead.
  const absl::MutexLock lock(&mutex);
  RETURN_IF_ERROR(cb_status);

  if (!solve_result_proto.ok()) {
    return solve_result_proto.status();
  }

  return SolveResult::FromProto(expected_storage, solve_result_proto.value());
}

SubprocessSolver::InitArgs ToSubprocessSolverInitArgs(
    const SubprocessSolverInitArguments& arguments,
    SolveInterrupter* local_canceller) {
  CHECK_NE(local_canceller, nullptr);
  return {.canceller = local_canceller,
          .sandboxing = arguments.sandboxing,
          .streamable = arguments.streamable.Proto()};
}

}  // namespace

absl::StatusOr<SolveResult> SubprocessSolve(
    const Model& model, const SolverType solver_type,
    const SolveArguments& solve_args,
    const SubprocessSolverInitArguments& init_args) {
  SolveInterrupter local_canceller;
  const ScopedSolveInterrupterCallback user_canceller_cb(
      init_args.canceller, [&]() { local_canceller.Interrupt(); });
  ASSIGN_OR_RETURN(
      const std::unique_ptr<SubprocessSolver> subprocess_solver,
      SubprocessSolver::New(
          EnumToProto(solver_type), model.ExportModel(),
          ToSubprocessSolverInitArgs(init_args, &local_canceller)));
  return CallSolve(*subprocess_solver, model.storage(), solve_args,
                   local_canceller);
}

absl::StatusOr<std::unique_ptr<SubprocessIncrementalSolver>>
SubprocessIncrementalSolver::New(Model* const model,
                                 const SolverType solver_type,
                                 SubprocessSolverInitArguments arguments) {
  if (model == nullptr) {
    return absl::InvalidArgumentError("input model can't be null");
  }
  auto local_canceller = std::make_shared<SolveInterrupter>();
  auto user_canceller_cb =
      std::make_unique<const ScopedSolveInterrupterCallback>(
          arguments.canceller,
          [local_canceller]() { local_canceller->Interrupt(); });
  std::unique_ptr<UpdateTracker> update_tracker = model->NewUpdateTracker();
  ASSIGN_OR_RETURN(const ModelProto model_proto, update_tracker->ExportModel());
  ASSIGN_OR_RETURN(
      std::unique_ptr<SubprocessSolver> solver,
      SubprocessSolver::New(
          EnumToProto(solver_type), model_proto,
          ToSubprocessSolverInitArgs(arguments, local_canceller.get())));
  return absl::WrapUnique<SubprocessIncrementalSolver>(
      new SubprocessIncrementalSolver(
          solver_type, std::move(arguments), std::move(local_canceller),
          std::move(user_canceller_cb), model->storage(),
          std::move(update_tracker), std::move(solver)));
}

SubprocessIncrementalSolver::SubprocessIncrementalSolver(
    SolverType solver_type, SubprocessSolverInitArguments init_args,
    std::shared_ptr<SolveInterrupter> local_canceller,
    std::unique_ptr<const ScopedSolveInterrupterCallback> user_canceller_cb,
    const ModelStorage* const expected_storage,
    std::unique_ptr<UpdateTracker> update_tracker,
    std::unique_ptr<SubprocessSolver> solver)
    : solver_type_(solver_type),
      init_args_(std::move(init_args)),
      local_canceller_(std::move(local_canceller)),
      user_canceller_cb_(std::move(user_canceller_cb)),
      expected_storage_(expected_storage),
      update_tracker_(std::move(update_tracker)),
      subprocess_solver_(std::move(solver)) {}

absl::StatusOr<SolveResult> SubprocessIncrementalSolver::Solve(
    const SolveArguments& arguments) {
  RETURN_IF_ERROR(Update().status());
  return SolveWithoutUpdate(arguments);
}

absl::StatusOr<UpdateResult> SubprocessIncrementalSolver::Update() {
  ASSIGN_OR_RETURN(std::optional<ModelUpdateProto> model_update,
                   update_tracker_->ExportModelUpdate());
  if (!model_update.has_value()) {
    return UpdateResult(true, std::move(model_update));
  }

  OR_ASSIGN_OR_RETURN3(const bool did_update,
                       subprocess_solver_->Update(*model_update),
                       _ << "update failed");
  RETURN_IF_ERROR(update_tracker_->AdvanceCheckpoint());

  if (did_update) {
    return UpdateResult(true, std::move(model_update));
  }

  ASSIGN_OR_RETURN(const ModelProto model_proto,
                   update_tracker_->ExportModel());
  OR_ASSIGN_OR_RETURN3(
      subprocess_solver_,
      SubprocessSolver::New(
          EnumToProto(solver_type_), model_proto,
          ToSubprocessSolverInitArgs(init_args_, local_canceller_.get())),
      _ << "solver re-creation failed");

  return UpdateResult(false, std::move(model_update));
}

absl::StatusOr<SolveResult> SubprocessIncrementalSolver::SolveWithoutUpdate(
    const SolveArguments& arguments) const {
  return CallSolve(*subprocess_solver_, expected_storage_, arguments,
                   *local_canceller_);
}

}  // namespace operations_research::math_opt
