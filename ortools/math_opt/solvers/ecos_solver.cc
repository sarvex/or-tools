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

// TODO(b/260616646): Unsupported features:
//  * Primal/Dual Rays
//  * Infeasible solutions on limit reached
//  * SOCP constraints
//  * Solver specific parameters
//  * Some common parameters (e.g. scaling)
//  * Incrementalism
//
// Supporting message callback and interruption would require minor changes in
// the ECOS source.

#include "ortools/math_opt/solvers/ecos_solver.h"

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/str_join.h"
#include "ortools/base/protoutil.h"
#include "ortools/base/status_macros.h"
#include "ortools/math_opt/core/math_opt_proto_utils.h"
#include "ortools/math_opt/core/solver_interface.h"
#include "ortools/math_opt/core/sparse_vector_view.h"
#include "ortools/math_opt/result.pb.h"
#include "ortools/math_opt/solvers/ecos/ecos_bridge.h"
#include "ortools/math_opt/validators/callback_validator.h"

namespace operations_research::math_opt {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

// For now, just LP.
constexpr SupportedProblemStructures kEcosSupportedStructures = {};

std::string UnsupportedParam(absl::string_view param_name) {
  return absl::StrCat("parameter ", param_name, " not supported for ECOS");
}

// TODO(b/261603235): if we keep filters, we can make this more efficient by
// filtering as we go, instead of building the entire SparseDoubleVectorProto
// first and then filtering.
SparseDoubleVectorProto ApplyFilter(const SparseDoubleVectorProto& input,
                                    const SparseVectorFilterProto& filter) {
  SparseDoubleVectorProto result;
  SparseVectorFilterPredicate predicate(filter);
  for (const auto [id, val] : MakeView(input)) {
    if (predicate.AcceptsAndUpdate(id, val)) {
      result.add_ids(id);
      result.add_values(val);
    }
  }
  return result;
}

absl::Status SetParameters(const SolveParametersProto& params,
                           EcosSolver& solver) {
  solver.set_verbose(params.enable_output());
  std::vector<std::string> errors;

  // If status is an error with code kInvalidArgument, appends the error to
  // `errors`, otherwise returns status.
  auto record_error = [&errors](const absl::Status status) -> absl::Status {
    if (!status.ok() && status.code() == absl::StatusCode::kInvalidArgument) {
      errors.push_back(std::string(status.message()));
      return absl::OkStatus();
    }
    RETURN_IF_ERROR(status) << "ECOS error";
    return absl::OkStatus();
  };
  if (params.has_iteration_limit()) {
    RETURN_IF_ERROR(
        record_error(solver.set_max_iterations(params.iteration_limit())));
  }
  if (params.has_absolute_gap_tolerance()) {
    RETURN_IF_ERROR(record_error(
        solver.set_absolute_tolerance(params.absolute_gap_tolerance())));
  }
  if (params.has_relative_gap_tolerance()) {
    RETURN_IF_ERROR(record_error(
        solver.set_relative_tolerance(params.relative_gap_tolerance())));
  }
  if (params.has_best_bound_limit()) {
    errors.push_back(UnsupportedParam("best_bound_limit"));
  }
  if (params.has_cutoff_limit()) {
    errors.push_back(UnsupportedParam("cutoff_limit"));
  }
  if (params.has_node_limit()) {
    errors.push_back(UnsupportedParam("node_limit"));
  }
  if (params.has_objective_limit()) {
    errors.push_back(UnsupportedParam("objective_limit"));
  }
  if (params.has_random_seed()) {
    errors.push_back(UnsupportedParam("random_seed"));
  }
  if (params.has_solution_limit()) {
    errors.push_back(UnsupportedParam("solution_limit"));
  }
  if (params.has_threads() && params.threads() != 1) {
    errors.push_back(UnsupportedParam("threads"));
  }
  if (params.has_time_limit()) {
    errors.push_back(UnsupportedParam("time_limit"));
  }
  if (params.cuts() != EMPHASIS_UNSPECIFIED) {
    errors.push_back(UnsupportedParam("cuts"));
  }
  // TODO(b/260616646): it appears EQUILIBRATION is on by default and can be
  // disabled, this is how ECOS does scaling?
  // https://github.com/embotech/ecos/blob/develop/include/ecos.h#L80
  if (params.scaling() != EMPHASIS_UNSPECIFIED) {
    errors.push_back(UnsupportedParam("scaling"));
  }
  if (params.heuristics() != EMPHASIS_UNSPECIFIED) {
    errors.push_back(UnsupportedParam("heuristics"));
  }
  if (params.presolve() != EMPHASIS_UNSPECIFIED) {
    errors.push_back(UnsupportedParam("presolve"));
  }
  if (params.lp_algorithm() != LP_ALGORITHM_UNSPECIFIED &&
      params.lp_algorithm() != LP_ALGORITHM_BARRIER) {
    errors.push_back(UnsupportedParam("lp_algorithm"));
  }
  if (!errors.empty()) {
    return absl::InvalidArgumentError(absl::StrJoin(errors, "; "));
  }
  return absl::OkStatus();
}

absl::StatusOr<std::pair<TerminationProto, ProblemStatusProto>>
BuildTermination(const EcosExitCode code) {
  ProblemStatusProto feasibility;
  feasibility.set_primal_status(FEASIBILITY_STATUS_UNDETERMINED);
  feasibility.set_dual_status(FEASIBILITY_STATUS_UNDETERMINED);
  std::string detail = absl::StrCat("ECOS exit code: ", ToString(code));
  switch (code) {
    case EcosExitCode::kOptimal:
      feasibility.set_primal_status(FEASIBILITY_STATUS_FEASIBLE);
      feasibility.set_dual_status(FEASIBILITY_STATUS_FEASIBLE);
      return std::make_pair(TerminateForReason(TERMINATION_REASON_OPTIMAL),
                            feasibility);
    case EcosExitCode::kPrimalInfeasible:
      feasibility.set_primal_status(FEASIBILITY_STATUS_INFEASIBLE);
      return std::make_pair(TerminateForReason(TERMINATION_REASON_INFEASIBLE),
                            feasibility);
    case EcosExitCode::kDualInfeasible:
      feasibility.set_dual_status(FEASIBILITY_STATUS_INFEASIBLE);
      return std::make_pair(
          TerminateForReason(TERMINATION_REASON_INFEASIBLE_OR_UNBOUNDED,
                             detail),
          feasibility);
    case EcosExitCode::kOptimalInaccurate:
    case EcosExitCode::kPrimalInfeasibleInaccurate:
    case EcosExitCode::kDualInfeasibleInaccurate:
      // TODO(b/211679884): improve FeasibilityStatusProto
      return std::make_pair(
          TerminateForReason(TERMINATION_REASON_IMPRECISE, detail),
          feasibility);
    case EcosExitCode::kMaxIterations:
      return std::make_pair(TerminateForLimit(LIMIT_ITERATION, false),
                            feasibility);
    case EcosExitCode::kNumerics:
    case EcosExitCode::kOutsideCone:
      return std::make_pair(
          TerminateForReason(TERMINATION_REASON_NUMERICAL_ERROR, detail),
          feasibility);
    case EcosExitCode::kInterrupted:
      return std::make_pair(TerminateForLimit(LIMIT_INTERRUPTED, false),
                            feasibility);
    case EcosExitCode::kUnknown:
      return absl::InternalError("ECOS terminated with exit code UNKNOWN.");
  }
  return absl::InternalError(
      absl::StrCat("unimplemented ECOS exit code: ", ToString(code)));
}

}  // namespace

EcosMathOptSolver::EcosMathOptSolver(const ModelProto& model)
    : bridge_(model) {}

absl::StatusOr<std::unique_ptr<SolverInterface>> EcosMathOptSolver::New(
    const ModelProto& model, const InitArgs& init_args) {
  RETURN_IF_ERROR(ModelIsSupported(model, kEcosSupportedStructures, "ECOS"));
  return absl::WrapUnique(new EcosMathOptSolver(model));
}

absl::StatusOr<SolveResultProto> EcosMathOptSolver::Solve(
    const SolveParametersProto& parameters,
    const ModelSolveParametersProto& model_parameters,
    MessageCallback message_cb,
    const CallbackRegistrationProto& callback_registration, Callback cb,
    SolveInterrupter* interrupter) {
  const absl::Time start = absl::Now();
  if (message_cb != nullptr) {
    return absl::InvalidArgumentError(internal::kMessageCallbackNotSupported);
  }
  RETURN_IF_ERROR(CheckRegisteredCallbackEvents(callback_registration,
                                                /*supported_events=*/{}));
  RETURN_IF_ERROR(bridge_.ListInvertedBounds().ToStatus());
  EcosSolver solver;
  RETURN_IF_ERROR(solver.Init(bridge_.ecos_instance()));

  RETURN_IF_ERROR(SetParameters(parameters, solver));
  EcosExitCode ecos_exit = solver.Solve();
  SolveResultProto result;
  ASSIGN_OR_RETURN((auto [termination, feasibility]),
                   BuildTermination(ecos_exit));
  *result.mutable_termination() = std::move(termination);
  *result.mutable_solve_stats()->mutable_problem_status() =
      std::move(feasibility);
  // TODO(b/260616646): extract suboptimal solutions, dual solutions, and rays.
  if (ecos_exit == EcosExitCode::kOptimal) {
    SolutionProto& solution = *result.add_solutions();
    const Eigen::VectorXd primal_solution = solver.primal_solution();
    *solution.mutable_primal_solution() =
        bridge_.RecoverPrimalSolution(primal_solution);
    if (model_parameters.has_variable_values_filter()) {
      *solution.mutable_primal_solution()->mutable_variable_values() =
          ApplyFilter(solution.primal_solution().variable_values(),
                      model_parameters.variable_values_filter());
    }
    *solution.mutable_dual_solution() = bridge_.RecoverDualSolution(
        solver.equality_dual_solution(), solver.cone_dual_solution());
    if (model_parameters.has_reduced_costs_filter()) {
      *solution.mutable_dual_solution()->mutable_reduced_costs() =
          ApplyFilter(solution.dual_solution().reduced_costs(),
                      model_parameters.reduced_costs_filter());
    }
    if (model_parameters.has_dual_values_filter()) {
      *solution.mutable_dual_solution()->mutable_dual_values() =
          ApplyFilter(solution.dual_solution().dual_values(),
                      model_parameters.dual_values_filter());
    }

    const double obj = solution.primal_solution().objective_value();
    solution.mutable_dual_solution()->set_objective_value(obj);
    result.mutable_solve_stats()->set_best_primal_bound(obj);

    result.mutable_solve_stats()->set_best_dual_bound(obj);
  } else {
    if (bridge_.is_maximize()) {
      result.mutable_solve_stats()->set_best_primal_bound(-kInf);
      result.mutable_solve_stats()->set_best_dual_bound(kInf);
    } else {
      result.mutable_solve_stats()->set_best_primal_bound(kInf);
      result.mutable_solve_stats()->set_best_dual_bound(-kInf);
    }
  }
  ASSIGN_OR_RETURN(*result.mutable_solve_stats()->mutable_solve_time(),
                   util_time::EncodeGoogleApiProto(absl::Now() - start));
  return result;
}

absl::StatusOr<bool> EcosMathOptSolver::Update(
    const ModelUpdateProto& model_update) {
  return false;
}

MATH_OPT_REGISTER_SOLVER(SOLVER_TYPE_ECOS, EcosMathOptSolver::New)

}  // namespace operations_research::math_opt
