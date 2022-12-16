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

#include "ortools/math_opt/cpp/flags.h"

#include <optional>
#include <string>

#include "absl/flags/flag.h"
#include "absl/time/time.h"
#include "google/protobuf/io/tokenizer.h"
#include "ortools/math_opt/cpp/math_opt.h"

ABSL_FLAG(std::optional<operations_research::math_opt::SolverType>,
          mo_solver_type, std::nullopt,
          "If not null, the solver to use, e.g. gurobi, gscip, glop, see "
          "Enum<SolverType>::ToOptString for possible values.");

ABSL_FLAG(std::optional<bool>, mo_enable_output, std::nullopt,
          "If not null, sets SolveParameters.enable_output.");

ABSL_FLAG(std::optional<absl::Duration>, mo_time_limit, std::nullopt,
          "If not null, sets SolveParameters.time_limit.");

ABSL_FLAG(std::optional<int64_t>, mo_iteration_limit, std::nullopt,
          "If not null, sets SolveParameters.iteration_limit.");

ABSL_FLAG(std::optional<int64_t>, mo_node_limit, std::nullopt,
          "If not null, sets SolveParameters.node_limit.");

ABSL_FLAG(std::optional<double>, mo_cutoff_limit, std::nullopt,
          "If not null, sets SolveParameters.cutoff_limit.");

ABSL_FLAG(std::optional<double>, mo_objective_limit, std::nullopt,
          "If not null, sets SolveParameters.objective_limit.");

ABSL_FLAG(std::optional<double>, mo_best_bound_limit, std::nullopt,
          "If not null, sets SolveParameters.best_bound_limit.");

ABSL_FLAG(std::optional<int32_t>, mo_solution_limit, std::nullopt,
          "If not null, sets SolveParameters.solution_limit.");

ABSL_FLAG(std::optional<int32_t>, mo_threads, std::nullopt,
          "If not null, sets SolveParameters.threads.");

ABSL_FLAG(std::optional<int32_t>, mo_random_seed, std::nullopt,
          "If not null, sets SolveParameters.random_seed.");

ABSL_FLAG(std::optional<double>, mo_absolute_gap_tolerance, std::nullopt,
          "If not null, sets SolveParameters.absolute_gap_tolerance.");

ABSL_FLAG(std::optional<double>, mo_relative_gap_tolerance, std::nullopt,
          "If not null, sets SolveParameters.relative_gap_tolerance.");

ABSL_FLAG(std::optional<int32_t>, mo_solution_pool_size, std::nullopt,
          "If not null, sets SolveParameters.solution_pool_size.");

ABSL_FLAG(std::optional<operations_research::math_opt::LPAlgorithm>,
          mo_lp_algorithm, std::nullopt,
          "If not null, sets SolveParameters.lp_algorithm, see "
          "Enum<LPAlgorithm>::ToOptString for possible values.");

ABSL_FLAG(std::optional<operations_research::math_opt::Emphasis>, mo_presolve,
          std::nullopt,
          "If not null, sets SolveParameters.presolve, see "
          "Enum<Emphasis>::ToOptString for possible values.");

ABSL_FLAG(std::optional<operations_research::math_opt::Emphasis>, mo_cuts,
          std::nullopt,
          "If not null, sets SolveParameters.cuts, see "
          "Enum<Emphasis>::ToOptString for possible values.");

ABSL_FLAG(std::optional<operations_research::math_opt::Emphasis>, mo_heuristics,
          std::nullopt,
          "If not null, sets SolveParameters.heuristics, see "
          "Enum<Emphasis>::ToOptString for possible values.");

ABSL_FLAG(std::optional<operations_research::math_opt::Emphasis>, mo_scaling,
          std::nullopt,
          "If not null, sets SolveParameters.scaling, see "
          "Enum<Emphasis>::ToOptString for possible values.");

ABSL_FLAG(std::optional<operations_research::math_opt::SolveParameters>,
          mo_solve_parameters, std::nullopt,
          "If not null, the SolveParameters as given by a SolveParametersProto "
          "in text format, overwriting all defaults");

ABSL_FLAG(
    std::optional<
        operations_research::ProtoFlag<operations_research::GScipParameters>>,
    mo_gscip_parameters, std::nullopt,
    "If not null, sets SolveParameters.gscip as given by a "
    "GScipParameters proto in text format, overwriting all defaults.");
ABSL_FLAG(std::optional<operations_research::ProtoFlag<
              operations_research::math_opt::GurobiParametersProto>>,
          mo_gurobi_parameters, std::nullopt,
          "If not null, sets SolveParameters.gurobi_parameters as given by a "
          "GurobiParametersProto in text format, overwriting all defaults.");
ABSL_FLAG(std::optional<operations_research::ProtoFlag<
              operations_research::glop::GlopParameters>>,
          mo_glop_parameters, std::nullopt,
          "If not null, sets SolveParameters.glop as given by a "
          "glop::GlopParameters in text format, overwriting all defaults.");
ABSL_FLAG(std::optional<operations_research::ProtoFlag<
              operations_research::sat::SatParameters>>,
          mo_cp_sat_parameters, std::nullopt,
          "If not null, sets SolveParameters.cp_sat as given by a "
          "sat::SatParameters in text format, overwriting all defaults.");
// MOE: begin_strip
ABSL_FLAG(std::optional<operations_research::ProtoFlag<
              operations_research::pdlp::PrimalDualHybridGradientParams>>,
          mo_pdlp_parameters, std::nullopt,
          "If not null, sets SolveParameters.pdlp as given by a "
          "pdlp::PrimalDualHybridGradientParams in text format, overwriting "
          "all defaults.");
ABSL_FLAG(std::optional<operations_research::ProtoFlag<
              operations_research::math_opt::OsqpSettingsProto>>,
          mo_osqp_parameters, std::nullopt,
          "If not null, sets SolveParameters.osqp as given by a "
          "OsqpSettingsProto in text format, overwriting all defaults.");
// MOE:end_strip

namespace operations_research::math_opt {
namespace {

template <typename T>
T ValueOrDefault(const absl::Flag<std::optional<T>>& flag,
                 const T& default_value) {
  std::optional<T> flag_value = absl::GetFlag(flag);
  if (flag_value.has_value()) {
    return *flag_value;
  }
  return default_value;
}

template <typename T>
std::optional<T> ValueOrOptionalDefault(
    const absl::Flag<std::optional<T>>& flag,
    const std::optional<T>& default_value) {
  std::optional<T> flag_value = absl::GetFlag(flag);
  if (flag_value.has_value()) {
    return flag_value;
  }
  return default_value;
}

template <typename ProtoType>
void ReadProtoFlag(const absl::Flag<std::optional<ProtoFlag<ProtoType>>>& flag,
                   ProtoType& proto) {
  const std::optional<ProtoFlag<ProtoType>>& flag_value = absl::GetFlag(flag);
  if (flag_value.has_value()) {
    proto = flag_value->message;
  }
}

}  // namespace

SolverType SolverTypeFromFlags(const SolverType default_solver) {
  return ValueOrDefault<SolverType>(FLAGS_mo_solver_type, default_solver);
}

SolveParameters SolveParametersFromFlags(
    const SolveParameters& default_params) {
  SolveParameters result =
      ValueOrDefault(FLAGS_mo_solve_parameters, default_params);
  result.enable_output =
      ValueOrDefault(FLAGS_mo_enable_output, result.enable_output);
  result.time_limit = ValueOrDefault(FLAGS_mo_time_limit, result.time_limit);
  result.iteration_limit =
      ValueOrOptionalDefault(FLAGS_mo_iteration_limit, result.iteration_limit);
  result.node_limit =
      ValueOrOptionalDefault(FLAGS_mo_node_limit, result.node_limit);

  result.cutoff_limit =
      ValueOrOptionalDefault(FLAGS_mo_cutoff_limit, result.cutoff_limit);
  result.objective_limit =
      ValueOrOptionalDefault(FLAGS_mo_objective_limit, result.objective_limit);
  result.best_bound_limit = ValueOrOptionalDefault(FLAGS_mo_best_bound_limit,
                                                   result.best_bound_limit);
  result.solution_limit =
      ValueOrOptionalDefault(FLAGS_mo_solution_limit, result.solution_limit);
  result.threads = ValueOrOptionalDefault(FLAGS_mo_threads, result.threads);
  result.random_seed =
      ValueOrOptionalDefault(FLAGS_mo_random_seed, result.random_seed);
  result.absolute_gap_tolerance = ValueOrOptionalDefault(
      FLAGS_mo_absolute_gap_tolerance, result.absolute_gap_tolerance);
  result.relative_gap_tolerance = ValueOrOptionalDefault(
      FLAGS_mo_relative_gap_tolerance, result.relative_gap_tolerance);
  result.solution_pool_size = ValueOrOptionalDefault(
      FLAGS_mo_solution_pool_size, result.solution_pool_size);

  result.lp_algorithm =
      ValueOrOptionalDefault(FLAGS_mo_lp_algorithm, result.lp_algorithm);
  result.presolve = ValueOrOptionalDefault(FLAGS_mo_presolve, result.presolve);
  result.cuts = ValueOrOptionalDefault(FLAGS_mo_cuts, result.cuts);
  result.heuristics =
      ValueOrOptionalDefault(FLAGS_mo_heuristics, result.heuristics);
  result.scaling = ValueOrOptionalDefault(FLAGS_mo_scaling, result.scaling);

  {
    const std::optional<ProtoFlag<GurobiParametersProto>> gurobi_params =
        absl::GetFlag(FLAGS_mo_gurobi_parameters);
    if (gurobi_params.has_value()) {
      result.gurobi = GurobiParameters::FromProto(gurobi_params->message);
    }
  }
  ReadProtoFlag(FLAGS_mo_gscip_parameters, result.gscip);
  ReadProtoFlag(FLAGS_mo_glop_parameters, result.glop);
  ReadProtoFlag(FLAGS_mo_cp_sat_parameters, result.cp_sat);

  // MOE: begin_strip
  ReadProtoFlag(FLAGS_mo_pdlp_parameters, result.pdlp);
  ReadProtoFlag(FLAGS_mo_osqp_parameters, result.osqp);
  // MOE:end_strip

  return result;
}

}  // namespace operations_research::math_opt
