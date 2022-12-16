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

// Command line flags for configuring the MathOpt Solve function.
//
// This is best used in launchers for benchmarks/experiments, called directly
// from the file containing main(). Generally you should avoid defining flags in
// libraries (or linking to this target from a cc_library), see go/totw/45.
//
// The flags are all prefixed with "mo_" (for MathOpt) to try and reduce the
// chance of a flag collision, since this is a library.
//
// Note: this file will not compile with lite protos.
#ifndef OR_TOOLS_MATH_OPT_CPP_FLAGS_H_
#define OR_TOOLS_MATH_OPT_CPP_FLAGS_H_

#include <optional>

#include "absl/flags/declare.h"
#include "absl/time/time.h"
#include "ortools/math_opt/cpp/math_opt.h"
#include "ortools/util/proto_flag.h"

namespace operations_research::math_opt {

// Returns the solver from flag mo_solver_type or default solver if the flag is
// not set.
//
// E.g. for gurobi, use the flag --mo_solver_type=gurobi.
//
// Use from (near) main(), avoid flags in libraries (see go/totw/45).
SolverType SolverTypeFromFlags(SolverType default_solver);

// Returns the SolveParameter default_params as configured by several flags.
//
// Individual fields can be set, overwriting their value in default_params:
//   --mo_time_limit=30s
//   --mo_enable_output
// The entire SolveParameters object can be replaced by a text format
// SolveParametersProto:
//   --mo_solve_parameters='enable_output: true relative_gap_tolerance 0.1'
// Solver specific parameters can also be overwritten with text protos:
//   --mo_glop_parameters='refactorization_threshold: 1e-7'
//
// When --mo_solve_parameters and field specific flags are both set (e.g.
// --mo_time_limit), the field specific flags win.
//
// Use from (near) main(), avoid flags in libraries (see go/totw/45).
SolveParameters SolveParametersFromFlags(
    const SolveParameters& default_params = {});

}  // namespace operations_research::math_opt

////////////////////////////////////////////////////////////////////////////////
// Implementation Details
////////////////////////////////////////////////////////////////////////////////

// The flag declarations below are visible for testing purposes only, do not
// depend on these directly, use SolverTypeFromFlags() or
// SolveParametersFromFlags() instead.
ABSL_DECLARE_FLAG(std::optional<operations_research::math_opt::SolverType>,
                  mo_solver_type);
ABSL_DECLARE_FLAG(std::optional<bool>, mo_enable_output);
ABSL_DECLARE_FLAG(std::optional<absl::Duration>, mo_time_limit);
ABSL_DECLARE_FLAG(std::optional<int64_t>, mo_iteration_limit);
ABSL_DECLARE_FLAG(std::optional<int64_t>, mo_node_limit);
ABSL_DECLARE_FLAG(std::optional<double>, mo_cutoff_limit);
ABSL_DECLARE_FLAG(std::optional<double>, mo_objective_limit);
ABSL_DECLARE_FLAG(std::optional<double>, mo_best_bound_limit);
ABSL_DECLARE_FLAG(std::optional<int32_t>, mo_solution_limit);
ABSL_DECLARE_FLAG(std::optional<int32_t>, mo_threads);
ABSL_DECLARE_FLAG(std::optional<int32_t>, mo_random_seed);
ABSL_DECLARE_FLAG(std::optional<double>, mo_absolute_gap_tolerance);
ABSL_DECLARE_FLAG(std::optional<double>, mo_relative_gap_tolerance);
ABSL_DECLARE_FLAG(std::optional<operations_research::math_opt::LPAlgorithm>,
                  mo_lp_algorithm);
ABSL_DECLARE_FLAG(std::optional<operations_research::math_opt::Emphasis>,
                  mo_presolve);
ABSL_DECLARE_FLAG(std::optional<operations_research::math_opt::Emphasis>,
                  mo_cuts);
ABSL_DECLARE_FLAG(std::optional<operations_research::math_opt::Emphasis>,
                  mo_heuristics);
ABSL_DECLARE_FLAG(std::optional<operations_research::math_opt::Emphasis>,
                  mo_scaling);

ABSL_DECLARE_FLAG(std::optional<operations_research::math_opt::SolveParameters>,
                  mo_solve_parameters);
ABSL_DECLARE_FLAG(
    std::optional<
        operations_research::ProtoFlag<operations_research::GScipParameters>>,
    mo_gscip_parameters);
ABSL_DECLARE_FLAG(std::optional<operations_research::ProtoFlag<
                      operations_research::math_opt::GurobiParametersProto>>,
                  mo_gurobi_parameters);
ABSL_DECLARE_FLAG(std::optional<operations_research::ProtoFlag<
                      operations_research::glop::GlopParameters>>,
                  mo_glop_parameters);
ABSL_DECLARE_FLAG(std::optional<operations_research::ProtoFlag<
                      operations_research::sat::SatParameters>>,
                  mo_cp_sat_parameters);
// MOE: begin_strip
ABSL_DECLARE_FLAG(
    std::optional<operations_research::ProtoFlag<
        operations_research::pdlp::PrimalDualHybridGradientParams>>,
    mo_pdlp_parameters);
ABSL_DECLARE_FLAG(std::optional<operations_research::ProtoFlag<
                      operations_research::math_opt::OsqpSettingsProto>>,
                  mo_osqp_parameters);
// MOE: end_strip

#endif  // OR_TOOLS_MATH_OPT_CPP_FLAGS_H_
