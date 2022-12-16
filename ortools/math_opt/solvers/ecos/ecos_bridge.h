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

#ifndef OR_TOOLS_MATH_OPT_SOLVERS_ECOS_ECOS_BRIDGE_H_
#define OR_TOOLS_MATH_OPT_SOLVERS_ECOS_ECOS_BRIDGE_H_

#include <cstdint>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "ortools/convex/ecos/ecos.h"
#include "ortools/math_opt/core/inverted_bounds.h"
#include "ortools/math_opt/model.pb.h"
#include "ortools/math_opt/solution.pb.h"

namespace operations_research::math_opt {

// Transforms a MathOpt optimization model into an ECOS optimization model, and
// converts solutions between the formats.
//
// Note that ECOS only supports unbounded variables, = constraints, and <=
// constraints, so some auxiliary variable/constraints are required.
//
// Variable transformations: for each variable x in MathOpt, we have a variable
// x' in ECOS with x = x' and:
//  * x >= lb: add inequality constraint -x' <= -lb.
//  * x <= ub: add inequality constraint x' <= ub.
//  * lb <= x <= ub: add inequality constraints -x' <= -lb and x' <= ub.
//  * x free: no extra constraints.
//
// Objective transformations:
//  * For maximization, negates the objective and changes to min.
//  * Replaces sum_i c_i x_i by sum_i c_i x_i' to move to the new variables.
//  * Discards the offset (ECOS does not support an offset).
//
// Linear constraints transformations:
//  * a * x >= b: add -a * x' <= -b.
//  * a * x <= b: add a * x' <= b.
//  * a * x = b: add a * x' = b.
//  * lb <= a * x <= ub and lb != ub: add a * x' = z', -z' <= -lb, z' <= ub.
//  * unbounded linear constraints: do not add.
//
// See go/mathopt-dev-transformations#ecos-mapping for more details.
class EcosBridge {
 public:
  explicit EcosBridge(const ModelProto& model_proto);
  // Delete copy
  EcosBridge(const EcosBridge&) = delete;
  EcosBridge& operator=(const EcosBridge&) = delete;
  // Keep move
  EcosBridge(EcosBridge&&) = default;
  EcosBridge& operator=(EcosBridge&&) = default;

  // The equivalent ECOS model to the input MathOpt model.
  const EcosInstance& ecos_instance() const { return ecos_instance_; }

  // Converts a solution to the ECOS model back to a solution to the MathOpt
  // model.
  PrimalSolutionProto RecoverPrimalSolution(
      const Eigen::VectorXd& ecos_solution) const;

  // Converts a solution to the ECOS model back to a solution to the MathOpt
  // model. Note: we do not fill in the objective value here.
  DualSolutionProto RecoverDualSolution(
      const Eigen::VectorXd& equals_duals,
      const Eigen::VectorXd& cone_duals) const;

  // If the original MathOpt model was maximization.
  bool is_maximize() const { return is_maximize_; }

  // Returns the list of inconsistent upper and lower bounds from the MathOpt
  // model (lower bound > upper bound).
  InvertedBounds ListInvertedBounds() const;

 private:
  // Guard values for unused indices into the ECOS model.
  static constexpr int64_t kUnusedEcosIndex = -1;

  // Characterizes an interval bound [lb, ub]. We do not consider:
  //  * either lb or ub being NaN,
  //  * lb = +inf,
  //  * ub = -inf.
  // See EcosBridge::bound_type(double lb, double ub) for building this enum.
  enum class BoundType {
    // lb = -inf, ub = inf
    kFree,
    // lb = finite, ub = inf
    kLb,
    // lb = -inf, ub finite
    kUb,
    // lb and ub finite and unequal
    kRanged,
    // lb and ub finite and equal
    kEq,
  };

  // Contains the ECOS data for each MathOpt variable (and some properties of
  // the MathOpt variable).
  //
  // MathOpt variables may require reformulation and auxiliary variables and
  // constraints to model in ECOS, see the EcosBridge class description for the
  // rules.
  struct VarData {
    // The MathOpt variable lower bound.
    double lb = 0.0;

    // The MathOpt variable upper bound.
    double ub = 0.0;

    // The MathOpt variable objective coefficient.
    double obj = 0.0;

    // The ECOS variable index for the variable in ecos, always used.
    int64_t ecos_var = kUnusedEcosIndex;
    // index for the constraint -x <= -lb, used only when lb is finite.
    int64_t ecos_lb_cons = kUnusedEcosIndex;
    // index for the constraint x <= ub, used only when ub is finite.
    int64_t ecos_ub_cons = kUnusedEcosIndex;

    // A summary of the upper and lower bounds for the MathOpt variable.
    //
    // Note: EcosBridge treats variables with bound type kEq and kRanged
    // identically.
    BoundType bound_type() const { return EcosBridge::bound_type(lb, ub); }
  };

  // Contains the ECOS data for each MathOpt linear constraint (and some
  // properties of the MathOpt constraint)..
  //
  // MathOpt constraints may require reformulation and auxiliary variables and
  // constraints to model in ECOS, see the EcosBridge class description for the
  // rules.
  struct LinConData {
    // The MathOpt lower bound for this constraint.
    double lb = 0.0;
    // The MathOpt upper bound for this constraint.
    double ub = 0.0;

    // The ECOS equality constraint index for this constraint, used when
    // bound_type() is kRanged or kEq.
    int64_t ecos_eq_cons = kUnusedEcosIndex;

    // The ECOS cone constraint index for this constraint, used when
    // bound_type() is kLb or kUb.
    int64_t ecos_cone_cons = kUnusedEcosIndex;

    // For bound_type() kRanged, the ECOS variable index of the auxiliary
    // variable z.
    int64_t ecos_aux_var = kUnusedEcosIndex;

    // For bound_type() kRanged, the ECOS cone constraint index for the
    // auxiliary constraint -z <= -lb.
    int64_t ecos_aux_lb_cons = kUnusedEcosIndex;

    // For bound_type() kRanged, the ecos cone constraint index for the
    // auxiliary constraint z <= ub.
    int64_t ecos_aux_ub_cons = kUnusedEcosIndex;

    // A summary of the upper and lower bounds for the MathOpt constraint.
    BoundType bound_type() const { return EcosBridge::bound_type(lb, ub); }
  };

  static BoundType bound_type(double lb, double ub);

  EcosInstance ecos_instance_;
  bool is_maximize_ = false;
  double objective_offset_ = 0.0;
  absl::flat_hash_map<int64_t, VarData> variables_;
  absl::flat_hash_map<int64_t, LinConData> linear_constraints_;
};

}  // namespace operations_research::math_opt

#endif  // OR_TOOLS_MATH_OPT_SOLVERS_ECOS_ECOS_BRIDGE_H_
