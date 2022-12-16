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

#include "ortools/math_opt/solvers/ecos/ecos_bridge.h"

#include <cmath>
#include <vector>

#include "Eigen/Core"
#include "Eigen/SparseCore"
#include "absl/algorithm/container.h"
#include "ortools/math_opt/core/inverted_bounds.h"
#include "ortools/math_opt/core/sparse_vector_view.h"

namespace operations_research::math_opt {
namespace {

template <typename V>
google::protobuf::RepeatedField<int64_t> SortedKeys(
    const absl::flat_hash_map<int64_t, V>& int_map) {
  google::protobuf::RepeatedField<int64_t> result;
  result.Reserve(static_cast<int>(int_map.size()));
  for (const auto& [key, unused] : int_map) {
    result.Add(key);
  }
  absl::c_sort(result);
  return result;
}

// A helper to build an EcosInstance where variables and constraints can be
// added one at a time. Only the minimum functionality needed for EcosBridge is
// implemented.
class EcosBuilder {
 public:
  using idxint = EcosInstance::idxint;

  idxint add_var();
  idxint add_eq_constraint(double rhs);
  idxint add_le_constraint(double rhs);

  void set_objective_coefficient(idxint var, double obj_coef);

  // Note: if called multiple times for the same (eq_constraint, var) pair,
  // the values of `coef` are added together.
  void add_eq_term(idxint eq_constraint, idxint var, double coef);

  // Note: if called multiple times for the same (le_constraint, var) pair,
  // the values of `coef` are added together.
  void add_le_term(idxint le_constraint, idxint var, double coef);

  idxint num_vars() const;
  idxint num_eq_constraints() const;
  idxint num_le_constraints() const;

  EcosInstance Build() const;

 private:
  using Triplet = Eigen::Triplet<double, idxint>;

  std::vector<Triplet> cone_matrix_;
  std::vector<Triplet> equality_matrix_;
  std::vector<double> objective_;
  std::vector<double> cone_rhs_;
  std::vector<double> eq_rhs_;
};

EcosInstance EcosBuilder::Build() const {
  EcosInstance result;
  result.cone_matrix.resize(num_le_constraints(), num_vars());
  result.cone_matrix.setFromTriplets(cone_matrix_.begin(), cone_matrix_.end());
  result.equality_matrix.resize(num_eq_constraints(), num_vars());
  result.equality_matrix.setFromTriplets(equality_matrix_.begin(),
                                         equality_matrix_.end());
  result.objective_vector = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(
      objective_.data(), objective_.size());
  result.cone_constant = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(
      cone_rhs_.data(), cone_rhs_.size());
  result.equality_constant =
      Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(eq_rhs_.data(),
                                                          eq_rhs_.size());
  result.orthant_dimension = static_cast<int>(num_le_constraints());
  return result;
}

EcosBuilder::idxint EcosBuilder::add_var() {
  idxint result = objective_.size();
  objective_.push_back(0.0);
  return result;
}

void EcosBuilder::set_objective_coefficient(idxint var, double obj_coef) {
  objective_[var] = obj_coef;
}

EcosBuilder::idxint EcosBuilder::add_eq_constraint(double rhs) {
  idxint result = eq_rhs_.size();
  eq_rhs_.push_back(rhs);
  return result;
}
EcosBuilder::idxint EcosBuilder::add_le_constraint(double rhs) {
  idxint result = cone_rhs_.size();
  cone_rhs_.push_back(rhs);
  return result;
}

void EcosBuilder::add_eq_term(idxint eq_constraint, idxint var, double coef) {
  equality_matrix_.push_back({eq_constraint, var, coef});
}
void EcosBuilder::add_le_term(idxint le_constraint, idxint var, double coef) {
  cone_matrix_.push_back({le_constraint, var, coef});
}

EcosBuilder::idxint EcosBuilder::num_vars() const { return objective_.size(); }

EcosBuilder::idxint EcosBuilder::num_eq_constraints() const {
  return eq_rhs_.size();
}
EcosBuilder::idxint EcosBuilder::num_le_constraints() const {
  return cone_rhs_.size();
}

}  // namespace

EcosBridge::BoundType EcosBridge::bound_type(double lb, double ub) {
  const bool lb_finite = std::isfinite(lb);
  const bool ub_finite = std::isfinite(ub);
  if (lb == ub) {
    return BoundType::kEq;
  }
  if (lb_finite && ub_finite) {
    return BoundType::kRanged;
  }
  if (lb_finite) {
    return BoundType::kLb;
  }
  if (ub_finite) {
    return BoundType::kUb;
  }
  return BoundType::kFree;
}

EcosBridge::EcosBridge(const ModelProto& model_proto) {
  EcosBuilder ecos;
  is_maximize_ = model_proto.objective().maximize();
  objective_offset_ = model_proto.objective().offset();
  for (int i = 0; i < model_proto.variables().ids_size(); ++i) {
    const int64_t id = model_proto.variables().ids(i);
    VarData& var_data = variables_[id];
    var_data.lb = model_proto.variables().lower_bounds(i);
    var_data.ub = model_proto.variables().upper_bounds(i);
    var_data.ecos_var = ecos.add_var();
    // Add the constraint -x' <= -lb if needed.
    if (std::isfinite(var_data.lb)) {
      var_data.ecos_lb_cons = ecos.add_le_constraint(-var_data.lb);
      ecos.add_le_term(var_data.ecos_lb_cons, var_data.ecos_var, -1.0);
    }
    // Add the constraint x <= ub if needed.
    if (std::isfinite(var_data.ub)) {
      var_data.ecos_ub_cons = ecos.add_le_constraint(var_data.ub);
      ecos.add_le_term(var_data.ecos_ub_cons, var_data.ecos_var, 1.0);
    }
  }

  // Set up the objective
  for (const auto [var_id, obj] :
       MakeView(model_proto.objective().linear_coefficients())) {
    VarData& var_data = variables_[var_id];
    var_data.obj = obj;
    const double ecos_obj = is_maximize_ ? -obj : obj;
    ecos.set_objective_coefficient(var_data.ecos_var, ecos_obj);
  }

  const int mo_lin_cons = model_proto.linear_constraints().ids_size();
  for (int i = 0; i < mo_lin_cons; ++i) {
    const int64_t id = model_proto.linear_constraints().ids(i);
    LinConData& lin_con_data = linear_constraints_[id];
    lin_con_data.lb = model_proto.linear_constraints().lower_bounds(i);
    lin_con_data.ub = model_proto.linear_constraints().upper_bounds(i);
    switch (lin_con_data.bound_type()) {
      case BoundType::kFree:
        // We will not add the constraint to ecos, it doesn't do anything!
        break;
      case BoundType::kLb:
        lin_con_data.ecos_cone_cons = ecos.add_le_constraint(-lin_con_data.lb);
        break;
      case BoundType::kUb:
        lin_con_data.ecos_cone_cons = ecos.add_le_constraint(lin_con_data.ub);
        break;
      case BoundType::kEq:
        lin_con_data.ecos_eq_cons = ecos.add_eq_constraint(lin_con_data.lb);
        break;
      case BoundType::kRanged:
        lin_con_data.ecos_eq_cons = ecos.add_eq_constraint(0.0);
        lin_con_data.ecos_aux_var = ecos.add_var();
        ecos.add_eq_term(lin_con_data.ecos_eq_cons, lin_con_data.ecos_aux_var,
                         -1.0);
        lin_con_data.ecos_aux_lb_cons =
            ecos.add_le_constraint(-lin_con_data.lb);
        ecos.add_le_term(lin_con_data.ecos_aux_lb_cons,
                         lin_con_data.ecos_aux_var, -1.0);
        lin_con_data.ecos_aux_ub_cons = ecos.add_le_constraint(lin_con_data.ub);
        ecos.add_le_term(lin_con_data.ecos_aux_ub_cons,
                         lin_con_data.ecos_aux_var, 1.0);
        break;
    }
  }

  for (int i = 0; i < model_proto.linear_constraint_matrix().row_ids_size();
       ++i) {
    const double coef = model_proto.linear_constraint_matrix().coefficients(i);
    const int64_t var_id = model_proto.linear_constraint_matrix().column_ids(i);
    const int64_t ecos_var = variables_.at(var_id).ecos_var;
    const int64_t lin_con_id =
        model_proto.linear_constraint_matrix().row_ids(i);
    const LinConData& lin_con = linear_constraints_.at(lin_con_id);
    switch (lin_con.bound_type()) {
      case BoundType::kFree:
        break;
      case BoundType::kUb:
        ecos.add_le_term(lin_con.ecos_cone_cons, ecos_var, coef);
        break;
      case BoundType::kLb:
        ecos.add_le_term(lin_con.ecos_cone_cons, ecos_var, -coef);
        break;
      case BoundType::kEq:
      case BoundType::kRanged:
        ecos.add_eq_term(lin_con.ecos_eq_cons, ecos_var, coef);
        break;
    }
  }
  ecos_instance_ = ecos.Build();
}

PrimalSolutionProto EcosBridge::RecoverPrimalSolution(
    const Eigen::VectorXd& ecos_solution) const {
  CHECK_EQ(ecos_solution.size(), ecos_instance_.equality_matrix.cols());
  PrimalSolutionProto result;
  result.set_feasibility_status(SOLUTION_STATUS_FEASIBLE);

  // Fill the variable ids and put them sorted order.
  *result.mutable_variable_values()->mutable_ids() = SortedKeys(variables_);

  // Fill in the variable values and compute the objective value.
  auto& var_values = *result.mutable_variable_values()->mutable_values();
  double obj = objective_offset_;
  for (const int64_t var_id : result.variable_values().ids()) {
    const VarData& var_data = variables_.at(var_id);
    const double var_val = ecos_solution[var_data.ecos_var];
    var_values.Add(var_val);
    obj += var_data.obj * var_val;
  }
  result.set_objective_value(obj);

  return result;
}

DualSolutionProto EcosBridge::RecoverDualSolution(
    const Eigen::VectorXd& equals_duals,
    const Eigen::VectorXd& cone_duals) const {
  CHECK_EQ(equals_duals.size(), ecos_instance_.equality_matrix.rows());
  CHECK_EQ(cone_duals.size(), ecos_instance_.cone_matrix.rows());
  DualSolutionProto result;
  result.set_feasibility_status(SOLUTION_STATUS_FEASIBLE);

  // Fill the variable/constraint ids and put them sorted order.
  *result.mutable_reduced_costs()->mutable_ids() = SortedKeys(variables_);
  *result.mutable_dual_values()->mutable_ids() =
      SortedKeys(linear_constraints_);

  // Fill in reduced costs and dual variables
  auto& reduced_cost_values = *result.mutable_reduced_costs()->mutable_values();
  for (const int64_t var_id : result.reduced_costs().ids()) {
    const VarData& var_data = variables_.at(var_id);
    double rc;
    switch (var_data.bound_type()) {
      case BoundType::kFree:
        rc = 0.0;
        break;
      case BoundType::kLb:
        rc = cone_duals[var_data.ecos_lb_cons];
        break;
      case BoundType::kUb:
        rc = -cone_duals[var_data.ecos_ub_cons];
        break;
      case BoundType::kRanged:
      case BoundType::kEq:
        rc = -cone_duals[var_data.ecos_ub_cons] +
             cone_duals[var_data.ecos_lb_cons];
        break;
    }
    if (is_maximize_) {
      rc = -rc;
    }
    reduced_cost_values.Add(rc);
  }
  auto& dual_values = *result.mutable_dual_values()->mutable_values();
  for (const int64_t lin_con_id : result.dual_values().ids()) {
    const LinConData& lin_con_data = linear_constraints_.at(lin_con_id);
    double dual_value;
    switch (lin_con_data.bound_type()) {
      case BoundType::kFree:
        dual_value = 0.0;
        break;
      case BoundType::kLb:
        dual_value = cone_duals[lin_con_data.ecos_cone_cons];
        break;
      case BoundType::kUb:
        dual_value = -cone_duals[lin_con_data.ecos_cone_cons];
        break;
      case BoundType::kEq:
        dual_value = -equals_duals[lin_con_data.ecos_eq_cons];
        break;
      case BoundType::kRanged:
        dual_value = -cone_duals[lin_con_data.ecos_aux_ub_cons] +
                     cone_duals[lin_con_data.ecos_aux_lb_cons];
        break;
    }
    if (is_maximize_) {
      dual_value = -dual_value;
    }
    dual_values.Add(dual_value);
  }
  return result;
}

InvertedBounds EcosBridge::ListInvertedBounds() const {
  InvertedBounds inverted_bounds;
  for (const auto& [var_id, var_data] : variables_) {
    if (var_data.lb > var_data.ub) {
      inverted_bounds.variables.push_back(var_id);
    }
  }
  absl::c_sort(inverted_bounds.variables);
  for (const auto& [lin_con_id, lin_con_data] : linear_constraints_) {
    if (lin_con_data.lb > lin_con_data.ub) {
      inverted_bounds.linear_constraints.push_back(lin_con_id);
    }
  }
  absl::c_sort(inverted_bounds.linear_constraints);
  return inverted_bounds;
}

}  // namespace operations_research::math_opt
