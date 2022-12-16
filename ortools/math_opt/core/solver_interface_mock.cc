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

#include "ortools/math_opt/core/solver_interface_mock.h"

#include <atomic>
#include <functional>
#include <memory>

#include "absl/synchronization/mutex.h"
#include "ortools/base/logging.h"
#include "ortools/math_opt/core/solver_interface.h"
#include "ortools/math_opt/model.pb.h"
#include "ortools/math_opt/parameters.pb.h"

namespace operations_research {
namespace math_opt {

namespace {

// We use an atomic to have thread-safe generation of unique SolverTypeProto.
// The type of the atomic is `int` since `SolverTypeProto + 1` is not a
// SolverTypeProto.
std::atomic<int> next_unique_solver_type = SolverTypeProto_MAX + 1;

}  // namespace

SolverFactoryRegistration::SolverFactoryRegistration(
    SolverInterface::Factory factory)
    : caller_data_(std::make_shared<CallerData>(factory)),
      solver_type_(static_cast<SolverTypeProto>(next_unique_solver_type++)) {
  // We make a copy of the shared_ptr since we make a copy of the member field
  // in the capture below. It must be an identifier.
  const auto caller_data = caller_data_;

  // We register a lambda that shares the same CallerData instance at this class
  // using a shared_ptr.
  AllSolversRegistry::Instance()->Register(
      solver_type_, [caller_data](const ModelProto& model,
                                  const SolverInterface::InitArgs& init_args) {
        // We hold the lock during the call of the factory since we want to
        // delay the destruction of the registration during the call to the
        // factory (the factory may be invalid after the destruction).
        absl::MutexLock lock(&caller_data->mutex);
        CHECK(caller_data->factory != nullptr)
            << "Can't use this solver factory after the destruction of the "
               "SolverFactoryRegistration!";
        return caller_data->factory(model, init_args);
      });
}

SolverFactoryRegistration::~SolverFactoryRegistration() {
  absl::MutexLock lock(&caller_data_->mutex);
  caller_data_->factory = nullptr;
}

SolverFactoryRegistration::CallerData::CallerData(
    SolverInterface::Factory factory)
    : factory(ABSL_DIE_IF_NULL(factory)) {}

}  // namespace math_opt
}  // namespace operations_research
