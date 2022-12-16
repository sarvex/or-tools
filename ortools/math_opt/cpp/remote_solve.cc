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

#include "ortools/math_opt/cpp/remote_solve.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/time/time.h"
#include "net/loadshedding/proto/request_qos.pb.h"
#include "net/loadshedding/proto/request_qos_overrides.pb.h"
#include "net/rpc2/contrib/util/smart-service.pb.h"
#include "net/rpc2/contrib/util/smart-stub.h"
#include "ortools/base/status_macros.h"
#include "ortools/math_opt/cpp/model.h"
#include "ortools/math_opt/cpp/model_solve_parameters.h"
#include "ortools/math_opt/cpp/solve_arguments.h"
#include "ortools/math_opt/cpp/solve_result.h"
#include "ortools/math_opt/model.pb.h"
#include "ortools/math_opt/rpc.pb.h"
#include "ortools/util/status_macros.h"
#include "production/rpc/stubs/proto/stub_configuration.pb.h"
#include "production/rpc/stubs/public/stubs_client_channel.h"

ABSL_FLAG(production::rpc::stubs::OptionalCanonicalStub,
          math_opt_solve_service_spec,
          {operations_research::math_opt::DefaultStubOptions().proto_options},
          "Stub options for MathOpt SolveService.");

namespace operations_research::math_opt {

using net::loadshedding::RequestQoS;
using production::rpc::stubs::CanonicalStubOptions;
using production::rpc::stubs::NewStubsClientChannel;
using production::rpc::stubs::OptionalCanonicalStub;
using production::rpc::stubs::proto::CanonicalStubConfiguration;

CanonicalStubOptions DefaultStubOptions() {
  CanonicalStubConfiguration config;
  config.set_server_name("SolveService");
  rpc2::contrib::SmartService* smart_service = config.mutable_smart_service();
  smart_service->add_target(
      "blade:operations_research.math_opt.solveservice-prod");
  smart_service->mutable_gslb_channel_options()->set_use_session_balancing(
      true);
  config.set_traffic_type(CanonicalStubConfiguration::BATCH);
  config.set_enable_longest_allowed_deadline_seconds(false);
  net::loadshedding::RequestQoSOverrides* qos_overrides =
      config.mutable_request_qos_overrides();
  qos_overrides->set_ceiling_criticality(RequestQoS::CRITICAL_PLUS);
  qos_overrides->set_default_criticality(RequestQoS::SHEDDABLE_PLUS);
  CanonicalStubOptions result;
  result.proto_options = std::move(config);
  return result;
}

absl::StatusOr<CanonicalStubOptions> StubOptionsFromFlags() {
  OptionalCanonicalStub optional_stub_config =
      absl::GetFlag(FLAGS_math_opt_solve_service_spec);
  if (!optional_stub_config.configuration.has_value()) {
    return absl::FailedPreconditionError(
        "Could not parse command line flag math_opt_solve_service_spec (should "
        "be a CanonicalStubConfiguration text proto)");
  }
  CanonicalStubOptions result;
  result.proto_options = optional_stub_config.configuration.value();
  return result;
}

absl::StatusOr<std::unique_ptr<SolveService>> SolveServerStub(
    const CanonicalStubOptions& options, const absl::Duration timeout) {
  ASSIGN_OR_RETURN(rpc2::contrib::ChannelPtr ulss_channel,
                   NewStubsClientChannel(options));
  std::unique_ptr<SolveService> result = SolveService::NewStub(ulss_channel);
  if (timeout > absl::ZeroDuration()) {
    if (!production::rpc::stubs::WaitUntilReachableWithTimeout(
            result->rpc_channel(), timeout)) {
      return absl::UnavailableError(
          "Cannot reach operations_research.math_opt.SolveService");
    }
  }
  return result;
}

absl::StatusOr<SolveResultAndLogs> RemoteSolve(SolveService& stub,
                                               const Model& model,
                                               const SolverType solver_type,
                                               const RemoteSolveArgs& args) {
  RETURN_IF_ERROR(args.model_parameters.CheckModelStorage(model.storage()))
      << "invalid model_parameters";

  SolveRequest request;
  *request.mutable_model() = model.ExportModel();
  request.set_solver_type(EnumToProto(solver_type));
  *request.mutable_parameters() = args.parameters.Proto();
  *request.mutable_model_parameters() = args.model_parameters.Proto();
  SolveResponse response;
  RETURN_IF_ERROR(stub.Solve(request, response));
  ASSIGN_OR_RETURN(auto solve_result,
                   SolveResult::FromProto(model.storage(), response.result()),
                   _ << "could not read SolveResultProto from server");
  std::vector<std::string> messages(response.messages().begin(),
                                    response.messages().end());
  return SolveResultAndLogs{.solve_result = std::move(solve_result),
                            .messages = std::move(messages)};
}

}  // namespace operations_research::math_opt
