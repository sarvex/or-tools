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

// This file contains the tests that validates that GTL set functions work
// properly with the IdSet.
#include "gtest/gtest.h"
#include "ortools/math_opt/cpp/id_set.h"
#include "ortools/math_opt/cpp/id_set_test.h"

namespace operations_research {
namespace math_opt {
namespace {

TEST_F(SimpleIdSetTest, ContainsKey) {
  EXPECT_TRUE(vset_.contains(a_));
  EXPECT_FALSE(vset_.contains(b_));
}

}  // namespace
}  // namespace math_opt
}  // namespace operations_research
