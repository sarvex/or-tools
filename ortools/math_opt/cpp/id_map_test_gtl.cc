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

// This file contains the tests that validates that GTL map functions works
// properly with the IdMap.
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "ortools/base/logging.h"
#include "ortools/base/map_util.h"
#include "ortools/math_opt/cpp/id_map_test.h"
#include "ortools/math_opt/cpp/variable_and_expressions.h"

namespace operations_research {
namespace math_opt {
namespace {

using ::testing::AnyOf;
using ::testing::Pair;
using ::testing::UnorderedElementsAre;

TEST_F(SimpleIdMapTest, FindOrDieConst) {
  const VariableMap<double>& ref = vmap_;
  const double& d = gtl::FindOrDie(ref, a_);
  EXPECT_EQ(d, 3.0);

  // Check that we have a valid reference on the underlying storage.
  vmap_.at(a_) += 1.0;
  EXPECT_EQ(d, 4.0);
}

TEST_F(SimpleIdMapTest, FindOrDie) {
  // Make sure we use the non const version of FindOrDie.
  VariableMap<double>& ref = vmap_;
  double& d = gtl::FindOrDie(ref, a_);
  EXPECT_EQ(d, 3.0);

  d = -d;
  EXPECT_EQ(vmap_.at(a_), -3.0);
}

TEST_F(SimpleIdMapTest, FindWithDefault) {
  {
    const double& d = gtl::FindWithDefault(vmap_, a_);
    CHECK_EQ(d, 3.0);

    vmap_.at(a_) += 1.0;
    CHECK_EQ(d, 4.0);
  }
  {
    const double& d = gtl::FindWithDefault(vmap_, b_);
    CHECK_EQ(d, 0.0);
  }
}

TEST_F(SimpleIdMapTest, FindOrNullConst) {
  const VariableMap<double>& ref = vmap_;
  const double* const d = gtl::FindOrNull(ref, a_);
  EXPECT_EQ(*d, 3.0);

  // Check that we have a valid reference on the underlying storage.
  vmap_.at(a_) += 1.0;
  EXPECT_EQ(*d, 4.0);
}

TEST_F(SimpleIdMapTest, FindOrNull) {
  // Make sure we use the non const version of FindOrNull.
  VariableMap<double>& ref = vmap_;
  double* const d = gtl::FindOrNull(ref, a_);
  EXPECT_EQ(*d, 3.0);

  *d = -*d;
  EXPECT_EQ(vmap_.at(a_), -3.0);
}

TEST_F(SimpleIdMapTest, ContainsKeyValuePair) {
  EXPECT_TRUE(gtl::ContainsKeyValuePair(vmap_, a_, 3.0));
  EXPECT_FALSE(gtl::ContainsKeyValuePair(vmap_, a_, 4.0));
  EXPECT_FALSE(gtl::ContainsKeyValuePair(vmap_, b_, 3.0));
}

TEST_F(SimpleIdMapTest, InsertOrUpdate) {
  EXPECT_TRUE(gtl::InsertOrUpdate(&vmap_, {b_, 2.0}));
  EXPECT_FALSE(vmap_.insert_or_assign(a_, 1.0).second);
  EXPECT_THAT(
      vmap_, UnorderedElementsAre(Pair(a_, 1.0), Pair(b_, 2.0), Pair(c_, 5.0)));
}

TEST_F(SimpleIdMapTest, InsertAndDeleteExisting) {
  // Class tracking its deletion by modifying an external variable.
  class Tracker {
   public:
    explicit Tracker(bool& deleted) : deleted_(deleted) {}
    Tracker(const Tracker&) = delete;
    Tracker& operator=(const Tracker&) = delete;
    ~Tracker() { deleted_ = true; }

   private:
    bool& deleted_;
  };

  VariableMap<Tracker*> map;

  bool first_deleted = false;
  EXPECT_TRUE(map.emplace(a_, new Tracker(first_deleted)).second);

  bool second_deleted = false;
  EXPECT_FALSE(
      gtl::InsertAndDeleteExisting(&map, a_, new Tracker(second_deleted)));
  EXPECT_TRUE(first_deleted);
  EXPECT_FALSE(second_deleted);

  bool third_deleted = false;
  EXPECT_TRUE(
      gtl::InsertAndDeleteExisting(&map, b_, new Tracker(third_deleted)));
  EXPECT_TRUE(first_deleted);
  EXPECT_FALSE(second_deleted);
  EXPECT_FALSE(third_deleted);

  for (const auto pair : map) {
    delete pair.second;
  }
  map.clear();

  // Post-condition.
  EXPECT_TRUE(first_deleted);
  EXPECT_TRUE(second_deleted);
  EXPECT_TRUE(third_deleted);
}

TEST_F(SimpleIdMapTest, InsertIfNotPresent) {
  EXPECT_TRUE(gtl::InsertIfNotPresent(&vmap_, {b_, 2.0}));
  EXPECT_FALSE(gtl::InsertIfNotPresent(&vmap_, a_, 1.0));
  EXPECT_THAT(
      vmap_, UnorderedElementsAre(Pair(a_, 3.0), Pair(b_, 2.0), Pair(c_, 5.0)));
}

TEST_F(SimpleIdMapTest, InsertOrDie) {
  VariableMap<double> map;
  gtl::InsertOrDieNoPrint(&map, {b_, 2.0});
  gtl::InsertOrDie(&map, a_, 1.0);
  EXPECT_THAT(map, UnorderedElementsAre(Pair(a_, 1.0), Pair(b_, 2.0)));
}

TEST_F(SimpleIdMapDeathTest, InsertOrDie) {
  EXPECT_DEATH(gtl::InsertOrDieNoPrint(&vmap_, {a_, 2.0}), "duplicate value");
  EXPECT_DEATH(gtl::InsertOrDie(&vmap_, a_, 1.0), "duplicate key");
}

TEST_F(SimpleIdMapTest, InsertKeyOrDie) {
  VariableMap<double> map;
  gtl::InsertKeyOrDie(&map, a_);
  EXPECT_THAT(map, UnorderedElementsAre(Pair(a_, 0.0)));
}

TEST_F(SimpleIdMapDeathTest, InsertKeyOrDie) {
  EXPECT_DEATH(gtl::InsertKeyOrDie(&vmap_, a_), "duplicate key");
}

TEST_F(SimpleIdMapTest, LookupOrInsert) {
  double& a_value = gtl::LookupOrInsert(&vmap_, a_, 1.0);
  EXPECT_EQ(a_value, 3.0);

  double& b_value = gtl::LookupOrInsert(&vmap_, {b_, -2.0});
  EXPECT_EQ(b_value, -2.0);

  EXPECT_THAT(vmap_, UnorderedElementsAre(Pair(a_, 3.0), Pair(b_, -2.0),
                                          Pair(c_, 5.0)));

  // Note: at this point, a_value is no longer valid because inserting b may
  // have triggered a rehash.
  b_value += 1.0;
  EXPECT_EQ(vmap_.at(b_), -1.0);
}

TEST_F(SimpleIdMapTest, LookupOrInsertNew) {
  struct Value {
    Value(int x, int y) : x(x), y(y) {}
    int x;
    int y;
  };

  VariableMap<Value*> map;

  Value*& a_first_value = gtl::LookupOrInsertNew(&map, a_, 1, 2);
  EXPECT_TRUE(a_first_value == map.at(a_));
  EXPECT_EQ(map.at(a_)->x, 1);
  EXPECT_EQ(map.at(a_)->y, 2);

  Value*& a_second_value = gtl::LookupOrInsertNew(&map, a_, 3, 4);
  EXPECT_TRUE(a_second_value == map.at(a_));
  EXPECT_TRUE(a_second_value == a_first_value);
  EXPECT_EQ(map.at(a_)->x, 1);
  EXPECT_EQ(map.at(a_)->y, 2);

  for (const auto pair : map) {
    delete pair.second;
  }
}

TEST_F(SimpleIdMapTest, InsertOrReturnExisting) {
  {
    double* const ret = gtl::InsertOrReturnExisting(&vmap_, {b_, -2.0});
    EXPECT_TRUE(ret == nullptr);
  }

  double* const ret = gtl::InsertOrReturnExisting(&vmap_, a_, 1.0);
  ASSERT_TRUE(ret != nullptr);
  EXPECT_EQ(*ret, 3.0);

  EXPECT_THAT(vmap_, UnorderedElementsAre(Pair(a_, 3.0), Pair(b_, -2.0),
                                          Pair(c_, 5.0)));

  *ret += 1.0;
  EXPECT_EQ(vmap_.at(a_), 4.0);
}

TEST_F(SimpleIdMapTest, ReverseMap) {
  {
    const VariableMap<Variable> map{{a_, b_}, {b_, c_}};
    VariableMap<Variable> reversed;
    EXPECT_TRUE(gtl::ReverseMap(map, &reversed));
    EXPECT_THAT(reversed, UnorderedElementsAre(Pair(b_, a_), Pair(c_, b_)));
  }
  {
    const VariableMap<Variable> map{{a_, b_}, {c_, b_}};
    VariableMap<Variable> reversed;
    EXPECT_FALSE(gtl::ReverseMap(map, &reversed));
    EXPECT_THAT(reversed, AnyOf(UnorderedElementsAre(Pair(b_, a_)),
                                UnorderedElementsAre(Pair(b_, c_))));
  }
  {
    const VariableMap<int> map{{a_, 1}, {b_, 2}};
    absl::flat_hash_map<int, Variable> reversed;
    EXPECT_TRUE(gtl::ReverseMap(map, &reversed));
    EXPECT_THAT(reversed, UnorderedElementsAre(Pair(1, a_), Pair(2, b_)));
  }
  {
    const VariableMap<int> map{{a_, 1}, {c_, 1}};
    absl::flat_hash_map<int, Variable> reversed;
    EXPECT_FALSE(gtl::ReverseMap(map, &reversed));
    EXPECT_THAT(reversed, AnyOf(UnorderedElementsAre(Pair(1, a_)),
                                UnorderedElementsAre(Pair(1, c_))));
  }
  {
    const absl::flat_hash_map<int, Variable> map{{1, a_}, {2, b_}};
    VariableMap<int> reversed;
    EXPECT_TRUE(gtl::ReverseMap(map, &reversed));
    EXPECT_THAT(reversed, UnorderedElementsAre(Pair(a_, 1), Pair(b_, 2)));
  }
  {
    const absl::flat_hash_map<int, Variable> map{{1, a_}, {2, a_}};
    VariableMap<int> reversed;
    EXPECT_FALSE(gtl::ReverseMap(map, &reversed));
    EXPECT_THAT(reversed, AnyOf(UnorderedElementsAre(Pair(a_, 1)),
                                UnorderedElementsAre(Pair(a_, 2))));
  }
}

}  // namespace
}  // namespace math_opt
}  // namespace operations_research
