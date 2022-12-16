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

// Binary used to test the Python `binary_testing` library.

#include <iostream>
#include <ostream>

#include "absl/flags/flag.h"
#include "ortools/base/init_google.h"

ABSL_FLAG(bool, fail, false, "make the binary fail");

int main(int argc, char* argv[]) {
  InitGoogle(argv[0], &argc, &argv, /*remove_flags=*/true);

  if (absl::GetFlag(FLAGS_fail)) {
    std::cerr << "Failure output in stderr." << std::endl;
    std::cout << "Failure output in stdout." << std::endl;
    return 1;
  }

  std::cerr << "Output in stderr." << std::endl;
  std::cout << "Output in stdout." << std::endl;

  return 0;
}
