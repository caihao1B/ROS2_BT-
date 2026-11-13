// Teaching example only: evidence readiness is separate from process success.
#include <cassert>
#include <iostream>
#include <string>
std::string release_state(bool inputs, bool prerequisite, bool runtime, bool bounded) {
  if (!inputs) return "INVALID";
  return prerequisite && runtime && bounded ? "PASS" : "PARTIAL";
}
int main() {
  assert(release_state(true, false, true, true) == "PARTIAL");
  assert(release_state(false, true, true, true) == "INVALID");
  assert(release_state(true, true, true, true) == "PASS");
  std::cout << "PASS: 3 teaching cases; real project status is not inferred.\n";
}
