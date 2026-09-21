#pragma once

#include <string>

namespace truck_demo {

class DemoSession;

[[nodiscard]] std::string makeRunDirectory(const std::string& parent = "runs");
[[nodiscard]] std::string writeSessionLog(
    const std::string& directory,
    const DemoSession& session);

}  // namespace truck_demo
