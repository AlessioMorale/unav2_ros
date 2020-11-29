#ifndef UNAV2_TOPICS_H
#define UNAV2_TOPICS_H
#pragma once

namespace unav2_hardware {
namespace topics {
static constexpr auto &joint_command = "control/joints_cmd";
static constexpr auto &joint_status = "status/joints";
static constexpr auto &system_topic = "status/system";
static constexpr auto &battery_state_topic = "status/battery";
} // namespace topics
} // namespace unav2_hardware
#endif /* UNAV2_TOPICS_H */
