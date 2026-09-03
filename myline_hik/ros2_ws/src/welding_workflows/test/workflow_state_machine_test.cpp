#include "welding_workflows/workflow_state_machine.hpp"

#include <gtest/gtest.h>

TEST(WorkflowStateMachine, EnforcesExpectedStatePauseResumeAndTerminalRules)
{
  welding_workflows::WorkflowStateMachine machine;
  std::string error;
  ASSERT_TRUE(machine.start("session_1", "task_1", "test", {"CHECK", "RUN"}, &error));
  EXPECT_FALSE(machine.advance("RUN", &error));
  EXPECT_TRUE(machine.pause("CHECK", &error));
  EXPECT_FALSE(machine.advance("CHECK", &error));
  EXPECT_TRUE(machine.resume(&error));
  EXPECT_EQ(machine.stage(), "CHECK");
  EXPECT_TRUE(machine.advance("CHECK", &error));
  EXPECT_EQ(machine.stage(), "RUN");
  EXPECT_TRUE(machine.advance("RUN", &error));
  EXPECT_EQ(machine.stage(), "COMPLETE");
  EXPECT_TRUE(machine.terminal());
  EXPECT_FALSE(machine.resume(&error));
}

TEST(WorkflowStateMachine, AbortCannotResume)
{
  welding_workflows::WorkflowStateMachine machine;
  std::string error;
  ASSERT_TRUE(machine.start("session_1", "task_1", "test", {"RUN"}, &error));
  EXPECT_TRUE(machine.abort(&error));
  EXPECT_EQ(machine.stage(), "ABORTED");
  EXPECT_FALSE(machine.resume(&error));
}
