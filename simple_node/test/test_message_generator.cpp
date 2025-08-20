// test_message_generator.cpp

#include <gtest/gtest.h>
#include "simple_node/message_generator.hpp" // The header file to be tested
#include <memory>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>


// Define a test fixture to prepare common objects for each test.
class MessageGeneratorTest : public ::testing::Test {
protected:
  // The SetUp method, called before each test case is run.
  void SetUp() override {
    // Initialize MessageGenerator with a fixed seed for reproducible tests.
    unsigned int fixed_seed = 12345;
    message_generator_ = std::make_unique<simple_node::MessageGenerator>(fixed_seed);
  }

  // The instance of the class under test.
  std::unique_ptr<simple_node::MessageGenerator> message_generator_;
};

/**
 * @brief Test for the repeat_message method.
 *
 * Case: Extend the string "abc" to 10 bytes.
 * The expected result is "abcabcabca", and the size should be 10.
 */
TEST_F(MessageGeneratorTest, RepeatMessageCorrectly) {
  // Arrange
  const std::string given_message = "abc";
  const size_t target_size = 10;
  std::string result_message;
  const std::string expected_message = "abcabcabca";

  // Act
  message_generator_->repeat_message(given_message, target_size, result_message);

  // Assert
  ASSERT_EQ(result_message.size(), target_size);
  ASSERT_EQ(result_message, expected_message);
}

/**
 * @brief Test for the randomize_message method.
 *
 * Case: Generate a random message of 50 bytes.
 * As per the requirements, we verify that the size of the generated message is 50.
 */
TEST_F(MessageGeneratorTest, RandomizeMessageHasCorrectSize) {
  // Arrange
  const std::string dummy_message = "abc"; // This argument is currently unused.
  const size_t target_size = 50;
  std::string result_message;

  // Act
  message_generator_->randomize_message(dummy_message, target_size, result_message);

  // Assert
  ASSERT_EQ(result_message.size(), target_size);
  ASSERT_FALSE(result_message.empty());
}

/**
 * @brief Test for generate_message method (GIVEN_MESSAGE pattern).
 *
 * Verifies the behavior when the "given_message" pattern is specified in the YAML config.
 */
TEST_F(MessageGeneratorTest, GenerateMessageWithGivenPattern) {
  // Arrange
  const char* yaml_string = R"(
- topic: /topic/message/mixed/as_is/name0
  frequency: 10
  message_pattern: 0 # as-is
  message: "This is a mixed message with no pattern"
)";
  YAML::Node config = YAML::Load(yaml_string);

  // print the YAML config for debugging
  std_msgs::msg::String result_msg;

  // Act
  message_generator_->generate_message(config[0], result_msg);

  // Assert
  ASSERT_EQ(result_msg.data.size(), 39);
  ASSERT_EQ(result_msg.data, "This is a mixed message with no pattern");
}

/**
 * @brief Test for generate_message method (REPEAT pattern).
 *
 * Verifies the behavior when the "repeat" pattern is specified in the YAML config.
 */
TEST_F(MessageGeneratorTest, GenerateMessageWithRepeatPattern) {
  // Arrange
  const char* yaml_string = R"(
- topic: /topic/message/repeat/name0
  frequency: 5
  message_pattern: 1 # repeat
  message: "r"
  message_bytes: 20 # Total size of the message
)";
  YAML::Node config = YAML::Load(yaml_string);
  std_msgs::msg::String result_msg;

  // Act
  message_generator_->generate_message(config[0], result_msg);

  // Assert
  ASSERT_EQ(result_msg.data.size(), 20);
  ASSERT_EQ(result_msg.data, "rrrrrrrrrrrrrrrrrrrr");
}

/**
 * @brief Test for generate_message method (RANDOM pattern).
 *
 * Verifies the behavior when the "random" pattern is specified in the YAML config.
 * As per the requirements, this confirms that the size of the generated message is correct.
 */
TEST_F(MessageGeneratorTest, GenerateMessageWithRandomPattern) {
  // Arrange
  const char* yaml_string = R"(
- topic: /topic/message/random/name0
  frequency: 2
  message_pattern: 2 # random
  message: "random"
  message_bytes: 123 # Total size of the message
)";
  YAML::Node config = YAML::Load(yaml_string);
  std_msgs::msg::String result_msg;

  // Act
  message_generator_->generate_message(config[0], result_msg);

  // Assert
  ASSERT_EQ(result_msg.data.size(), 123);
}

/**
 * @brief Test for generate_message method (invalid pattern).
 *
 * When an invalid pattern is specified, it should default to the GIVEN_MESSAGE pattern
 * and return an empty string (based on the current implementation). This is an edge case test.
 */
TEST_F(MessageGeneratorTest, GenerateMessageWithInvalidPattern) {
    // Arrange
    const char* yaml_string = R"(
- topic: /topic/message/invalid/name0
  frequency: 1
  message_pattern: 999 # Invalid pattern
  message: "" # Empty message
  message_bytes: 0 # No bytes reserved
)";
    YAML::Node config = YAML::Load(yaml_string);
    std_msgs::msg::String result_msg;

    // Act
    message_generator_->generate_message(config[0], result_msg);

    // Assert
    ASSERT_TRUE(result_msg.data.empty());
}
