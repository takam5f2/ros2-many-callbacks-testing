#ifndef __TALKER_TOPIC_MANAGER_HPP__
#define __TALKER_TOPIC_MANAGER_HPP__

#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include "simple_node/message_generator.hpp"
#include <string>
#include <sstream>
#include <iomanip>

namespace simple_node
{
  struct TalkerTopic {
    public:
     // constructor and destructor
      TalkerTopic() = default;
      virtual ~TalkerTopic() = default;

      bool create_publisher (rclcpp::Node *node, const std::string &topic_name){
        try
        {
          publisher_ = node->create_publisher<std_msgs::msg::String>(topic_name, 10);
        }
        catch (std::exception &e) {
          return false;
        }
        return true;
      }

      std_msgs::msg::String &get_message() {
        return message_;
      }

      void add_publish_count_to_message() {
        std::stringstream suffix_stream;
        suffix_stream << " [" << std::setw(8) << std::setfill('0') << publishing_count_ << "]";
        std::string suffix = suffix_stream.str();

        message_.data += suffix;
      }

      void delete_suffix_from_message() {
        message_.data.resize(message_.data.length() - SUFFIX_LENGTH);
      }

      void publish() {
        if (show_count_) {
          add_publish_count_to_message();
        }
        publisher_->publish(message_);
        increment_count();
        if (show_count_) {
          dalete_suffix_from_message();
        }
      }

      inline void increment_count() {
        publishing_count_++;
        publishing_count_ = publishing_count_ & MASK_TO_DIGITS;
      }

      void set_launch(const bool launch) {
        launch_ = launch;
      }

      void check_launch() {
        return launch_;
      }

      void set_show_count(const bool show_count) {
        show_count_ = show_count;
      }

    private:
      rclcpp::Node *ptr_node_;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_{nullptr};
      std_msgs::msg::String message_{};
      bool launch_{false};
      bool show_count_{false};
      unsigned int publishing_count_{0};
      static constexpr unsigned int MASK_TO_DIGITS = 0xffffffff ^ (0xffffffff << 26); // this bit mask set limits of counters for 8 digits.
      static constexpr unsigned int SUFFIX_LENGTH = 11u;
    };

  class TalkerTopicManager
  {
    public:
      TalkerTopicManager() = default;
      virtual ~TalkerTopicManager() = default;

      bool init_by_config(rclcpp::Node *node, const YAML::Node config, unsigned int random_seed);

      const unsigned int TalkerTopicManager::get_topic_num();
      const bool TalkerTopicManager::check_launch(const unsigned int id);

      bool publish_buffered_message(const unsigned int id, const bool  show_count);

      void increment_publishing_count(const unsigned int id);

    private:
      std::vector<struct TalkerTopic> talker_topic_list_{};
  };
}

#endif // __TALKER_TOPIC_MANAGER_HPP__
