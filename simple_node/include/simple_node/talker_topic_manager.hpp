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
        publisher_->publish(message_);
        increment_count();
      }

      inline void increment_count() {
        publishing_count_++;
        publishing_count_ = publishing_count_ & MASK_TO_DIGITS;
      }

    private:
      rclcpp::Node *ptr_node_;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_{nullptr};
      std_msgs::msg::String message_{};
      unsigned int publishing_count_{0};
      static constexpr unsigned int MASK_TO_DIGITS = 0xffffffff ^ (0xffffffff << 26); // this bit mask set limits of counters for 8 digits.
      static constexpr unsigned int SUFFIX_LENGTH = 11u;
    };

  class TalkerTopicManager
  {
    public:
  TalkerTopicManager(
      const std::shared_ptr<MessageGenerator> ptr_message_generator, const unsigned int message_num);
      virtual ~TalkerTopicManager() = default;

      void register_message_generator(const std::shared_ptr<MessageGenerator>  message_generator);

      bool create_publisher(const unsigned int id, rclcpp::Node *node, const std::string & topic_name);

      bool generate_message(const unsigned int id, const YAML::Node & config);

      bool publish_buffered_message(const unsigned int id, const bool  show_count);

      void increment_publishing_count(const unsigned int id);

    private:
      bool add_publish_count_to_message(const unsigned int id);
      bool delete_publish_count_from_message(const unsigned int id);


      std::vector<struct TalkerTopic> talker_topic_list_;
      std::shared_ptr<MessageGenerator> message_generator_;
  };
}

#endif // __TALKER_TOPIC_MANAGER_HPP__
