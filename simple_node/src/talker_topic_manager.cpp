#include "simple_node/talker_topic_manager.hpp"

#include <rclcpp/rclcpp.hpp>

namespace simple_node
{
  TalkerTopicManager::TalkerTopicManager(const std::shared_ptr<MessageGenerator> ptr_message_generator, const unsigned int message_num)
      : message_generator_(ptr_message_generator)
  {
    talker_topic_list_.reserve(message_num);
  }

  void TalkerTopicManager::register_message_generator(const std::shared_ptr<MessageGenerator> message_generator)
  {
    if (message_generator != nullptr)
    {
      message_generator_ = message_generator;
    }
  }

  bool TalkerTopicManager::create_publisher(const unsigned int id, rclcpp::Node *node, const std::string &topic_name)
  {
    return talker_topic_list_[id].create_publisher(node, topic_name);
  }

  bool TalkerTopicManager::generate_message(const unsigned int id, const YAML::Node &config)
  {
    if (id >= talker_topic_list_.size() || message_generator_ == nullptr)
    {
      return false;
    }

    message_generator_->generate_message(config, talker_topic_list_[id].get_message());
    return true;
  }

  bool TalkerTopicManager::add_publish_count_to_message(const unsigned int id)
  {
    if (id >= talker_topic_list_.size())
    {
      return false;
    }
    talker_topic_list_[id].add_publish_count_to_message();
    return true;
  }

  bool TalkerTopicManager::delete_publish_count_from_message(const unsigned int id)
  {
    if (id >= talker_topic_list_.size())
    {
      return false;
    }
    talker_topic_list_[id].delete_suffix_from_message(SUFFIX_LENGTH);
    return false;
  }

  bool TalkerTopicManager::publish_buffered_message(const unsigned int id, const bool show_count)
  {
    if (id >= talker_topic_list_.size())
    {
      return false;
    }

    if (show_count && add_publish_count_to_message(id)) {
      return false;
    }

    talker_topic_list_[id].publish();

    if (show_count)
    {
      delete_publish_count_from_message(id);
    }
    return true;
  }

  void TalkerTopicManager::increment_publishing_count(const unsigned int id) {
    talker_topic_list_[id].increment_count();
  }
}
