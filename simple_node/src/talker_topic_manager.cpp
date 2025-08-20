#include "simple_node/talker_topic_manager.hpp"

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace simple_node
{
  bool TalkerTopicManager::init_by_config (rclcpp::Node *node,
    const YAML::Node config, unsigned int random_seed)  {

    if (!config["callbacks"] || !config["callbacks"].IsSequence()) {
      return false;
    }

    MessageGenerator message_generator{random_seed};

    for (auto cb_config : config["callbacks"]) {
      TalkerTopic talker_topic;

      talker_topic.set_launch(cb_config["launch"].as<bool>(true));
      talker_topic.set_show_count(cb_config["show_config"].as<bool>(true));

      if (talker_topic.check_launch()) {
        std::string topic_name = cb_config["topic"].as<std::string>("/topic/no_name/message0");
        talker_topic.create_publisher(node, topic_name);
        message_generator.generate_message(cb_config, talker_topic.get_message());
      }
      talker_topic_list_.emplace_back(talker_topic);
    }
    return true;
  }

  unsigned int TalkerTopicManager::get_topic_num() {
    return talker_topic_list_.size();
  }

  bool TalkerTopicManager::check_launch(const unsigned int id) {
    if (id >= get_topic_num()) {
      return false;
    }
    return talker_topic_list_[id].check_launch();
  }

  const char* TalkerTopicManager::get_topic_name(const unsigned int id) {
    return talker_topic_list_[id].get_topic_name();
  }

  bool TalkerTopicManager::publish_buffered_message(const unsigned int id)
  {
    if (id >= talker_topic_list_.size())
    {
      return false;
    }

    talker_topic_list_[id].publish();
    return true;
  }

  void TalkerTopicManager::increment_publishing_count(const unsigned int id) {
    talker_topic_list_[id].increment_count();
  }

}
