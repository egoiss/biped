/* Copyright 2019-2020 The MathWorks, Inc. */

//This function is for internal use only. It may be removed in the future.
// DO NOT EDIT!

#include "gazebotransport/gazebocustom/gazebocustommsghandler/stoupentoPlugin_msgs_msgs_ControlCommandCustomMessageHandler.hpp"

namespace robotics {
namespace gazebotransport {

stoupentoPlugin_msgs_msgs_ControlCommandCustomMessageHandler::stoupentoPlugin_msgs_msgs_ControlCommandCustomMessageHandler(gazebo::transport::NodePtr node)
    : m_node(node)
    , m_mutex()
{
}

/// Destructor
stoupentoPlugin_msgs_msgs_ControlCommandCustomMessageHandler::~stoupentoPlugin_msgs_msgs_ControlCommandCustomMessageHandler() {
}

void stoupentoPlugin_msgs_msgs_ControlCommandCustomMessageHandler::setCustomPub(gazebo::transport::PublisherPtr customPubPtr,
                                          std::string const& topicName) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_customPublisherMap.insert(std::make_pair(topicName, customPubPtr));
}

gazebo::transport::PublisherPtr stoupentoPlugin_msgs_msgs_ControlCommandCustomMessageHandler::getCustomPub(std::string const& topicName) {
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto iter = m_customPublisherMap.find(topicName); // find input topic name
        if (iter != m_customPublisherMap.end()) {
            return iter->second;
        } else {
            return nullptr; // For invalid topic name
        }
    }
}

std::string stoupentoPlugin_msgs_msgs_ControlCommandCustomMessageHandler::initPublisher(mw::internal::robotics::gazebotransport::Packet const& msgContent) {

    std::string topicName = msgContent.init_custom_publisher().topic_name();

    mw::internal::robotics::gazebotransport::Packet replyMsg;
    replyMsg.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID_STATUS);
    replyMsg.mutable_header()->mutable_time_stamp()->set_seconds(0);
    replyMsg.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);

    auto iter = m_customPublisherMap.find(topicName); // find input topic name

    if (iter != m_customPublisherMap.end()) {
        // For already initialized topic name
		replyMsg.set_status(mw::internal::robotics::gazebotransport::Packet_CoSimError_NONE);
    } else {
        this->setCustomPub(this->m_node->Advertise<stoupentoPlugin_msgs::msgs::ControlCommand>(topicName),
                           topicName);
        replyMsg.set_status(mw::internal::robotics::gazebotransport::Packet_CoSimError_NONE);
    }

    // Returns success message
    return replyMsg.SerializeAsString();
}

std::pair<std::shared_ptr<stoupentoPlugin_msgs_msgs_ControlCommandSubscriber>, bool> stoupentoPlugin_msgs_msgs_ControlCommandCustomMessageHandler::insert(
    std::string const& topic_name,
    std::shared_ptr<stoupentoPlugin_msgs_msgs_ControlCommandSubscriber> subsciberPointer) {
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto insertResult =
            this->m_subscriberPtrMap.insert(std::make_pair(topic_name, subsciberPointer));
        return std::make_pair(insertResult.first->second, insertResult.second);
    }
}

std::shared_ptr<stoupentoPlugin_msgs_msgs_ControlCommandSubscriber> stoupentoPlugin_msgs_msgs_ControlCommandCustomMessageHandler::find(std::string const& topic) {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto iter = this->m_subscriberPtrMap.find(topic);
    if (iter != this->m_subscriberPtrMap.end()) {
        return iter->second;
    } else {
        return nullptr;
    }
}

std::string stoupentoPlugin_msgs_msgs_ControlCommandCustomMessageHandler::initSubscriber(mw::internal::robotics::gazebotransport::Packet const& msgContent) {

    std::string topicName = msgContent.init_custom_subscriber().topic_name();

    mw::internal::robotics::gazebotransport::Packet replyMsg;
    replyMsg.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID_STATUS);
    replyMsg.mutable_header()->mutable_time_stamp()->set_seconds(0);
    replyMsg.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);

    auto iter = m_subscriberPtrMap.find(topicName); // find input topic name

    if (iter != m_subscriberPtrMap.end()) {
        // For already initialized topic name
		replyMsg.set_status(mw::internal::robotics::gazebotransport::Packet_CoSimError_NONE);
    } else {
        // create and store subscriber instance for each topic name
        auto insertResult =
            this->insert(topicName, std::make_shared<stoupentoPlugin_msgs_msgs_ControlCommandSubscriber>(topicName));
        insertResult.first->InitImpl(this->m_node);
        replyMsg.set_status(mw::internal::robotics::gazebotransport::Packet_CoSimError_NONE);
    }

    // Returns success message
    return replyMsg.SerializeAsString();
}

std::string stoupentoPlugin_msgs_msgs_ControlCommandCustomMessageHandler::publishCustomMsg(mw::internal::robotics::gazebotransport::Packet const& msgContent) {
    std::string topicName = msgContent.custom_message_support().topic_name();
    std::string dataString = msgContent.custom_message_support().data();

    // Reply error message
    mw::internal::robotics::gazebotransport::Packet replyMsg;
    replyMsg.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID_STATUS);
    replyMsg.mutable_header()->mutable_time_stamp()->set_seconds(0);
    replyMsg.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);

    // Find topic is initialized or not
    auto pubPointer = this->getCustomPub(topicName);

    if (pubPointer) {
        stoupentoPlugin_msgs::msgs::ControlCommand stoupentoPlugin_msgs_msgs_ControlCommand_message;
        if (stoupentoPlugin_msgs_msgs_ControlCommand_message.ParseFromString(dataString)) {
            pubPointer->Publish(stoupentoPlugin_msgs_msgs_ControlCommand_message);
            // assign no error
            replyMsg.set_status(mw::internal::robotics::gazebotransport::Packet_CoSimError_NONE);
        } else {
            // assign invalid custom message type error
            replyMsg.set_status(mw::internal::robotics::gazebotransport::Packet_CoSimError_CUSTOM_MESSAGE_INVALID);
        }
    } else {
        // assign invalid topic name error
        replyMsg.set_status(mw::internal::robotics::gazebotransport::Packet_CoSimError_TOPIC_NAME_INVALID);
    }

    // Returns serialized message
    return replyMsg.SerializeAsString();
}

std::pair<bool,std::string>  stoupentoPlugin_msgs_msgs_ControlCommandCustomMessageHandler::subscribeCustomMsg(mw::internal::robotics::gazebotransport::Packet const& msgContent) {
    std::string topic_name = msgContent.request_custom_message_support().topic_name();

    auto result = this->find(topic_name);

    if (result) {
        std::string messageString = result->getMessage();
        bool messageStatus = result->getMessageStatus();
        result->resetMessageStatus();
        return std::make_pair(messageStatus, messageString);
    } else {
        return std::make_pair(false,"");
    }
}

} // namespace gazebotransport
} // namespace robotics
