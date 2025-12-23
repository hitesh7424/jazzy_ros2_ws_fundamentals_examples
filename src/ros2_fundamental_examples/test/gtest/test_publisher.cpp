/**
 * @file test_publisher.cpp

 * @brief Unit test for the ros 2 minimal publisher node.
 * 
 * this file contains test cases to verify the functionality of out minimal publisher.
 * we test two main things:
 * 1. that the node is created correctly with the right name and topic
 * 2. that it publishes the expected "Hello World!" message
 * 
 * Testing framework:
 *  Google test (gtest) for c++ unit testing
 * 
 * tests:
 *      testNodeCreation: verifies node name and publisher setup
 *      testMessageContent: verifies published message format
 * 
 *   @author Hitesh 
 * @date December 21, 2025
 * 
 * 
 */


 #include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalCppPublisher;

#define TESTING_EXCLUDE_MAIN
#include "../../src/cpp_minimal_publisher.cpp"

class TestMinimalPublisher : public ::testing::Test
{
protected:
    void SetUp(){
        rclcpp::init(0,nullptr);
        node = std::make_shared<MinimalCppPublisher>();
    }
    void TearDown() override
    {
        node.reset();
        rclcpp::shutdown();
    }
    std::shared_ptr<MinimalCppPublisher> node;

};

TEST_F(TestMinimalPublisher, TestNodeCreation)
{
    EXPECT_EQ(std::string(node->get_name()), std::string("minimal_cpp_publisher"));

    auto pub_endpoints = node->get_publishers_info_by_topic("/cpp_example_topic");

    EXPECT_EQ(pub_endpoints.size(), 1);

}

TEST_F(TestMinimalPublisher, TestMessageContent)
{
    std::shared_ptr<std_msgs::msg::String> received_msg;

    auto subscription = node->create_subscription<std_msgs::msg::String>("/cpp_example_topic", 10, [&received_msg](const std_msgs::msg::String::SharedPtr msg){
        received_msg = std::make_shared<std_msgs::msg::String>(*msg);
    });

    node->timerCallback();

    rclcpp::spin_some(node);

    ASSERT_NE(received_msg, nullptr);
    EXPECT_EQ(received_msg->data.substr(0,12), "Hello World!");
    
}

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

