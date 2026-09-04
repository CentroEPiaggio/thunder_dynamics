#include <cstddef>
#include <chrono>
#include <memory>
#include <string>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <rrr_server/msg/float32_array.hpp>
#include <rrr_server/srv/get_value.hpp>
#include <rrr_server/srv/set_value.hpp>

using namespace std::chrono_literals;

namespace {

template <typename Client>
void wait_for_service_or_throw(
    const Client& client,
    const rclcpp::Node::SharedPtr& node,
    const std::string& service_name) {
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            throw std::runtime_error("Interrupted while waiting for " + service_name);
        }
        RCLCPP_INFO(node->get_logger(), "Waiting for %s", service_name.c_str());
    }
}

template <typename Future>
auto wait_for_response(
    const rclcpp::Node::SharedPtr& node,
    Future future,
    const std::string& service_name) {
    if (rclcpp::spin_until_future_complete(node, future) != rclcpp::FutureReturnCode::SUCCESS) {
        throw std::runtime_error("Service call failed: " + service_name);
    }
    return future.get();
}

}  // namespace

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("rrr_service_client");

    try {
        const auto set_q = node->create_client<rrr_server::srv::SetValue>("/rrr_server/set_q");
        const auto get_q = node->create_client<rrr_server::srv::GetValue>("/rrr_server/get_q");
        const auto get_m = node->create_client<rrr_server::srv::GetValue>("/rrr_server/get_M");

        wait_for_service_or_throw(set_q, node, "/rrr_server/set_q");
        wait_for_service_or_throw(get_q, node, "/rrr_server/get_q");
        wait_for_service_or_throw(get_m, node, "/rrr_server/get_M");

        auto set_request = std::make_shared<rrr_server::srv::SetValue::Request>();
        set_request->value = {0.1F, 0.2F, 0.3F};
        auto set_response = wait_for_response(node, set_q->async_send_request(set_request), "/rrr_server/set_q");
        if (!set_response->success) {
            throw std::runtime_error("set_q rejected the request: " + set_response->message);
        }

        auto q_response = wait_for_response(
            node, get_q->async_send_request(std::make_shared<rrr_server::srv::GetValue::Request>()), "/rrr_server/get_q");
        RCLCPP_INFO(node->get_logger(), "q = [%.3f, %.3f, %.3f]",
                    q_response->value.at(0), q_response->value.at(1), q_response->value.at(2));

        auto m_response = wait_for_response(
            node, get_m->async_send_request(std::make_shared<rrr_server::srv::GetValue::Request>()), "/rrr_server/get_M");
        RCLCPP_INFO(node->get_logger(), "M has %zu row-major values", m_response->value.size());
        for (std::size_t index = 0; index < m_response->value.size(); ++index) {
            RCLCPP_INFO(node->get_logger(), "M[%zu] = %.6f", index, m_response->value[index]);
        }
    } catch (const std::exception& error) {
        RCLCPP_ERROR(node->get_logger(), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
