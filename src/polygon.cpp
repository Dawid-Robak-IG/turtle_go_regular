#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <cmath>
#include <vector>

class Polygon : public rclcpp::Node {
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    int n;
    double m;
    int s;
    size_t vertices_idx;
    std::vector<std::pair<double, double>> vertices;
    std::vector<std::pair<double, double>> points;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client;
public:
    Polygon(): Node("polygon_navigator") {
        vertices_idx=0;
        action_client = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "THERE IS NO NAVIGATE_TO_POSE ACTION, TURN ON NAV2!");
            rclcpp::shutdown();
        }        

        this->declare_parameter<int>("sides",4);
        this->get_parameter("sides",n);
        if(n<3)n=3;
        RCLCPP_INFO(this->get_logger(),"number of sides: %d",n);

        this->declare_parameter<double>("len",0.70);
        this->get_parameter("len",m);
        if(m<0.0)m=1.0;
        RCLCPP_INFO(this->get_logger(),"length of sides: %.2f",m);

        this->declare_parameter<int>("points",10);
        this->get_parameter("points",s);
        if(s<1)s=1;
        RCLCPP_INFO(this->get_logger(),"points for side: %d",s);

    }
    void move() {
        double angle_step = 2 * M_PI / n;
        double x = m;
        double y = 0.0;

        for (int i = 0; i < n; ++i) {
            x = m * cos(angle_step * i);
            y = m * sin(angle_step * i);
            vertices.emplace_back(x, y);
        }
        for (size_t i = 0; i < vertices.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "vertex %lu: (%.2f, %.2f)", i, vertices[i].first, vertices[i].second);
        }

        for (int i = 0; i < n; ++i) {
            auto [x1, y1] = vertices[i];
            auto [x2, y2] = vertices[(i + 1) % n];

            for (int j = 0; j < s; ++j) {
                double t = static_cast<double>(j)/(s);
                double x = x1 + t * (x2 - x1);
                double y = y1 + t * (y2 - y1);
                points.emplace_back(x, y);
            }
        }
        send_goals(points,0);
    }
private:
    void send_goals(const std::vector<std::pair<double, double>>& points, size_t idx) {
        if (idx >= points.size()) {
            RCLCPP_INFO(this->get_logger(), "I finished going on my polygon");
            set_last();
            return;
        }

        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = now();
        msg.header.frame_id = "map";
        msg.pose.position.x = points[idx].first;
        msg.pose.position.y = points[idx].second;
        msg.pose.position.z = 0.0;

        vertices_idx = idx/s;
        double dx = vertices[(vertices_idx+1)%vertices.size()].first - vertices[vertices_idx].first;
        double dy = vertices[(vertices_idx+1)%vertices.size()].second - vertices[vertices_idx].second;
        double yaw = atan2(dy, dx);

        msg.pose.orientation.z = sin(yaw / 2.0);
        msg.pose.orientation.w = cos(yaw / 2.0);
        
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = msg;

        RCLCPP_INFO(this->get_logger(), "Sending goal %lu: (%.2f, %.2f)", idx, msg.pose.position.x, msg.pose.position.y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this, idx](const GoalHandleNavigateToPose::WrappedResult& result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Goal %lu succeeded!", idx);
            } else {
                RCLCPP_WARN(this->get_logger(), "Goal %lu failed or was canceled", idx);
            }
            send_goals(this->points, idx + 1);
        };

        action_client->async_send_goal(goal_msg, send_goal_options);
    }
    void set_last(){
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = now();
        msg.header.frame_id = "map";
        msg.pose.position.x = points[0].first;
        msg.pose.position.y = points[0].second;
        msg.pose.position.z = 0.0;

        double dx = vertices[1].first - vertices[0].first;
        double dy = vertices[1].second - vertices[0].second;
        double yaw = atan2(dy, dx);

        msg.pose.orientation.z = sin(yaw / 2.0);
        msg.pose.orientation.w = cos(yaw / 2.0);
        
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = msg;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult& result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Final goal succeeded!");
            } else {
                RCLCPP_WARN(this->get_logger(), "Final goal failed or was canceled");
            }
            rclcpp::shutdown();
        };

        action_client->async_send_goal(goal_msg, send_goal_options);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto polygon = std::make_shared<Polygon>();
    polygon->move();
    rclcpp::spin(polygon);
    rclcpp::shutdown();
    return 0;
}
