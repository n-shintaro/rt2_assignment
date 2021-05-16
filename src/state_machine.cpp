#include <memory>
#include <chrono>
#include <cinttypes>
#include <memory>
#include <cstdlib>

#include "rt2_assignment1/srv/command.hpp"
#include "rt2_assignment1/srv/position.hpp"
#include "rt2_assignment1/srv/random_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"


bool start = false;
using RandomPosition = rt2_assignment1::srv::RandomPosition;
using Position=rt2_assignment1::srv::Position;
using Command=rt2_assignment1::srv::Command;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1{
class StateMachine : public rclcpp::Node
{
    public:

    StateMachine(const rclcpp::NodeOptions & options)
    : Node("state_machine", options)
    {
    service_ = this->create_service<Command>("user_interface", std::bind(&StateMachine::user_interface, this, _1, _2, _3));
    client_rp = this->create_client<RandomPosition>("/position_server");
    client_p = this->create_client<Position>("/go_to_point");


    this->request_rp = std::make_shared<rt2_assignment1::srv::RandomPosition::Request>();
    this->response_rp = std::make_shared<rt2_assignment1::srv::RandomPosition::Response>();
    this->request_p = std::make_shared<rt2_assignment1::srv::Position::Request>();

    this->request_rp->x_max = 5.0;
    this->request_rp->x_min = -5.0;
    this->request_rp->y_max = 5.0;
    this->request_rp->y_min = -5.0;

    if(start){
        go_to_destination();
    }
    }
    private:

    void go_to_destination(){
        get_random_position();

        this->request_p->x = response_rp->x;
   		this->request_p->y = response_rp->y;
   		this->request_p->theta = response_rp->theta;
        
        

        RCLCPP_INFO(this->get_logger(), "Go to x=%f y=%f theta=%f",
        this->request_p->x, this->request_p->y, this->request_p->theta);

        using ServiceResponseFuture =
        rclcpp::Client<Position>::SharedFuture;

        auto response_received_callback = [this](ServiceResponseFuture future) {
        RCLCPP_INFO(this->get_logger(), "Got result: [%" PRId64 "]", future.get());
        };
        auto future_result = client_p->async_send_request(this->request_p, response_received_callback);
    }

    void user_interface(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<Command::Request> request,
        const std::shared_ptr<Command::Response> response
    )
        
    {
        (void)request_header;
        if (request->command == "start"){
            start = true;
        }
        else {
            start = false;
        }
        response->ok=true;
    }

    void get_random_position(){
        using ServiceResponseFuture =
        rclcpp::Client<RandomPosition>::SharedFuture;

        auto rp_received_callback = [this](ServiceResponseFuture future) {
            RCLCPP_INFO(this->get_logger(), "Got result:  x=%f y=%f theta=%f", 
            future.get()->x,future.get()->y,future.get()->theta);
            response_rp=future.get();
        };

        auto future_result = client_rp->async_send_request(this->request_rp, rp_received_callback);

    }

    bool start=false;


    rclcpp::Service<Command>::SharedPtr service_;
    rclcpp::Client<Position>::SharedPtr client_p; 
    rclcpp::Client<RandomPosition>::SharedPtr client_rp;

    std::shared_ptr<RandomPosition::Request> request_rp;
    std::shared_ptr<rt2_assignment1::srv::RandomPosition::Response> response_rp;

    std::shared_ptr<Position::Request> request_p;

    rclcpp::TimerBase::SharedPtr timer_;   
};
}
RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::StateMachine) 