

/**
This file define state machine
This compnent sends random position when the robot receives the user request.
*/


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


using RandomPosition = rt2_assignment1::srv::RandomPosition;
using Position=rt2_assignment1::srv::Position;
using Command=rt2_assignment1::srv::Command;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

using namespace std::chrono_literals;

namespace rt2_assignment1{
class StateMachine : public rclcpp::Node
{
    public:
    /**
    Constructor
    Server : "user_interface"
            callback function : user_interface
    
    Client :/position_server
    
    Client :/go_to_point
    */
    StateMachine(const rclcpp::NodeOptions & options)
    : Node("state_machine", options)
    {
    service_ = this->create_service<Command>("/user_interface", std::bind(&StateMachine::user_interface, this, _1, _2, _3));

    client_rp = this->create_client<RandomPosition>("/position_server");
    this->request_rp = std::make_shared<rt2_assignment1::srv::RandomPosition::Request>();
    this->response_rp = std::make_shared<rt2_assignment1::srv::RandomPosition::Response>();
    
    while (!client_rp->wait_for_service(std::chrono::seconds(1))){
     if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service /position_server to appear...");
    }

    client_p = this->create_client<Position>("/go_to_point");
    this->request_p = std::make_shared<rt2_assignment1::srv::Position::Request>();

    while (!client_p->wait_for_service(std::chrono::seconds(1))){
     if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service /go_to_point to appear...");
    }

    timer_ = this->create_wall_timer(
          500ms, std::bind(&StateMachine::set_new_destination, this));

    // max and min value of random position
    this->request_rp->x_max = 5.0;
    this->request_rp->x_min = -5.0;
    this->request_rp->y_max = 5.0;
    this->request_rp->y_min = -5.0;

    
    }
    std::shared_ptr<RandomPosition::Request> request_rp;
    std::shared_ptr<rt2_assignment1::srv::RandomPosition::Response> response_rp;
    std::shared_ptr<Position::Request> request_p;
    private:

    /**
    This is the callback function to set
    */
    void set_new_destination(){
        if(start && !moving){
            go_to_destination();
            moving=true;
        }
        else if(start && moving)
        {
            std::cout << "the robot is moving to the goal, please wait until the robot reaches it" << std::endl;
        }
        else if (!start && !moving)
        {
            std::cout << "The robot waits until the user commands" << std::endl;
        }
        else{
            moving=false;
        }
    }


    /**
    This is the function which sends the request 
    to /go_to_point and set the destination of the robot
    which is received via '/position_server'.
    */
    void go_to_destination(){
        // get the random position
        get_random_position();

        // set the random position to send it to /go_to_point
        this->request_p->x = this->response_rp->x;
   		this->request_p->y = this->response_rp->y;
   		this->request_p->theta = this->response_rp->theta; 

        RCLCPP_INFO(this->get_logger(), "Go to x=%f y=%f theta=%f",
        this->request_p->x, this->request_p->y, this->request_p->theta);

        using ServiceResponseFuture =
        rclcpp::Client<Position>::SharedFuture;

        // if the robot reaches the goal, 
        auto response_received_callback = [this](ServiceResponseFuture future) {
        RCLCPP_INFO(this->get_logger(), "Got result: [%" PRId64 "]", future.get());
        };
        
        // Send a request to the server (asynchronous)
        auto future_result = client_p->async_send_request(this->request_p, response_received_callback);
    }

    /**
    This is the callback function to change start flage to true when request from the user is "start"
    which is received via '/position_server'.
    */
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


    /**
    This is the callback function 
    to get the random position using service(/position_server)
    */
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
    bool moving=false;

    rclcpp::Service<Command>::SharedPtr service_;
    rclcpp::Client<Position>::SharedPtr client_p; 
    rclcpp::Client<RandomPosition>::SharedPtr client_rp;

    

    rclcpp::TimerBase::SharedPtr timer_;   
};
}
RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::StateMachine) 