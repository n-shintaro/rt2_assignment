#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rt2_assignment1/RandomPosition.h"
#include "rt2_assignment1/Position.h"
#include "rt2_assignment1/Command.h"

bool start = false;
using RandomPosition = rt2_assignment1::RandomPosition;
using Position=rt2_assignment1::Position;
using Command=rt2_assignment1::Command;

namespace rt2_assignment1{
class StateMachine : public rclcpp::Node
{
    public:
    StateMachine(const rclcpp::NodeOptions & options)
    : Node("state_machine", options)
    {
    service_ = this->create_service<???>("user_interface", std::bind(&StateMachine::user_interface, this, _1, _2, _3));
    client_rp = this->create_client<RandomPosition>("/position_server");
    client_p = this->create_client<Position>("/go_to_point");

    rt2_assignment1::RandomPosition rp;
    rp.request.x_max = 5.0;
    rp.request.x_min = -5.0;
    rp.request.y_max = 5.0;
    rp.request.y_min = -5.0;
    rt2_assignment1::Position p;

    while(ros::ok()){
   	ros::spinOnce();
   	if (start){
   		client_rp.call(rp);
   		p.request.x = rp.response.x;
   		p.request.y = rp.response.y;
   		p.request.theta = rp.response.theta;
   		std::cout << "\nGoing to the position: x= " << p.request.x << " y= " <<p.request.y << " theta = " <<p.request.theta << std::endl;
   		client_p.call(p);
   		std::cout << "Position reached" << std::endl;
   	}
   }
    }

    private:
    void user_interface(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<Command::Request> request,
        const std::shared_ptr<Command::Response> response
    )
        
    {
        (void)request_header;
    }
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::StateMachine) 