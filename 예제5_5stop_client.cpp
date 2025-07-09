/*stop 서비스를 request하는 client와 stop request 구현하기
(1) node 정의
(2) client 정의
(3) request 정의
(4) emtpy형식의 서비스를 정의했기 때문에 내용없는 응답(response)정의 --> result_future(내용없는 객체 정의)

이 흐름은 이렇게 작동해:
async_send_request()로 비동기 요청 보냄.
spin_until_future_complete()는 이 요청에 대한 응답이 도착할 때까지 스핀을 돌림.
응답이 오면 → FutureReturnCode::SUCCESS 반환.

✅ 여기서 핵심:
SUCCESS는 "내용 있는 응답이 도착했는가?"가 아니라 "응답이 도착했는가?"를 기준으로 판단해.
*/

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

#include <memory>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("service_client");
    rclcpp::Client<std_srvs::srv::Empty>::Shared_Ptr client = node->create_client<std_srvs::srv::Empty>("stop");

    auto request = std::make_shared<std_srvs::srv::Empty::Request>();

    while (!client->wait_for_service(1s)) {
        if(!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"service not available, waiting again...");
    }

    auto result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node,result_future)== rclcpp::FutureReturnCode::SUCCESS){
        auto result= result_future.get();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"The robot has stopped");
    } else{
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Failed to call service /stop");
    }
    
    rclcpp::shutdown();
    return 0;
}
    

/* client->wait_for_service(1s)는
client가 괄호 안의 수만큼 서버를 기다리고,
서버가 있다면 True, 없으면 False

rclcpp::ok()는 
정상작동하면 True, 비정상 작동하면 false
*/
