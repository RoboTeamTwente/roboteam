// #include <string>
// #include <chrono>
// #include <thread>
// #include <iostream>
// #include <zmqpp/zmqpp.hpp>

// int main() 
// {
//     using namespace std::chrono_literals;

//     std::cout << "Starting server..." << std::endl;

//     // initialize the zmq context with a single IO thread
//     zmqpp::context context;

//     // construct a REP (reply) socket and bind to interface
//     zmqpp::socket_t socket{context, zmqpp::socket_type::rep};
//     socket.bind("tcp://*:5555");

//     std::cout << "Server is running on port 5555..." << std::endl;

//     // prepare some static data for responses
//     const std::string data{"World"};

//     for (;;) 
//     {
//         zmqpp::message_t request;

//         std::cout << "Waiting for messages..." << std::endl;
        
//         // After (warning fixed):
//         if (!socket.recv(request, zmqpp::recv_flags::none)) {
//             std::cerr << "Failed to receive message" << std::endl;
//             continue;
//         }
//         std::cout << "Received " << request.to_string() << std::endl;

//         // simulate work
//         std::this_thread::sleep_for(1s);

//         // send the reply to the client
//         auto res = socket.send(zmqpp::buffer(data), zmqpp::send_flags::none);
//         if (!res) {
//             std::cerr << "Failed to send response" << std::endl;
//             continue;
//         }
//         std::cout << "Sent response: " << data << std::endl;
//     }

//     return 0;
// }