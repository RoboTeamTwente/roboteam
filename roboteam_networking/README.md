# roboteam_networking
Networking repository for handling connections between rtt components like AI, RobotHub and World

It is uses .proto files which use the protobuf protocol. The data is serialized/packed using the .proto files (they provide the structure).
The accompanying pb2 files take care of actually handling this data. The .proto files are like the template for the data streams.

For example World.proto defines the package that sends all information about the world state. We can access this data stream with the World_pb2.py file. It was created using the following command: protoc --python_out=. World.proto WorldBall.proto WorldRobot.proto RobotProcessedFeedback.proto
