//
// Created by rolf on 28-05-22.
//

#include "LogFileWriter.h"

bool rtt::LogFileWriter::open(const std::string &file_name) {
    file = std::make_unique<std::ofstream>(file_name,std::ios::out | std::ios::binary);
    if(!file->is_open()){
        std::cout<<"Could not open file at location: "<<file_name<<"\n";
        close();
        return false;
    }

    LogFileHeader header;
    last_written_timestamp = 0;
    file->write(reinterpret_cast<char*>(&header),sizeof(header));
    bool good = file->good();
    if (!good){
        std::cout<<"First file write failed, closing file\n";
        close();
    }else{
        std::cout<<"Writing to log file: "<<file_name<<"\n";
    }
    return good;
}

bool rtt::LogFileWriter::addMessage(const rtt::logged_proto_type &message, logged_time_type timestamp) {

    if (timestamp < last_written_timestamp){
        return false;
    }

    std::size_t message_size = message.ByteSizeLong();

    if(!message.IsInitialized()){
        std::cout<<"Cannot write uninitialized messages to Log File!\n";
        return false;
    }
    if(serialization_buffer.size() < message_size){
        serialization_buffer.resize(message_size);
    }
    if(!message.SerializeToArray(serialization_buffer.data(),message_size)){
        std::cout<<"Error in serializing message to Log File!\n";
        return false;
    }

    last_written_timestamp = timestamp;
    LogDataHeader dataHeader{
        .timestamp = timestamp,
        .message_size = message_size
    };
    file->write(reinterpret_cast<char*>(&dataHeader),sizeof(dataHeader));
    file->write(serialization_buffer.data(),message_size);
    bool good = file->good();
    if (!good){
        std::cout<<"Error writing to file!\n";
    }
    return good;
}

void rtt::LogFileWriter::close() {
    file->clear();
    file->close();
    file.reset();//Clears the file stream
}
