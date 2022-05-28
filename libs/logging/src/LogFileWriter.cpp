//
// Created by rolf on 28-05-22.
//

#include "roboteam_logging/LogFileWriter.h"

bool rtt::LogFileWriter::open(const std::string &file_name) {
    file = std::make_unique<std::ofstream>(file_name,std::ios_base::out | std::ios_base::binary);
    if(!file->is_open()){
        std::cout<<"Could not open file at location: "<<file_name<<"\n";
        close();
        return false;
    }

    LogFileHeader header;
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
    char * message_buffer = new char[message_size];
    if(!message.IsInitialized()){
        std::cout<<"Cannot write uninitialized messages to Log File!\n";
        return false;
    }
    if(!message.SerializeToArray(message_buffer,message_size)){
        std::cout<<"Error in serializing message to Log File!\n";
        return false;
    }

    last_written_timestamp = timestamp;
    LogDataHeader dataHeader{
        .timestamp = timestamp,
        .message_size = message_size
    };
    file->write(reinterpret_cast<char*>(&dataHeader),sizeof(dataHeader));
    file->write(message_buffer,message_size);
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
