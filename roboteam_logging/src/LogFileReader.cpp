//
// Created by rolf on 28-05-22.
//

#include "LogFileReader.h"

//TODO: unify error handling strategy

void rtt::LogFileReader::close() {
    file->clear();
    file->close();
    file.reset();

}

bool rtt::LogFileReader::open(const std::string &file_name) {
    file = std::make_unique<std::ifstream>(file_name,std::ios::in | std::ios::binary);
    if(!file->is_open()){
        std::cout<<"Could not open log file: "<<file_name<<"\n";
        close();
        return false;
    }
    LogFileHeader header;
    file->read(reinterpret_cast<char *>(&header),sizeof(LogFileHeader));
    if(strncmp(header.name,DEFAULT_LOGFILE_HEADER_NAME,LOGFILE_HEADER_NAME_SIZE) != 0){
        std::cout<<"Unrecognized logfile header name: "<<std::string(header.name)<<" \n";
        close();
        return false;
    }
    //Please try to maintain backwards compatibility, especially after a tournament has passed
    //Being able to read the old data may help us in the future!
    version = header.version;
    if(version != 0){
        std::cout<<"Logfile has unreadable version: "<<version<<"\n";
        close();
        return false;
    }
    std::cout<<"Reading log file: "<<file_name<<" (version "<<version<<")\n";
    bool file_good = indexFile();

    if(!file_good){
        close();
    }
    return file_good;
}

std::pair<rtt::logged_time_type, rtt::logged_proto_type> rtt::LogFileReader::readPacket(long file_offset) {
    if(file_offset != index.back()){
        file->seekg(file_offset);
        if(!file->eof()){
            LogDataHeader dataHeader;
            file->read(reinterpret_cast<char*>(&dataHeader),sizeof(LogDataHeader));
            char * buffer = new char[dataHeader.message_size];
            file->read(buffer,dataHeader.message_size);
            rtt::logged_proto_type message;
            bool success = message.ParseFromArray(buffer,dataHeader.message_size);
            delete[] buffer;
            if(success){
                return std::make_pair(dataHeader.timestamp,message);
            }else{
                return std::make_pair(INVALID_LOGGED_TIME,rtt::logged_proto_type());
            }
        }
    }
    file->clear();
    return std::make_pair(INVALID_LOGGED_TIME,rtt::logged_proto_type());
}

bool rtt::LogFileReader::indexFile() {
    index.clear();
    LogDataHeader dataHeader;
    std::size_t packetNumber = 0;
    //TODO: clean up end check with index check maybe?
    while(!file->eof() && file->is_open()){
        long position = file->tellg();
        index.emplace_back(position);
        file->read(reinterpret_cast<char *>(&dataHeader),sizeof(dataHeader));
        if(file->bad()){
            return false;
        }
        //We also index the eof, this makes some checks a bit easier.
        if(file->eof()){
            break;
        }
        //Jump forward by message size
        file->seekg(dataHeader.message_size,std::ios_base::cur);
        packetNumber++;
    }
    resetToStartOfFile();
    return true;
}

std::pair<rtt::logged_time_type, rtt::logged_proto_type> rtt::LogFileReader::readNext() {
    //After reading a previous packet, the pointer is already pointing to the next Data frame's packet
    return readPacket(file->tellg());
}

void rtt::LogFileReader::resetToStartOfFile() {
    file->clear();//we clear any errors to prevent EOF flag from seekg failing!
    file->seekg(index[0]);
}

std::size_t rtt::LogFileReader::fileMessageCount() const {
    if(index.empty()){
        return 0;
    }
    return index.size()-1; //The end pos is also indexed
}

std::pair<rtt::logged_time_type, rtt::logged_proto_type> rtt::LogFileReader::readFrame(std::size_t frame_number) {
    return readPacket(index[frame_number]);
}

