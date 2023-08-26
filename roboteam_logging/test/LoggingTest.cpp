//
// Created by rolf on 28-05-22.
//

#include <filesystem>

#include "gtest/gtest.h"
#include "roboteam_logging/LogFileReader.h"
#include "roboteam_logging/LogFileWriter.h"

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

bool removeFile(const std::string& file_name) {
    try {
        bool removed = std::filesystem::remove(file_name);
        if (!removed) {
            std::cout << "Could not delete file!\n";
            return false;
        }
    } catch (const std::filesystem::filesystem_error& err) {
        std::cout << "Could not remove file: " << err.what() << "\n";
        return false;
    }

    return true;
};

struct TestHeader {
    char name[rtt::LOGFILE_HEADER_NAME_SIZE];
    int version;
};
static_assert(sizeof(TestHeader) == sizeof(rtt::LogFileHeader));

TEST(logging_writing, open) {
    // Checks if the open function correctly opens the file and writes a header
    const std::string file_name("tmp.log");
    rtt::LogFileWriter writer;
    ASSERT_TRUE(writer.open(file_name));
    writer.close();

    ASSERT_TRUE(std::filesystem::exists(file_name));
    std::ifstream stream(file_name, std::ios_base::in | std::ios_base::binary);

    ASSERT_TRUE(stream.good());
    TestHeader testHeader;
    stream.read(reinterpret_cast<char*>(&testHeader), sizeof(TestHeader));
    ASSERT_TRUE(stream.good());
    ASSERT_TRUE(testHeader.version == rtt::LOGFILE_VERSION);
    for (int i = 0; i < rtt::LOGFILE_HEADER_NAME_SIZE; ++i) {
        ASSERT_TRUE(testHeader.name[i] == rtt::DEFAULT_LOGFILE_HEADER_NAME[i]);
    }
    ASSERT_TRUE(removeFile(file_name));
}

using Message = rtt::logged_proto_type;

TEST(logging, write_backwards_time) {
    // Checks if the logwriter and logreader correctly interpret each other

    const float constant = 0.12345;
    Message message;
    message.mutable_blue_robot_parameters()->mutable_parameters()->set_radius(constant);

    rtt::LogFileWriter writer;
    const std::string file_name("tmp.log");
    ASSERT_TRUE(writer.open(file_name));

    ASSERT_TRUE(writer.addMessage(message, 1));
    EXPECT_FALSE(writer.addMessage(message, 0));

    writer.close();
    ASSERT_TRUE(removeFile(file_name));
}

TEST(logging, write_uninitialized_proto) {
    // checks if the logwriter can not send uninitialized proto messages

    Message message;
    if (message.IsInitialized()) {
        return;  // Not necessary to test if we do not have any required fields
    }

    rtt::LogFileWriter writer;
    const std::string file_name("tmp.log");
    ASSERT_TRUE(writer.open(file_name));
    ASSERT_FALSE(writer.addMessage(message, 1));

    writer.close();
    ASSERT_TRUE(removeFile(file_name));
}

TEST(logging, write_read) {
    // Checks if the logwriter and logreader correctly interpret each other
    const float constant1 = 0.12345;
    Message message1;
    message1.mutable_blue_robot_parameters()->mutable_parameters()->set_radius(constant1);

    const float constant2 = 0.23456;
    Message message2;
    message2.mutable_blue_robot_parameters()->mutable_parameters()->set_radius(constant2);
    const std::string file_name("tmp.log");

    {
        rtt::LogFileWriter writer;
        ASSERT_TRUE(writer.open(file_name));

        ASSERT_TRUE(writer.addMessage(message1, 1));
        ASSERT_TRUE(writer.addMessage(message2, 2));

        writer.close();
    }
    {
        rtt::LogFileReader reader;
        ASSERT_TRUE(reader.open(file_name));

        auto [time1, read_message_1] = reader.readNext();
        EXPECT_TRUE(time1 == 1);
        EXPECT_TRUE(read_message_1.blue_robot_parameters().parameters().radius() == constant1);

        auto [time2, read_message_2] = reader.readNext();
        EXPECT_TRUE(time2 == 2);
        EXPECT_TRUE(read_message_2.blue_robot_parameters().parameters().radius() == constant2);

        auto [time1_skip_back, read_message_3] = reader.readFrame(0);
        EXPECT_TRUE(time1_skip_back == 1);

        reader.close();
    }
    ASSERT_TRUE(removeFile(file_name));
}