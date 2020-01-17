//
// Created by baris on 09/10/18.
//

#ifndef ROBOTEAM_AI_JSONREADER_H
#define ROBOTEAM_AI_JSONREADER_H

#include <gtest/gtest_prod.h>
#include <stdio.h> /* defines FILENAME_MAX */
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include "BTImport.h"
#include "json.h"
#include "vector"

#ifdef WINDOWS
#include <direct.h>
#define GetCurrentDir _getcwd
#else

#include <unistd.h>

#define GetCurrentDir getcwd
#endif

using json = nlohmann::json;

class JsonReader {
   private:
    FRIEND_TEST(JsonBasics, JsonTest);

    FRIEND_TEST(BT, FactoryTest);

    std::string getFilePath(std::string name);

    std::vector<std::string> split(std::string s, char c);

   protected:
   public:
    json readJSON(std::string fileName);

    JsonReader() = default;

    bool checkIfKeyExists(std::string key, json json);

    void editJSON(std::string fileName, std::string tree, std::string field, std::string newValue);
};

#endif  // ROBOTEAM_AI_JSONREADER_H
