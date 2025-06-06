#pragma once
#include <string>
#include <vector>

// Check if a flag exists in command line args
bool hasFlag(int argc, char **argv, const std::string &flag)
{
    for (int i = 1; i < argc; i++)
    {
        if (std::string(argv[i]) == flag)
        {
            return true;
        }
    }
    return false;
}

// Get a string argument from command line
std::string getArg(int argc, char **argv, const std::string &flag, const std::string &defaultValue)
{
    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        if (arg == flag && i + 1 < argc)
        {
            return argv[i + 1];
        }
    }
    return defaultValue;
}

// Get an integer argument from command line
int getArg(int argc, char **argv, const std::string &flag, int defaultValue)
{
    std::string value = getArg(argc, argv, flag, "");
    if (value.empty())
    {
        return defaultValue;
    }
    return std::stoi(value);
}

// Split a comma-separated string into a vector of strings
std::vector<std::string> splitString(const std::string &str, char delimiter = ',')
{
    std::vector<std::string> result;
    size_t start = 0;
    size_t end = str.find(delimiter);

    while (end != std::string::npos)
    {
        result.push_back(str.substr(start, end - start));
        start = end + 1;
        end = str.find(delimiter, start);
    }

    result.push_back(str.substr(start));
    return result;
}

// Get a vector of strings from a comma-separated command line argument
std::vector<std::string> getArgVector(int argc, char **argv, const std::string &flag,
                                      const std::string &defaultValueStr)
{
    std::string value = getArg(argc, argv, flag, defaultValueStr);
    return splitString(value);
}
