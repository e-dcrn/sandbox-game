#pragma once

#include <iostream>

// A simple binary output archive for stringstream that entt can use.
class OArchive {
public:
    OArchive(std::ostream& stream) : stream(stream) {}

    template<typename T>
    void operator()(const T& value) {
        stream.write(reinterpret_cast<const char*>(&value), sizeof(T));
    }

private:
    std::ostream& stream;
};

// A simple binary input archive for stringstream that entt can use.
class IArchive {
public:
    IArchive(std::istream& stream) : stream(stream) {}

    template<typename T>
    void operator()(T& value) {
        stream.read(reinterpret_cast<char*>(&value), sizeof(T));
    }

private:
    std::istream& stream;
};
