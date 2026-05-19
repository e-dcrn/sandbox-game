#pragma once

#include <iostream>

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
