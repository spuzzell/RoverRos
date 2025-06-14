#ifndef ESP_WHEEL_VOID_H
#define ESP_WHEEL_VOID_H

#include <string>

class Wheel_void
{
    public:

    std::string name = "";

    Wheel_void() = default;

    Wheel_void(const std::string &wheel_name){setup(wheel_name);}

    
    void setup(const std::string &wheel_name){name = wheel_name;}
};

#endif // ESP_WHEEL_VOID_H