#ifndef ESP_COMMS_H
#define ESP_COMMS_H

#include <serial/serial.h>
#include <sstream>
//#include <libserial/SerialPort.h>
#include <iostream>
#include <cstring>

class ESPComms
{
    public:

    ESPComms() = default;


    void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
    {
        timeout_ms_ = timeout_ms;
        serial_device_ = serial_device;
        baud_rate_ = baud_rate;


        serial::Timeout timeout = serial::Timeout::simpleTimeout(timeout_ms);
        serial_conn_.setPort(serial_device);
        serial_conn_.setBaudrate(baud_rate);
        serial_conn_.setTimeout(timeout);
        serial_conn_.open();
    }

    bool connected() const { return serial_conn_.isOpen(); }

    void disconnect() {if (serial_conn_.isOpen()){serial_conn_.close();}}

 

 

    void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);




    void sendEmptyMsg();
    void readEncoderValues(int &val_1, int &val_2);
    void setMotorValues(int val_1, int val_2);
    void setPidValues(float k_p, float k_d, float k_i, float k_o);

    

    std::string sendMsg(const std::string &msg_to_send, bool print_output = false);


    private:
    serial::Serial serial_conn_;  ///< Underlying serial connection 
    std::string serial_device_;    ///< Serial device name
    int32_t baud_rate_;            ///< Baud rate for the serial connection
    int32_t timeout_ms_;           ///< Timeout for the serial connection in milliseconds
};

#endif // ESP_COMMS_H