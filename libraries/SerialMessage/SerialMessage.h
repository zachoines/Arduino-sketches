#include "Arduino.h"

#ifndef SerialMessage_h
#define SerialMessage_h


#include <limits.h>
#include <Stream.h>
#include <HardwareSerial.h>
#include "LinkedList.h"

#ifdef __AVR__
	#include <SoftwareSerial.h>
#endif



class SerialMessage {
public:  
    enum TYPE { 
        CHAR, 
        SHORT, 
        INT, 
        FLOAT, 
        DOUBLE  
    };
    enum DIR { OUTGOING, INCOMING };
    typedef struct MessageConfig {
        const char* name;
        TYPE type;
        DIR dir;
    } MessageConfig;

    typedef struct Message {
        uint32_t timestamp;
        void* value;
        MessageConfig config;
    } Message;

    static constexpr const char sizes[] = { 
        sizeof(char), 
        sizeof(short int), 
        sizeof(int), 
        sizeof(float), 
        sizeof(double) 
    }; 

  private:

    // Union structures
    union short_to_bytes {
        uint8_t bytes[sizes[TYPE::SHORT]];
        short value;
    } short_converter;

    union int_to_bytes {
        uint8_t bytes[sizes[TYPE::INT]];
        int value;
    } int_converter;

    union float_to_bytes {
        uint8_t bytes[sizes[TYPE::FLOAT]];
        float value;
    } float_converter;

    union double_to_bytes {
        uint8_t bytes[sizes[TYPE::DOUBLE]];
        double value;
    } double_converter;
    
    // Variables
    bool debug;
	uint32_t timeout;
	uint8_t max_retry;
	HardwareSerial *hserial;
#ifdef __AVR__
	SoftwareSerial *sserial;
#endif
    LinkedList<MessageConfig> message_configs;
    LinkedList<Message> messages;
    LinkedList<String> names;
    LinkedList<int> ids;

    // Utility functions
    void* from_bytes(uint8_t buffer[], TYPE type);
    void to_bytes(uint8_t buffer[], TYPE type, void* value);
    static int compare(const void* left, const void* right);

    // Wire functions
    int read();
    int read(uint32_t timeout);
    int read_n(uint8_t cnt, uint8_t buffer[]);
    int write(uint8_t byte);
    int write_n(uint8_t cnt, uint8_t buffer[]);
    void clear();
    void flush();
    
    
  public:

    SerialMessage(HardwareSerial *hserial, uint32_t tout, MessageConfig messages[], int num_messages, bool debug=true);
#ifdef __AVR__
    SerialMessage(SoftwareSerial *sserial, uint32_t tout, MessageConfig messages[], int num_messages, bool debug=true);
#endif
    ~SerialMessage();
    void begin(long speed);
    bool set(String name, void* value);
    bool sync();
    Message get(String name);

};

#endif