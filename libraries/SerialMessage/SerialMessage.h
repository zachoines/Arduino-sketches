#include "Arduino.h"

#ifndef SerialMessage_h
#define SerialMessage_h
#define SLEEP 10
#define SIZE 10
#define START 0xFF
#define END 0xFF


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
        DOUBLE,
        LONG  
    };

    enum DIR { 
        OUTGOING, 
        INCOMING 
    };

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
        sizeof(double),
        sizeof(long int)
    }; 

  private:

    union byte_converter {
        uint8_t bytes[sizeof(double)];
        char c;
        short int s;
        int i;
        float f;
        double d;
        long int l;
    } converter;
    
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
    static int compare(const void* left, const void* right);
    bool valid(int id);
    bool get_new_message(uint8_t read_buffer[]);

    // Wire functions
    int peek();
    int read();
    int read(uint32_t timeout);
    int read_n(int cnt, uint8_t buffer[]);
    int write(uint8_t byte);
    int write_n(int cnt, uint8_t buffer[]);
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