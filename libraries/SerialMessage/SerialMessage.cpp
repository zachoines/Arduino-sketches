#include "Arduino.h"
#include "SerialMessage.h"



SerialMessage::SerialMessage(HardwareSerial *hserial, uint32_t tout, MessageConfig messages[], int num_messages, bool debug) 
{
    /*
        @desc Constructor
        @param hserial Hardware serial port reference.
        @param tout Timeout of requests
        @param messages Config of messages
        @param num_messages Number of defined messages. Currently support up to 256 unique message types
    */
    this->debug = debug;
    timeout = tout;
	this->hserial = hserial;
    max_retry = 100;
    

    qsort(messages, num_messages, sizeof(MessageConfig), compare);

    for (int i = 0; i < num_messages; i++) {
        MessageConfig config = messages[i];
        
        if (i >= UCHAR_MAX) {
            Serial.print("WARNING: Exceeded maximum message types, truncating ");
            Serial.println(String(config.name));
            continue;
        }
        void* default_value = 0;
        this->message_configs.Append(config);
        this->names.Append(String(config.name));
        this->ids.Append(i);
        this->messages.Append({
            micros(),
            default_value,
            config
        });
    }

#ifdef __AVR__
	sserial = 0;
#endif
}


#ifdef __AVR__
SerialMessage::SerialMessage(SoftwareSerial *sserial, uint32_t tout, MessageConfig messages[], int num_messages, bool debug) 
{
    this->debug = debug;
    timeout = tout;
	this->sserial = sserial;
    max_retry = 2;
	hserial = 0;
}
#endif

SerialMessage::~SerialMessage() {

}

int SerialMessage::compare(const void * left, const void * right) 
{
    const MessageConfig * a = (const MessageConfig *) left;
    const MessageConfig * b = (const MessageConfig *) right;
    return String(a->name).compareTo(String(b->name));
}

void SerialMessage::begin(long speed) 
{
	if(hserial){
		hserial->begin(speed);
	}
#ifdef __AVR__
	else{
		sserial->begin(speed);
	}
#endif
}

bool SerialMessage::valid(int id) 
{    
    int index = ids.Search(id);
    if (index > -1) {
        return true;
    } else {
        return false;
    }
}

int SerialMessage::peek()
{
    if(hserial){
		return hserial->peek();
	} else {
        return -1;
    }
}

void SerialMessage::clear()
{
	if(hserial){
		while(hserial->available())
			hserial->read();
	}
#ifdef __AVR__
	else{
		while(sserial->available())
			sserial->read();
	}
#endif
}

void SerialMessage::flush()
{
	if(hserial)
		hserial->flush();
}

int SerialMessage::read()
{
	if(hserial) {
        unsigned long start = micros();
        while (hserial->available() < 1) {
            if((micros()-start)>=timeout) {
                return -1;
            }
        }
		return hserial->read();
    }
#ifdef __AVR__
	else {
		return sserial->read();
    }
#endif
}

int SerialMessage::read_n(int cnt, uint8_t buffer[]) {

    int bytes_read = 0;
    if(hserial){
        // return hserial->readBytes(buffer, cnt);
        for (int i = 0; i < cnt; i++) {
            int incomming_byte = read();
            if (incomming_byte > -1) {
                buffer[i] = static_cast<uint8_t>(incomming_byte);
                bytes_read++;
            } 
        }
    } else {
        // TODO:: implement Software serial
    }

    return bytes_read;
}

int SerialMessage::write(uint8_t byte)
{
	if(hserial) {
        return hserial->write(byte);
    }
#ifdef __AVR__
	else {
		return sserial->write(byte);
    }
#endif
}

int SerialMessage::write_n(int cnt, uint8_t buffer[])
{

    int bytes_written = 0;
    if (hserial) {
        // return hserial->write(buffer, cnt);
        // flush();
        for (int i = 0; i < cnt; i++) {
            bytes_written += write(buffer[i]);
        } 
    }
#ifdef __AVR__
    else { 
        for (int index = 0; index < cnt; index++) {
            bytes_written += write(buffer[index]);
        } 
    }       
#endif   
    
    return bytes_written;
}


bool SerialMessage::get_new_message(uint8_t read_buffer[])
{
    unsigned long start = micros();
    while (hserial->available() < SIZE) {
        if((micros()-start)>=timeout) {
            if (debug)
                Serial.println("WARNING: Timed out while waiting for message.");
        
            return false;
        }
    }

    if (!(peek() == START)) {
        if (debug)
            Serial.println("WARNING: Message without valid start byte.");
        
        read();
        return false;
    }

    if (read_n(SIZE, read_buffer) < SIZE) { 
        if (debug)
            Serial.println("WARNING: Received incomplete message.");
    
        return false;
    }

    if (!valid(static_cast<int>(read_buffer[1]))) {
        if (debug)
            Serial.println("WARNING: Message without valid id.");
    
        return false;
    }

    return true;
}

bool SerialMessage::set(String name, void* value) 
{
    converter.d = 0;
    int index = names.Search(name);
    int id = *ids[index];
    int bytes_written = 0;
    uint8_t write_buffer[SIZE]; // [ type, .... data bytes .... ]
    
    MessageConfig config = *message_configs[index];

    if (index == -1) {
        if (debug) {
            Serial.print("WARNING: Message doesnt exist for ");
            Serial.println(name);
        }
        return false;
    } else if (config.dir != DIR::OUTGOING) {
        if (debug) {
            Serial.print("WARNING: Message direction type is not 'OUTGOING', Please verify messsage configs for ");
            Serial.println(name);
        }
        return false;
    }
    

    write_buffer[0] = START;
    write_buffer[1] = static_cast<uint8_t>(id); 

    switch (config.type)
    {
    case TYPE::CHAR:
        char data_char;
        data_char = *(char *)value; 
        converter.c = data_char;
        for (int j = 0; j < sizes[TYPE::CHAR]; j++) {
            write_buffer[j + 2] = converter.bytes[j];
        }
        if (debug) {
            char string_buffer[120];
            snprintf(string_buffer, sizeof(string_buffer),
                    "NOTE: Sending char value %c for message %s to receiver",
                    data_char,
                    config.name);
            Serial.println(string_buffer);
        }

        break;

    case TYPE::SHORT:
        short int data_short;
        data_short = *(short int *)value; 
        converter.s = data_short;
        for (int j = 0; j < sizes[TYPE::SHORT]; j++) {
            write_buffer[j + 2] = converter.bytes[j];
        }
        if (debug) {
            char string_buffer[120];
            snprintf(string_buffer, sizeof(string_buffer),
                    "NOTE: Sending short value %i for message %s to receiver",
                    data_short,
                    config.name);
            Serial.println(string_buffer);
        }

        break;
    
    case TYPE::INT:
        int data_int;
        data_int = *(int *)value; 
        converter.i = data_int;
        for (int j = 0; j < sizes[TYPE::INT]; j++) {
            write_buffer[j + 2] = converter.bytes[j];
        }
        if (debug) {
            char string_buffer[120];
            snprintf(string_buffer, sizeof(string_buffer),
                    "NOTE: Sending int value %i for message %s to receiver",
                    data_int,
                    config.name);
            Serial.println(string_buffer);
        }

        break;

    case TYPE::FLOAT:
        float data_float;
        data_float = *((float *)value);
        converter.f = data_float; 
        for (int j = 0; j < sizes[TYPE::FLOAT]; j++) {
            write_buffer[j + 2] = converter.bytes[j];
        }

        if (debug) {
            char string_buffer[120];
            snprintf(string_buffer, sizeof(string_buffer),
                    "NOTE: Sending float value %f for message %s to receiver",
                    data_float,
                    config.name);
            
            Serial.println(string_buffer);
        }

        break;

    case TYPE::DOUBLE:
        double data_double;
        data_double = *(double *)value; 
        converter.d = data_double;
        for (int j = 0; j < sizes[TYPE::DOUBLE]; j++) {
            write_buffer[j + 2] = converter.bytes[j];
        }
        if (debug) {
            char string_buffer[120];
            snprintf(string_buffer, sizeof(string_buffer),
                    "NOTE: Sending double value %f for message %s to receiver",
                    data_double,
                    config.name);
            Serial.println(string_buffer);
        }

        break;

    case TYPE::LONG:
        long int data_long;
        data_long = *(long int *)value; 
        converter.l = data_long;
        for (int j = 0; j < sizes[TYPE::LONG]; j++) {
            write_buffer[j + 2] = converter.bytes[j];
        }
        if (debug) {
            char string_buffer[120];
            snprintf(string_buffer, sizeof(string_buffer),
                    "NOTE: Sending long value %ld for message %s to receiver",
                    data_long,
                    config.name);
            Serial.println(string_buffer);
        }
        
        break;
    
    default:
        if (debug) {
            Serial.print("WARNING: Message data type doesn't exist, Please verify messsage configs for: ");
            Serial.println(name);
        }
        
        return false;
    }

    if (write_n(SIZE, write_buffer) == SIZE) {
        return true;
    } else {
        return false;
    }
}

bool SerialMessage::sync() {

    converter.l = 0;
    int id;
    int bytes_read;
    int index;
    uint8_t read_buffer[SIZE] = { 0 };

    if(hserial){
        if (!get_new_message(read_buffer))
            return false;

        id = static_cast<int>(read_buffer[1]);        
        index = ids.Search(id);

        Message *message = messages[index];
        MessageConfig config = message->config;
        message->timestamp = micros();
        
        if (index == -1) {
            if (debug) {
                Serial.println("WARNING: Received incomplete message. Flushing....");
                clear();
            }
            
            return false;
        }

        if (config.dir != DIR::INCOMING) {
            if (debug) {
                Serial.print("WARNING: Message direction type is not 'INCOMING', Please verify messsage configs for ");
                Serial.println(config.name);
            } 
            
            return false;
        }

        switch (config.type)
        {
        case TYPE::CHAR:
            char data_char;
            for (int j = 0; j < sizes[TYPE::CHAR]; j++) {
                converter.bytes[j] = read_buffer[j + 2];
            }

            data_char = converter.c; 
            *(char*) message->value = data_char;
            
            if (debug) {
                char string_buffer[120];
                snprintf(string_buffer, sizeof(string_buffer),
                        "NOTE: Receiving char value %c for message %s to receiver",
                        data_char,
                        config.name);
                Serial.println(string_buffer);
            }
            
            break;

        case TYPE::SHORT:
            short int data_short;
            for (int j = 0; j < sizes[TYPE::SHORT]; j++) {
                converter.bytes[j] = read_buffer[j + 2];
            }

            data_short = converter.s; 
            *(short int*) message->value = data_short;
            
            if (debug) {
                char string_buffer[120];
                snprintf(string_buffer, sizeof(string_buffer),
                        "NOTE: Receiving short value %i for message %s to receiver",
                        data_short,
                        config.name);
                Serial.println(string_buffer);
            }
            
            break;
        
        case TYPE::INT:
            int data_int;
            for (int j = 0; j < sizes[TYPE::INT]; j++) {
                converter.bytes[j] = read_buffer[j + 2];
            }

            data_int = converter.i; 
            *(int*) message->value = data_int;
            
            if (debug) {
                char string_buffer[120];
                snprintf(string_buffer, sizeof(string_buffer),
                        "NOTE: Receiving short value %i for message %s to receiver",
                        data_int,
                        config.name);
                Serial.println(string_buffer);
            }
            
            break;
            
        case TYPE::FLOAT:
            float data_float;
            for (int j = 0; j < sizes[TYPE::FLOAT]; j++) {
                converter.bytes[j] = read_buffer[j + 2];
            }

            data_float = converter.f; 
            *(float*) message->value = data_float;

            if (debug) {
                Serial.print("NOTE: Receiving float value ");
                Serial.print(data_float);
                Serial.print(" for message ");
                Serial.print(config.name);
                Serial.println(" to receiver");
            }

            break;

        case TYPE::DOUBLE:
            double data_double;
            for (int j = 0; j < sizes[TYPE::DOUBLE]; j++) {
                converter.bytes[j] = read_buffer[j + 2];
            }

            data_double = converter.d; 
            *(double*) message->value = data_double;

            if (debug) {
                Serial.print("NOTE: Receiving double value ");
                Serial.print(data_double);
                Serial.print(" for message ");
                Serial.print(config.name);
                Serial.println(" to receiver");
            }

            break;

        case TYPE::LONG:
            long int data_long;
            for (int j = 0; j < sizes[TYPE::LONG]; j++) {
                converter.bytes[j] = read_buffer[j + 2];
            }

            data_long = converter.l; 
            *(long int*) message->value = data_long;
            
            if (debug) {
                char string_buffer[120];
                snprintf(string_buffer, sizeof(string_buffer),
                        "NOTE: Receiving long value %ld for message %s to receiver",
                        data_long,
                        config.name);
                Serial.println(string_buffer);
            }
            
            break;

        default:
            if (debug) {
                Serial.print("WARNING: Message data type is doesnt exist, Please verify messsage configs for: ");
                Serial.println(config.name);
            }
            
            return false;
        }
        
        return true;
	}
}

SerialMessage::Message SerialMessage::get(String name) {
    int index = names.Search(name);
    Message message;
    if (index == -1) {
        if (debug) {
            Serial.print("WARNING: Message doesnt exist for ");
            Serial.println(name);
        }
        return message;
    }
    
    return *messages[index];
}