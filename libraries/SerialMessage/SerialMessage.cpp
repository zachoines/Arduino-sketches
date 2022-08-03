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
    max_retry = 2;

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

void* SerialMessage::from_bytes(uint8_t buffer[], TYPE type) 
{

    switch (type)
    {
    case TYPE::SHORT:
        for (int i = 0; i < sizes[type]; i++) {
            short_converter.bytes[i] = buffer[i];
        }
        return (void*) short_converter.value;
    
    default:
        return (void*)NULL;
    }
}

void SerialMessage::to_bytes(uint8_t buffer[], TYPE type, void* value) 
{
    switch (type)
    {
    case TYPE::SHORT:
        short_converter.value = *(short*)value;
        for (int i = 0; i < sizes[type]; i++) {
            buffer[i] = short_converter.bytes[i];
        }
    default:
        break;
    }
    
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
	if(hserial)
		return hserial->read();
#ifdef __AVR__
	else
		return sserial->read();
#endif
}

int SerialMessage::read(uint32_t timeout)
{
	if(hserial){
		uint32_t start = micros();
		// Empty buffer?
		while(!hserial->available()){
		   if((micros()-start)>=timeout)
		      return -1;
		}
		return hserial->read();
	}
#ifdef __AVR__
	else{
		if(sserial->isListening()){
			uint32_t start = micros();
			// Empty buffer?
			while(!sserial->available()){
			   if((micros()-start)>=timeout)
				  return -1;
			}
			return sserial->read();
		}
	}
#endif
}

int SerialMessage::read_n(uint8_t cnt, uint8_t buffer[]) {

    if(hserial){
        if (hserial->available() >= cnt) {
            return hserial->readBytes(buffer, cnt);
        }
    } else {
        // TODO:: implement Software serial
    }
}

int SerialMessage::write(uint8_t byte)
{
	if(hserial)
		return hserial->write(byte);
#ifdef __AVR__
	else
		return sserial->write(byte);
#endif
}

int SerialMessage::write_n(uint8_t cnt, uint8_t buffer[])
{

    int bytes_written = 0;
    if (hserial)
        bytes_written = hserial->write(buffer, cnt);
#ifdef __AVR__
    else 
        for (uint8_t index = 0; index < cnt; index++) {
            bytes_written += write(buffer[index]);
        }        
#endif   
    
    return bytes_written;
}

bool SerialMessage::set(String name, void* value) 
{
    int index = names.Search(name);
    int id = *ids[index];
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

    switch (config.type)
    {
    case TYPE::SHORT:
        short int data;
        data = *(short int *)value; 
        if (debug) {
            char string_buffer[120];
            snprintf(string_buffer, sizeof(string_buffer),
                    "NOTE: Sending value %i for message %s to receiver",
                    data,
                    config.name);
            Serial.println(string_buffer);
        }

        uint8_t data_buffer[sizes[TYPE::SHORT]];
        to_bytes(data_buffer, config.type, value);
        write(id);
        write_n(sizes[TYPE::SHORT], data_buffer);

        return true;
        break;
    
    default:
        if (debug) {
            Serial.print("WARNING: Message data type is doesnt exist, Please verify messsage configs for: ");
            Serial.println(name);
        }
        return false;
        break;
    }
}

bool SerialMessage::sync() {

    if(hserial){
		uint32_t start = micros();
		
        // Check for empty buffer
		while(!hserial->available()){
		   if((micros()-start)>=timeout) {
                if (debug) {
                    Serial.println("NOTE: Timed out while listening for messages");
                }
		        return false;
           }
		}
		
        uint8_t id = read();
        int index = ids.Search(id);
        MessageConfig config = *message_configs[index];
        
        if (index == -1) {
            if (debug) {
                Serial.println("WARNING: Received incompleate message. Flushing....");
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
        case TYPE::SHORT:
            short int data;
            read_n(sizes[TYPE::SHORT], short_converter.bytes);
            data = short_converter.value; 
            
            if (debug) {
                char string_buffer[120];
                snprintf(string_buffer, sizeof(string_buffer),
                        "NOTE: Receiving value %i for message %s to receiver",
                        data,
                        config.name);
                Serial.println(string_buffer);
            }

            return true;
            break;
        
        default:
            if (debug) {
                Serial.print("WARNING: Message data type is doesnt exist, Please verify messsage configs for: ");
                Serial.println(config.name);
            }
            return false;
            break;
        }
	}
}

SerialMessage::Message SerialMessage::get(String name) {
    Message message;
    return message;
}


