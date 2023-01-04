//
//  i2c_buffer.h
//  
//
//  Created by Samuel Schweighart on 1/3/23.
//

#ifndef i2c_buffer_h
#define i2c_buffer_h

class I2CBuffer {
public:
    I2CBuffer() = default;
    
    ~I2CBuffer() = default;
    

    // May be called inside or outside the ISR.
    // This should be safe because it can never be called while
    // the ISR is trying to read from the buffer or write to it.
    inline void initialize(size_t new_size) {
        reset();
        size = new_size;
        read_index = 0;
        write_index = 0;
    }

    // Empties the buffer.
    inline void reset() {
        read_index = 0;
        write_index = 0;
    }

    //Write
    
    // Returns 'false' if the buffer was already full.
    inline bool write(uint16_t data) {
        if (write_index == size) {
            return false;
        } else {
            buffer[write_index++] = data;
            return true;
        }
    }

    inline size_t get_bytes_written() {
        return write_index;
    }
    
    inline size_t write_bytes_remaining() {
        return size - write_index;
    }
    
    inline bool finished_writing() {
        return write_index == size;
    }

    //Read
    // Caller is responsible for preventing a read beyond the end of the buffer.
    inline uint16_t read() {
        if (read_index == size) {
            return 0;
        }
        return buffer[read_index++];
    }

    inline size_t get_bytes_read() {
        return write_index;
    }

    inline bool finished_reading() {
        return read_index == size;
    }

private:
    volatile uint16_t buffer[256];
    volatile size_t size = 0;
    volatile size_t read_index = 0;
    volatile size_t write_index = 0;
};






#endif /* i2c_buffer_h */
