// Copyright © 2019-2020 Richard Gemmell
// Released under the MIT License. See license.txt. (https://opensource.org/licenses/MIT)
//
// Fragments of this code copied from WireIMXRT.cpp © Paul Stoffregen.
// Please support the Teensy project at pjrc.com.

//#define DEBUG_I2C // Uncomment to enable debug tools
#ifdef DEBUG_I2C
#pragma clang diagnostic push
#pragma ide diagnostic ignored "hicpp-signed-bitwise"
#include <Arduino.h>
#endif
#include <Arduino.h>		
#include <imxrt.h>
#include <pins_arduino.h>
#include "imx_rt1060_i2c_driver.h"

#define DUMMY_BYTE 0x00 // Used when there's no real data to write.
#define NUM_FIFOS 4     // Number of Rx and Tx FIFOs available to master
#define MASTER_READ 1   // Makes the address a read request
#define MASTER_WRITE 0  // Makes the address a write request
#define MAX_MASTER_READ_LENGTH 256  // Maximum number of bytes that can be read by a single Master read
#define CLOCK_STRETCH_TIMEOUT 15000 // Timeout if a device stretches SCL this long, in microseconds

// Debug tools
#ifdef DEBUG_I2C
static void log_slave_status_register(uint32_t ssr);
static void log_master_status_register(uint32_t msr);
static void log_master_control_register(const char* message, uint32_t mcr);
#endif

//static uint8_t empty_buffer[0];



// NXP document the pad configuration in AN5078.pdf Rev 0.
// https://www.nxp.com/docs/en/application-note/AN5078.pdf
//
// DSE - (Drive Strength Enable) - See AN5078.pdf section 7.1
// Used to tune the output driver's impedance to match the load impedance.
// If the impedance is too high the circuit behaves like a low pass filter,
// rounding off the inputs.
// Suggested Value: IOMUXC_PAD_DSE(4) // 37 Ohm
//
// SPEED - See AN5078.pdf section 7.3
// "These options can either increase the output driver current in the higher
// frequency range, or reduce the switching noise in the lower frequency range."
// Suggested Value: IOMUXC_PAD_SPEED(1) // 100MHz
//
// SRE - Slew Rate Field - See AN5078.pdf section 7.2
// NXP recommend that this option should be disabled for I2C
// Suggested Value: disabled. Add IOMUXC_PAD_SRE to enable
//
// Pull/keeper - See AN5078.pdf sections 6 and 8.2.2
// AN5078 recommends disabling pullup except for "very low clock frequencies"
// Suggested Value: IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3) // Enable 22k pullup resistor
//
// Enable Open Drain - See AN5078.pdf sections 5.2
// Recommended for I2C as I2C depends on open drain drivers
// Suggested Value: IOMUXC_PAD_ODE  // Enabled
//
// Hysteresis - See AN5078.pdf section 7.2
// Reduces the number of false input signal changes (glitches) caused by noise
// near the centre point of the voltage range. Enabling this option on "slightly
// increases the pin power consumption as well as the propagation delay by
// several nanoseconds." (From AN5078)
// Suggested Value: IOMUXC_PAD_HYS  // Enabled
#define PAD_CONTROL_CONFIG \
    IOMUXC_PAD_DSE(4) \
    | IOMUXC_PAD_SPEED(1) \
    | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3) \
    | IOMUXC_PAD_ODE \
    | IOMUXC_PAD_HYS
I2CDriver::I2CDriver()
    : pad_control_config(PAD_CONTROL_CONFIG) {
}

static void initialise_pin(IMX_RT1060_I2CBase::PinInfo pin, uint32_t pad_control_config) {
    *(portControlRegister(pin.pin)) = pad_control_config;
    #ifdef DEBUG_I2C
    Serial.print("Pad control register: 0x");
    Serial.println(*(portControlRegister(pin.pin)), 16);
    #endif

    *(portConfigRegister(pin.pin)) = pin.mux_val;
    if (pin.select_input_register) {
        *(pin.select_input_register) = pin.select_val;
    }
}

static void initialise_common(IMX_RT1060_I2CBase::Config hardware, uint32_t pad_control_config) {
    // Set LPI2C Clock to 24MHz. Required by slaves as well as masters.
    CCM_CSCDR2 = (CCM_CSCDR2 & ~CCM_CSCDR2_LPI2C_CLK_PODF(63)) | CCM_CSCDR2_LPI2C_CLK_SEL;
    hardware.clock_gate_register |= hardware.clock_gate_mask;

    // Setup SDA and SCL pins and registers
    initialise_pin(hardware.sda_pin, pad_control_config);
    initialise_pin(hardware.scl_pin, pad_control_config);
}

static void stop(IMXRT_LPI2C_Registers* port, IRQ_NUMBER_t irq) {
    // Halt and reset Master Mode if it's running
    port->MCR = (LPI2C_MCR_RST | LPI2C_MCR_RRF | LPI2C_MCR_RTF);
    port->MCR = 0;

    // Halt and reset Slave Mode if it's running
    port->SCR = (LPI2C_SCR_RST | LPI2C_SCR_RRF | LPI2C_SCR_RTF);
    port->SCR = 0;

    // Disable interrupts
    NVIC_DISABLE_IRQ(irq);
    attachInterruptVector(irq, nullptr);
}

IMX_RT1060_I2CMaster::IMX_RT1060_I2CMaster(IMXRT_LPI2C_Registers* port, IMX_RT1060_I2CBase::Config& config, void (* isr)())
        : port(port), config(config), isr(isr) {
}

void IMX_RT1060_I2CMaster::begin(uint32_t frequency) {
    // Make sure master mode is disabled before configuring it.
    stop(port, config.irq);

    // Setup pins and master clock
    initialise_common(config, pad_control_config);

    // Configure and Enable Master Mode
    // Set FIFO watermarks. Determines when the RDF and TDF interrupts happen
    port->MFCR = LPI2C_MFCR_RXWATER(0) | LPI2C_MFCR_TXWATER(0);
    set_clock(frequency);
    // Setup interrupt service routine.
    attachInterruptVector(config.irq, isr);
    port->MIER = LPI2C_MIER_RDIE | LPI2C_MIER_SDIE | LPI2C_MIER_NDIE | LPI2C_MIER_ALIE | LPI2C_MIER_FEIE | LPI2C_MIER_PLTIE; //SDIE seems to trigger a second interupt call...
  //  port->MIER = LPI2C_MIER_RDIE | LPI2C_MIER_NDIE | LPI2C_MIER_ALIE | LPI2C_MIER_FEIE | LPI2C_MIER_PLTIE;
    NVIC_ENABLE_IRQ(config.irq);
}

void IMX_RT1060_I2CMaster::end() {
    stop(port, config.irq);
}

inline bool IMX_RT1060_I2CMaster::isFinished() {
    //We could say we are finished after transfer complete, but the problem with that is, if we try to start another I2C call, there is still the STOP command in the FIFO, so you get an error. (Cause our code checks to ensure FIFO is empty before putting items in the Tx Fifo.
    return state >= State::idle;
}

bool IMX_RT1060_I2CMaster::waitUntilFinished(float timeoutLimitMills) {
    elapsedMillis timeout;
    while (timeout < timeoutLimitMills) {
        if (isFinished()){
            return true;
        }
    }
    Serial.println("Master: ERROR timed out waiting for transfer to finish.");
    return false;
}


bool IMX_RT1060_I2CMaster::ok(const char* message) {
    if (has_error()) {
        Serial.print(message);
        Serial.print(" Error: ");
        Serial.println((int)error());
        return false;
    }
    return true;
}


// --- COMMAND BUFFERING ---

// -- INITIALIZE TRANSFER --
// Run this code first.  It first checks to see if the I2C system is idle.
// If so, it clears the read and command buffers and sets the state.

bool IMX_RT1060_I2CMaster::initialize_transfer(bool forceRestartOnFailure) {
    if (state != State::idle) {
        Serial.print("I2C - Attempted to initialize.  State is:");
        Serial.println((uint) state);
        if (!forceRestartOnFailure)
            return false;
        if (!abort_transaction_async())
            return false;
    }
    _error = I2CError::ok;
    state = State::initialized;
    readBuffer.reset();
    commandBuffer.reset();
}


// -- READ AT REGISTER --
// This function will add the commands to read a number of bytes (num_bytes) from a specified starting register (inital_register) from the I2C device at (i2c_address)
bool IMX_RT1060_I2CMaster::add_read_at_register_to_command_buffer(uint8_t i2c_address, uint8_t initial_register, size_t num_bytes) {
    if (state != State::initialized && state != State::loading) {
        Serial.print("I2C - Attempted to add to command buffer, but not idle/loading.  State is:");
        Serial.println((uint)state);
        return false;
    }
   
    if (commandBuffer.write_bytes_remaining() - 1 < 4) {
        //Need to leave one command open to add the STOP at the end.
        Serial.print("I2C - Not enough room left in buffer:");
        Serial.println(commandBuffer.write_bytes_remaining());
        return false;
    }
    
    state = State::loading;
    commandBuffer.write(LPI2C_MTDR_CMD_START    | i2c_address       | MASTER_WRITE);
    commandBuffer.write(LPI2C_MTDR_CMD_TRANSMIT | initial_register                );
    commandBuffer.write(LPI2C_MTDR_CMD_START    | i2c_address       | MASTER_READ );
    commandBuffer.write(LPI2C_MTDR_CMD_RECEIVE  | (num_bytes - 1)                 );
    return true;
    
}

// -- READ --
// This function will add the commands to read a number of bytes (num_bytes) from the I2C device at (i2c_address)
// This does NOT specify a starting register as some I2C devices do not require it.
bool IMX_RT1060_I2CMaster::add_read_to_command_buffer(uint8_t i2c_address, size_t num_bytes) {
    if (state != State::initialized && state != State::loading) {
        Serial.print("I2C - Attempted to add to command buffer, but not idle/loading.  State is:");
        Serial.println((uint)state);
        return false;
    }
    
    if (commandBuffer.write_bytes_remaining() - 1 < 2) {
        //Need to leave one command open to add the STOP at the end.
        Serial.print("I2C - Not enough room left in buffer:");
        Serial.println(commandBuffer.write_bytes_remaining());
        return false;
    }
    
    state = State::loading;
    commandBuffer.write(LPI2C_MTDR_CMD_START    | i2c_address       | MASTER_READ );
    commandBuffer.write(LPI2C_MTDR_CMD_RECEIVE  | (num_bytes - 1)                 );
    return true;
    
}

// -- WRITE AT REGISTER --
// This function will add the commands to write a number of bytes (num_bytes) from an array (buffer) starting at register (intial_register) to the I2C device at (i2c_address)
// This does NOT specify a starting register as some I2C devices do not require it.
bool IMX_RT1060_I2CMaster::add_write_at_register_to_command_buffer(uint8_t i2c_address, uint8_t initial_register, size_t num_bytes, const uint8_t* dataToWrite) {
    if (state != State::initialized && state != State::loading) {
        Serial.print("I2C - Attempted to add to command buffer, but not idle/loading.  State is:");
        Serial.println((uint) state);
        return false;
    }
    
    if (commandBuffer.write_bytes_remaining() - 1 < 2 + num_bytes) {
        //Need to leave one command open to add the STOP at the end.
        Serial.print("I2C - Not enough room left in buffer:");
        Serial.println(commandBuffer.write_bytes_remaining());
        return false;
    }
    
    state = State::loading;
    commandBuffer.write(LPI2C_MTDR_CMD_START    | i2c_address       | MASTER_WRITE);
    commandBuffer.write(LPI2C_MTDR_CMD_TRANSMIT | initial_register                );
    for (size_t index = 0; index < num_bytes; index++) {
        if (commandBuffer.finished_writing()) //Redundent with previous check...
            return false;
        commandBuffer.write(dataToWrite[index]);
    }
    return true;
}

// -- WRITE --
// This function will add the commands to write a number of bytes (num_bytes) from an array (buffer) to the I2C device at (i2c_address)
// This does NOT specify a starting register as some I2C devices do not require it.
bool IMX_RT1060_I2CMaster::add_write_to_command_buffer(uint8_t i2c_address, size_t num_bytes, const uint8_t* dataToWrite) {
    if (state != State::initialized && state != State::loading) {
        Serial.print("I2C - Attempted to add to command buffer, but not idle/loading.  State is:");
        Serial.println((uint) state);
        return false;
    }
    
    if (commandBuffer.write_bytes_remaining() - 1 < 1 + num_bytes) {
        //Need to leave one command open to add the STOP at the end.
        Serial.print("I2C - Not enough room left in buffer:");
        Serial.println(commandBuffer.write_bytes_remaining());
        return false;
    }
    
    state = State::loading;
    commandBuffer.write(LPI2C_MTDR_CMD_START    | i2c_address       | MASTER_WRITE);
    for (size_t index = 0; index < num_bytes; index++) {
        if (commandBuffer.finished_writing())
            return false;
        commandBuffer.write(dataToWrite[index]);
    }
    return true;
}

// -- EXECUTE --
// This function will start the I2C transaction. It is NOT blocking.
// Instead, when complete, the ISR will call the member function (method) I2CCallback from the I2CCallbackClass.
// An I2CCallbackClass object must be provided.  Note I2CCallbackClass is an abstract class, so the object must provide the member function.
// If no callback is required, just pass 'nullptr'
// abortOnFailure is set to true by defaut.  If there is something wrong, you want the code to 'hard' reset everything.
// However, it may just be that your code has looped around and is trying to start another I2C transaction before the previous one finished.
// If abortOnFailure is set to False, and you use async transactions, make sure you somehow set up a timeout to reset if things go poorly.

bool IMX_RT1060_I2CMaster::execute_command_buffer_async(I2CCallbackClass * I2CCallback_ptr, bool abortOnFailure) {
    
    if (state != State::loading) {
        // We haven't completed the previous transaction yet
        Serial.print("I2C - Cannot execute. Wrong state. State: ");
        Serial.print((uint)state);
        if (abortOnFailure) {
            abort_transaction_async();
            _error = I2CError::master_not_ready;
            state = State::idle;
        }
        return false;
    }
    
    // Make sure the FIFOs are empty before we start.
    if (tx_fifo_count() > 0 || rx_fifo_count() > 0) {
        Serial.print("Master: FIFOs not empty in start(). TX: ");
        Serial.print(tx_fifo_count());
        Serial.print(" RX: ");
        Serial.println(rx_fifo_count());
        if (abortOnFailure) {
            abort_transaction_async();
            _error = I2CError::master_fifos_not_empty;
            state = State::idle;
        }
        return false;
    }
    
    if (commandBuffer.write_bytes_remaining()  < 1) {
        Serial.print("I2C - Not enough room left in buffer to add the STOP");
        return false;
    }
    
    //Add the STOP byte
    commandBuffer.write(LPI2C_MTDR_CMD_STOP);
    
    // Start a new transaction
    state = State::starting;
   
    //Setup the callback.
    callback_ptr = I2CCallback_ptr;

    // Clear status flags
    clear_all_msr_flags();
   
    port->MCFGR1 &= ~LPI2C_MCFGR1_AUTOSTOP; //Turn-off Autostop
    port->MCR |= LPI2C_MCR_MEN;  //Master enable
     
    //Put as many bytes into the FIFO
    while (!commandBuffer.finished_reading() && (NUM_FIFOS - tx_fifo_count())) {
        port->MTDR = commandBuffer.read();
    }
    //Turn on the interrupts for the TX and RX buffers.
    port->MIER |= LPI2C_MIER_SDIE; //Stop Byte Detected
    port->MIER |= LPI2C_MIER_TDIE; //TX Buffer
    port->MIER |= LPI2C_MIER_RDIE; //RX Buffer
    
    return true;
    
}

bool IMX_RT1060_I2CMaster::execute_command_buffer_blocking(size_t num_bytes, uint8_t *buffer,  float timeoutLimitMills) {
    if (execute_command_buffer_async(nullptr)) {
        if (waitUntilFinished(timeoutLimitMills)) {
            if (buffer != nullptr){
                for (size_t index = 0; index < num_bytes; index ++)
                    buffer[index] = readBuffer.read();
            }
            return true;
        }
    }
    return false;
}
 
// --- SHORTCUT COMMANDS ---
// -- READ COMMANDS --

//This funciton reads data (num_bytes) at a specified register (initial_register) from the device at (i2c_address) and put it in buffer (buffer).
//The code will block until the data has been returned or timed out.
bool IMX_RT1060_I2CMaster::read_at_register_blocking(uint8_t i2c_address, uint8_t initial_register, size_t num_bytes, uint8_t* buffer) {
    if (initialize_transfer()) {
        if (add_read_at_register_to_command_buffer(i2c_address, initial_register,  num_bytes)) {
            return execute_command_buffer_blocking(num_bytes, buffer);
        }
    }
    return false;
}

//This funciton reads data (num_bytes) at a specified register (initial_register) from the device at (i2c_address).
//This code does not block.  Instead, when the data has been read, the system will call the function I2CCallback located in the I2Ccallback class
//The caller must specify the pointer of the object that is a child of I2Ccallback.
//Note this will be called from an ISR, so I2CCallback must be interupt safe.
bool IMX_RT1060_I2CMaster::read_at_register_async(uint8_t i2c_address, uint8_t initial_register,  size_t num_bytes, I2CCallbackClass * callback_ptr) {
    if (initialize_transfer()) {
        if (add_read_at_register_to_command_buffer(i2c_address, initial_register,  num_bytes)) {
            return execute_command_buffer_async(callback_ptr);
        }
    }
    return false;
}


//This funciton reads data (num_bytes) from the device at (i2c_address) and put it in buffer (buffer).
//The code will block until the data has been returned or timed out.
//This code does NOT first write to the I2C device which register to read from.  Some devices will just re-read the same register, thus saving time per transaction.
bool IMX_RT1060_I2CMaster::read_blocking(uint8_t i2c_address, size_t num_bytes, uint8_t* buffer) {
    if (initialize_transfer()) {
        if (add_read_to_command_buffer(i2c_address,  num_bytes)) {
            return execute_command_buffer_blocking(num_bytes, buffer);
        }
    }
    return false;
}

//This funciton reads data (num_bytes) at a specified register (initial_register) from the device at (i2c_address).
//This code does not block.  Instead, when the data has been read, the system will call the function I2CCallback located in the I2Ccallback class.
//The caller must specify the pointer of the object that is a child of I2Ccallback.
//Note this will be called from an ISR, so I2CCallback must be interupt safe.
//This code does NOT first write to the I2C device which register to read from.  Some devices will just re-read the same register, thus saving time per transaction.

bool IMX_RT1060_I2CMaster::read_async(uint8_t i2c_address, size_t num_bytes, I2CCallbackClass * callback_ptr) {
    if (initialize_transfer()) {
        if (add_read_to_command_buffer(i2c_address,  num_bytes)) {
            return execute_command_buffer_async(callback_ptr);
        }
    }
    return false;
    
}

// -- WRITE COMMANDS --
//This funciton writes data (num_bytes) at a specified register (initial_register) to the device at (i2c_address) from the array (buffer).
//The code will block until the data has been sent or timed out.
bool IMX_RT1060_I2CMaster::write_at_register_blocking(uint8_t i2c_address, uint8_t initial_register, size_t num_bytes, const uint8_t* dataToWrite) {
    if (initialize_transfer()) {
        if (add_write_at_register_to_command_buffer( i2c_address,  initial_register,  num_bytes,  dataToWrite)) {
            return execute_command_buffer_blocking(0, nullptr);
        }
    }
    return false;
}

//This funciton writes data (num_bytes) at a specified register (initial_register) to the device at (i2c_address) from the array (buffer).
//The code will not block.  An optional callback can be provided.
bool IMX_RT1060_I2CMaster::write_at_register_async(uint8_t i2c_address, uint8_t initial_register,  size_t num_bytes, const uint8_t* dataToWrite, I2CCallbackClass * callback_ptr) {
    if (initialize_transfer()) {
        if (add_write_at_register_to_command_buffer( i2c_address,  initial_register,   num_bytes,  dataToWrite)) {
            return execute_command_buffer_async(callback_ptr);
        }
    }
    return false;
}

//This funciton writes data (num_bytes) to the device at (i2c_address) from the array (buffer).
//The code will block until the data has been sent or timed out.
bool IMX_RT1060_I2CMaster::write_blocking(uint8_t i2c_address,size_t num_bytes, const uint8_t* dataToWrite) {
    if (initialize_transfer()) {
        if (add_write_to_command_buffer( i2c_address, num_bytes, dataToWrite)) {
            return execute_command_buffer_blocking(0, nullptr);
        }
    }
    return false;
}

//This funciton writes data (num_bytes) to the device at (i2c_address) from the array (buffer).
//The code will not block.  An optional callback can be provided.
bool IMX_RT1060_I2CMaster::write_async(uint8_t i2c_address,  size_t num_bytes,  const uint8_t* dataToWrite, I2CCallbackClass * callback_ptr) {
    if (initialize_transfer()) {
        if (add_write_to_command_buffer( i2c_address,   num_bytes,dataToWrite)) {
            return execute_command_buffer_async(callback_ptr);
        }
    }
    return false;
}




// Do not call this method directly
void IMX_RT1060_I2CMaster::_interrupt_service_routine() {
    uint32_t msr = port->MSR;
    #ifdef DEBUG_I2C
    Serial.print("ISR: enter: ");
    log_master_status_register(msr);
    #endif

    //NDF = Nack Detection Flag. No one acknolwedged
    //ALF = Arbitration Lost Flag.  Someone pulled the line low unexpectedly
    //FEF = FIFO Error Flag.  Send a Tx data packet withouth a Start
    //PLTF = Pulled Low Timeout Flag.  Pulled too low.
    if (msr & (LPI2C_MSR_NDF | LPI2C_MSR_ALF | LPI2C_MSR_FEF | LPI2C_MSR_PLTF)) {
        //If we were starting, then it is an address nak. (No one answered at that address.
        if (msr & LPI2C_MSR_NDF) {
            port->MSR = LPI2C_MSR_NDF; //Clear NAC flag
            if (state == State::starting) {
                _error = I2CError::address_nak;
            } else {
                _error = I2CError::data_nak;
            }
        }
        //If Arbitration lost.
        if (msr & LPI2C_MSR_ALF) {
            #ifdef DEBUG_I2C
            Serial.println("Master: Arbitration lost");
            #endif
            port->MSR = LPI2C_MSR_ALF; //Clear Flag
            _error = I2CError::arbitration_lost;
        }
        //If FIFO error,
        if (msr & LPI2C_MSR_FEF) {
            #ifdef DEBUG_I2C
            Serial.println("Master: FIFO Error");
            #endif
            port->MSR = LPI2C_MSR_FEF; //Clear Flag
            if (!has_error()) {
                _error = I2CError::master_fifo_error;
            }
            // else FEF was triggered by another error. Ignore it.
        }
        
        //If Pin low time out.... etc.
        if (msr & LPI2C_MSR_PLTF) {
            #ifdef DEBUG_I2C
            Serial.println("Master: Pin low timeout (PLTF)");
            #endif
            port->MSR = LPI2C_MSR_PLTF; //Clear Flag
            _error = I2CError::master_pin_low_timeout;
        }
        
        //Abort!!!
        if (state != State::stopping) {
            state = State::stopping;
            abort_transaction_async();
        }
        // else already trying to end the transaction
    }


    
    if (msr & LPI2C_MSR_RDF) {
        //The RDF flag is true if there is data above the watermark.  The watermark is set at 0, so this flag is true if there is any data in the read FIFO buffer.
        if (state == State::starting)
            state = State::active;
        while (rx_fifo_count() > 0) {
            #ifdef DEBUG_I2C
            uint8_t data = port->MRDR;
            readBuffer.write(data);
            Serial.print(data);
            Serial.print(", ");
            #else
            readBuffer.write(port->MRDR);
            #endif
        }
    }
    
    if (msr & LPI2C_MSR_TDF) {
        //TDF is true when the number of bytes in the FIFO < Watermark. The watermark is set at 0, so this flag is true if there is no data in the TX buffer.
        //If Autostop is on, this may cause problems since we still may have commands in the "Command Buffer"
        //For this reason, we manually (automatically?) add the stop command to the buffer as part of the ExecuteCommandBuffer Member function.
        if (state == State::starting)
            state = State::active;
        if (state == State::active) {
            while (!commandBuffer.finished_reading() && tx_fifo_count() < NUM_FIFOS) {
                port->MTDR = commandBuffer.read();  //Put the data in the FIFO with Tx command
            }
        }
    }
    
    // We have generated a STOP condition.
    // Clear the MSR flags.
    // Turn of the TDIE (Transmit Data Interrupt)
    // For some reason, one more interuppt is still called. Even if TDIE is already off.   If SPIE is off, then you don't get the extra one.
    if (msr & LPI2C_MSR_SDF) {
        port->MIER &= ~LPI2C_MIER_TDIE; // We don't want to handle TDF if we can avoid it. TDF is the low watermark in the TX FIFO
        port->MIER &= ~LPI2C_MIER_SDIE; // If we don't turn of SDIE, we get one more interrupt pass through.
        port->MSR = LPI2C_MSR_SDF;  //Clear Stop Detect Flag
        port->MSR = LPI2C_MSR_EPF;  //Clear Stop Detect Flag
        if (_error == I2CError::ok && callback_ptr != nullptr) {
            state = State::callback;
            callback_ptr->I2CCallback(readBuffer);
        }
        state = State::idle;
    }
    
}

inline uint8_t IMX_RT1060_I2CMaster::tx_fifo_count() {
    return port->MFSR & 0x7;
}

inline uint8_t IMX_RT1060_I2CMaster::rx_fifo_count() {
    return (port->MFSR >> 16) & 0x07;
}

inline void IMX_RT1060_I2CMaster::clear_all_msr_flags() {
    port->MSR &= (LPI2C_MSR_DMF | LPI2C_MSR_PLTF | LPI2C_MSR_FEF |
                  LPI2C_MSR_ALF | LPI2C_MSR_NDF | LPI2C_MSR_SDF |
                  LPI2C_MSR_EPF | LPI2C_MSR_RDF | LPI2C_MSR_TDF);
}



// In theory, you can use MCR[RST] to reset the master but
// this doesn't seem to work in some circumstances. e.g.
// When the master is trying to receive more bytes.
bool IMX_RT1060_I2CMaster::abort_transaction_async() {
    #ifdef DEBUG_I2C
    Serial.println("Master: abort_transaction");
    log_master_status_register(port->MSR);
    #endif

    // Don't handle anymore interrupts
    port->MIER &= ~LPI2C_MIER_TDIE;
    port->MIER &= ~LPI2C_MIER_RDIE;
    port->MIER &= ~LPI2C_MIER_SDIE;


    // Send a stop if haven't already done so and still control the bus
    uint32_t msr = port->MSR;
    if ((msr & LPI2C_MSR_MBF) && !(msr & LPI2C_MSR_SDF)) {
        #ifdef DEBUG_I2C
        Serial.println("  sending STOP");
        #endif
        port->MTDR = LPI2C_MTDR_CMD_STOP;
    }
    
    elapsedMillis timeout;
    while ((timeout < 200) && tx_fifo_count()) {
        continue;
    }
    
    // Clear out any commands that haven't been sent
    port->MCR |= LPI2C_MCR_RTF;
    port->MCR |= LPI2C_MCR_RRF;
    
    if (timeout >= 200)
        return false;
    else
        return true;
    
}

// Supports 100 kHz, 400 kHz and 1 MHz modes.
void IMX_RT1060_I2CMaster::set_clock(uint32_t frequency) {
    if (frequency < 400000) {
        // Use Standard Mode - up to 100 kHz
        port->MCCR0 = LPI2C_MCCR0_CLKHI(55) | LPI2C_MCCR0_CLKLO(59) |
                      LPI2C_MCCR0_DATAVD(25) | LPI2C_MCCR0_SETHOLD(40);
        port->MCFGR1 = LPI2C_MCFGR1_PRESCALE(1);
        port->MCFGR2 = LPI2C_MCFGR2_FILTSDA(5) | LPI2C_MCFGR2_FILTSCL(5) |
                       LPI2C_MCFGR2_BUSIDLE(2 * (59 + 40 + 2)); // Min BUSIDLE = (CLKLO+SETHOLD+2) × 2
        port->MCFGR3 = LPI2C_MCFGR3_PINLOW(CLOCK_STRETCH_TIMEOUT * 12 / 256 + 1);
    } else if (frequency < 1000000) {
        // Use Fast Mode - up to 400 kHz
        port->MCCR0 = LPI2C_MCCR0_CLKHI(26) | LPI2C_MCCR0_CLKLO(28) |
                      LPI2C_MCCR0_DATAVD(12) | LPI2C_MCCR0_SETHOLD(18);
        port->MCFGR1 = LPI2C_MCFGR1_PRESCALE(0);
        port->MCFGR2 = LPI2C_MCFGR2_FILTSDA(2) | LPI2C_MCFGR2_FILTSCL(2) |
                       LPI2C_MCFGR2_BUSIDLE(2 * (28 + 18 + 2)); // Min BUSIDLE = (CLKLO+SETHOLD+2) × 2
        port->MCFGR3 = LPI2C_MCFGR3_PINLOW(CLOCK_STRETCH_TIMEOUT * 24 / 256 + 1);
    } else {
        // Use Fast Mode Plus - up to 1 MHz
        port->MCCR0 = LPI2C_MCCR0_CLKHI(9) | LPI2C_MCCR0_CLKLO(10) |
                      LPI2C_MCCR0_DATAVD(4) | LPI2C_MCCR0_SETHOLD(7);
        port->MCFGR1 = LPI2C_MCFGR1_PRESCALE(0);
        port->MCFGR2 = LPI2C_MCFGR2_FILTSDA(1) | LPI2C_MCFGR2_FILTSCL(1) |
                       LPI2C_MCFGR2_BUSIDLE(2 * (10 + 7 + 2)); // Min BUSIDLE = (CLKLO+SETHOLD+2) × 2
        port->MCFGR3 = LPI2C_MCFGR3_PINLOW(CLOCK_STRETCH_TIMEOUT * 24 / 256 + 1);
    }
    port->MCCR1 = port->MCCR0;
}

void IMX_RT1060_I2CSlave::listen(uint8_t address) {
    // Listen to a single 7-bit address
    uint32_t samr = LPI2C_SAMR_ADDR0(address);
    uint32_t address_config = LPI2C_SCFGR1_ADDRCFG(0x0);
    listen(samr, address_config);
}

void IMX_RT1060_I2CSlave::listen(uint8_t first_address, uint8_t second_address) {
    // Listen to 2 7-bit addresses
    uint32_t samr = LPI2C_SAMR_ADDR0(first_address) | LPI2C_SAMR_ADDR1(second_address);
    uint32_t address_config = LPI2C_SCFGR1_ADDRCFG(0x02);
    listen(samr, address_config);
}

void IMX_RT1060_I2CSlave::listen_range(uint8_t first_address, uint8_t last_address) {
    // Listen to all 7-bit addresses in the range (inclusive)
    uint32_t samr = LPI2C_SAMR_ADDR0(first_address) | LPI2C_SAMR_ADDR1(last_address);
    uint32_t address_config = LPI2C_SCFGR1_ADDRCFG(0x06);
    listen(samr, address_config);
}

void IMX_RT1060_I2CSlave::listen(uint32_t samr, uint32_t address_config) {
    // Make sure slave mode is disabled before configuring it.
    stop_listening();

    initialise_common(config, pad_control_config);

    // Set the Slave Address
    port->SAMR = samr;

    // Enable clock stretching
    port->SCFGR1 = (address_config | LPI2C_SCFGR1_TXDSTALL | LPI2C_SCFGR1_RXSTALL);
    // Set up interrupts
    attachInterruptVector(config.irq, isr);
    port->SIER = (LPI2C_SIER_RSIE | LPI2C_SIER_SDIE | LPI2C_SIER_TDIE | LPI2C_SIER_RDIE);
    NVIC_ENABLE_IRQ(config.irq);

    // Enable Slave Mode
    port->SCR = LPI2C_SCR_SEN;
}

inline void IMX_RT1060_I2CSlave::stop_listening() {
    // End slave mode
    stop(port, config.irq);
}

inline void IMX_RT1060_I2CSlave::after_receive(std::function<void(size_t len, uint16_t address)> callback) {
    after_receive_callback = callback;
}

inline void IMX_RT1060_I2CSlave::before_transmit(std::function<void(uint16_t address)> callback) {
    before_transmit_callback = callback;
}

inline void IMX_RT1060_I2CSlave::after_transmit(std::function<void(uint16_t address)> callback) {
    after_transmit_callback = callback;
}

inline void IMX_RT1060_I2CSlave::set_transmit_buffer(uint8_t* buffer, size_t size) {
    tx_buffer.initialise(buffer, size);
}

inline void IMX_RT1060_I2CSlave::set_receive_buffer(uint8_t* buffer, size_t size) {
    rx_buffer.initialise(buffer, size);
}

// WARNING: Do not call directly.
void IMX_RT1060_I2CSlave::_interrupt_service_routine() {
    // Read the slave status register
    uint32_t ssr = port->SSR;
//    log_slave_status_register(ssr);

    if (ssr & LPI2C_SSR_AVF) {
        // Find out which address was used and clear to the address flag.
        address_called = (port->SASR & LPI2C_SASR_RADDR(0x7FF)) >> 1;
    }

    if (ssr & (LPI2C_SSR_RSF | LPI2C_SSR_SDF)) {
        // Detected Repeated START or STOP
        port->SSR = (LPI2C_SSR_RSF | LPI2C_SSR_SDF);
        end_of_frame();
    }

    if (ssr & LPI2C_SSR_RDF) {
        //  Received Data
        uint32_t srdr = port->SRDR; // Read the Slave Received Data Register
        if (srdr & LPI2C_SRDR_SOF) {
            // Start of Frame (The first byte since a (repeated) START or STOP condition)
            _error = I2CError::ok;
            if (rx_buffer.initialised()) {
                rx_buffer.reset();
                state = State::receiving;
            }
        }
        uint8_t data = srdr & LPI2C_SRDR_DATA(0xFF);
        if (rx_buffer.initialised()) {
            if (!rx_buffer.write(data)) {
                // The buffer is already full.
                _error = I2CError::buffer_overflow;
                // TODO: The spec says we should send NACK but how?
            }
        } else {
            // We are not interested in reading anything.
            // TODO: The spec says we should send NACK but how?
            // Clear previous status and error
            state = State::idle;
        }
    }

    if (ssr & LPI2C_SSR_TDF) {
        // Transmit Data Request - Master is requesting a byte
        bool start_of_frame = state >= State::idle;
        if (start_of_frame) {
            _error = I2CError::ok;
            state = State::transmitting;
            if (before_transmit_callback) {
                before_transmit_callback(address_called);
            }
        }
        if (tx_buffer.initialised()) {
            if (start_of_frame) {
                tx_buffer.reset();
            }
            if (tx_buffer.has_data_available()) {
                port->STDR = tx_buffer.read();
            } else {
                port->STDR = DUMMY_BYTE;
                // WARNING: We're always asked for one more byte then
                // the master actually requested. Use trailing_byte_sent
                // to work out whether the master actually asked for too much data.
                if (!trailing_byte_sent) {
                    trailing_byte_sent = true;
                } else {
                    _error = I2CError::buffer_underflow;
                }
            }
        } else {
            // We don't have any data to send.
            // TODO: The spec says we should send NACK but how?
            // Just send a dummy value for now.
            port->STDR = DUMMY_BYTE;
            _error = I2CError::buffer_underflow;  // Clear previous status
        }
    }

    if (ssr & LPI2C_SSR_FEF) {
        port->SSR = LPI2C_SSR_FEF;
        // Will not happen if clock stretching is enabled.
    }

    if (ssr & LPI2C_SSR_BEF) {
        #ifdef DEBUG_I2C
        Serial.println("I2C Slave: Bit Error");
        #endif
        // The bus is probably stuck at this point.
        // I don't think the slave can clear the fault. The master has to do it.
        port->SSR = LPI2C_SSR_BEF;
        state = State::aborted;
        _error = I2CError::bit_error;
        end_of_frame();
    }
}

// Called from within the ISR when we receive a Repeated START or STOP
void IMX_RT1060_I2CSlave::end_of_frame() {
    if (state == State::receiving) {
        if (after_receive_callback) {
            after_receive_callback(rx_buffer.get_bytes_transferred(), address_called);
        }
    } else if (state == State::transmitting) {
        trailing_byte_sent = false;
        if (after_transmit_callback) {
            after_transmit_callback(address_called);
        }
    }
    #ifdef DEBUG_I2C
    else if (state != State::idle) {
        Serial.print("Unexpected 'End of Frame'. State: ");
        Serial.println((int)state);
    }
    if (_error == I2CError::bit_error) {
        Serial.println("Transaction aborted because of bit error.");
    }
    #endif
    state = State::idle;
}

IMX_RT1060_I2CBase::Config i2c1_config = {
        CCM_CCGR2,
        CCM_CCGR2_LPI2C1(CCM_CCGR_ON),
        IMX_RT1060_I2CBase::PinInfo{18U, 3U | 0x10U, &IOMUXC_LPI2C1_SDA_SELECT_INPUT, 1U},
        IMX_RT1060_I2CBase::PinInfo{19U, 3U | 0x10U, &IOMUXC_LPI2C1_SCL_SELECT_INPUT, 1U},
        false,
        {},
        {},
        IRQ_LPI2C1
};

IMX_RT1060_I2CBase::Config i2c3_config = {
        CCM_CCGR2,
        CCM_CCGR2_LPI2C3(CCM_CCGR_ON),
        IMX_RT1060_I2CBase::PinInfo{17U, 1U | 0x10U, &IOMUXC_LPI2C3_SDA_SELECT_INPUT, 2U},
        IMX_RT1060_I2CBase::PinInfo{16U, 1U | 0x10U, &IOMUXC_LPI2C3_SCL_SELECT_INPUT, 2U},
        true,
        IMX_RT1060_I2CBase::PinInfo{36U, 2U | 0x10U, &IOMUXC_LPI2C3_SDA_SELECT_INPUT, 1U},
        IMX_RT1060_I2CBase::PinInfo{37U, 2U | 0x10U, &IOMUXC_LPI2C3_SCL_SELECT_INPUT, 1U},
        IRQ_LPI2C3
};

IMX_RT1060_I2CBase::Config i2c4_config = {
        CCM_CCGR6,
        CCM_CCGR6_LPI2C4_SERIAL(CCM_CCGR_ON),
        IMX_RT1060_I2CBase::PinInfo{25U, 0U | 0x10U, &IOMUXC_LPI2C4_SDA_SELECT_INPUT, 1U},
        IMX_RT1060_I2CBase::PinInfo{24U, 0U | 0x10U, &IOMUXC_LPI2C4_SCL_SELECT_INPUT, 1U},
        false,
        {},
        {},
        IRQ_LPI2C4
};

static void master_isr();

IMX_RT1060_I2CMaster Master = IMX_RT1060_I2CMaster(&LPI2C1, i2c1_config, master_isr);

static void master_isr() {
    Master._interrupt_service_routine();
}

static void master1_isr();

IMX_RT1060_I2CMaster Master1 = IMX_RT1060_I2CMaster(&LPI2C3, i2c3_config, master1_isr);

static void master1_isr() {
    Master1._interrupt_service_routine();
}

static void master2_isr();

IMX_RT1060_I2CMaster Master2 = IMX_RT1060_I2CMaster(&LPI2C4, i2c4_config, master2_isr);

static void master2_isr() {
    Master2._interrupt_service_routine();
}

static void slave_isr();

IMX_RT1060_I2CSlave Slave = IMX_RT1060_I2CSlave(&LPI2C1, i2c1_config, slave_isr);

static void slave_isr() {
    Slave._interrupt_service_routine();
}

static void slave1_isr();

IMX_RT1060_I2CSlave Slave1 = IMX_RT1060_I2CSlave(&LPI2C3, i2c3_config, slave1_isr);

static void slave1_isr() {
    Slave1._interrupt_service_routine();
}

static void slave2_isr();

IMX_RT1060_I2CSlave Slave2 = IMX_RT1060_I2CSlave(&LPI2C4, i2c4_config, slave2_isr);

static void slave2_isr() {
    Slave2._interrupt_service_routine();
}

#ifdef DEBUG_I2C
static void log_master_control_register(const char* message, uint32_t mcr) {
    Serial.print(message);
    Serial.print(" MCR: ");
    Serial.print(mcr);
    Serial.println("");
}

static void log_master_status_register(uint32_t msr) {
    //	return;
    if (msr) {
        Serial.print("MSR Flags: ");
    }
    if (msr & LPI2C_MSR_BBF) {
        Serial.print("BBF ");
    }
    if (msr & LPI2C_MSR_MBF) {
        Serial.print("MBF ");
    }
    if (msr & LPI2C_MSR_DMF) {
        Serial.print("DMF ");
    }
    if (msr & LPI2C_MSR_PLTF) {
        Serial.print("PLTF ");
    }
    if (msr & LPI2C_MSR_FEF) {
        Serial.print("FEF ");
    }
    if (msr & LPI2C_MSR_ALF) {
        Serial.print("ALF ");
    }
    if (msr & LPI2C_MSR_NDF) {
        Serial.print("NDF ");
    }
    if (msr & LPI2C_MSR_SDF) {
        Serial.print("SDF ");
    }
    if (msr & LPI2C_MSR_EPF) {
        Serial.print("EPF ");
    }
    if (msr & LPI2C_MSR_RDF) {
        Serial.print("RDF ");
    }
    if (msr & LPI2C_MSR_TDF) {
        Serial.print("TDF ");
    }
    if (msr) {
        Serial.println();
    }
}

static void log_slave_status_register(uint32_t ssr) {
    if (ssr) {
        Serial.print("SSR Flags: ");
    }
    if (ssr & LPI2C_SSR_BBF) {
        Serial.print("BBF ");
    }
    if (ssr & LPI2C_SSR_SBF) {
        Serial.print("SBF ");
    }
    if (ssr & LPI2C_SSR_SARF) {
        Serial.print("SARF ");
    }
    if (ssr & LPI2C_SSR_GCF) {
        Serial.print("GCF ");
    }
    if (ssr & LPI2C_SSR_AM1F) {
        Serial.print("GCF ");
    }
    if (ssr & LPI2C_SSR_AM0F) {
        Serial.print("AM0F ");
    }
    if (ssr & LPI2C_SSR_FEF) {
        Serial.print("FEF ");
    }
    if (ssr & LPI2C_SSR_BEF) {
        Serial.print("BBF ");
    }
    if (ssr & LPI2C_SSR_SDF) {
        Serial.print("SDF ");
    }
    if (ssr & LPI2C_SSR_RSF) {
        Serial.print("RSF ");
    }
    if (ssr & LPI2C_SSR_TAF) {
        Serial.print("TAF ");
    }
    if (ssr & LPI2C_SSR_AVF) {
        Serial.print("AVF ");
    }
    if (ssr & LPI2C_SSR_RDF) {
        Serial.print("RDF ");
    }
    if (ssr & LPI2C_SSR_TDF) {
        Serial.print("TDF ");
    }
    if (ssr) {
        Serial.println();
    }
}
#pragma clang diagnostic pop
#endif  //DEBUG_I2C
