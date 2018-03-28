/*
 * Copyright (c) 2014, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 *
 * \brief Sensirion SHTC1 demo toolbox.
 *
 * The demo_tools contains everything that is not needed for sensor communication:
 * delay functions, port pins used for LED/button and the usart are initialized here.
 * There is a function provided to the main application which allows to
 * send a string (printf-style) over the serial connection to the terminal.
 * (you can read it using any terminal application, look for
 * The used COM port can be found within the device manager called
 * "EDBG Virtual COM Port".)
 *
 */

#ifndef DEMO_TOOLS_H_
#define DEMO_TOOLS_H_

/* Configuration of I2C communication with shtc1 extension module */
extern struct i2c_master_module i2c_master_instance;

/**
 * Initializes board for shtc1 extension board demo, i.e.,
 * configures pins, usart and delay
 */
void shtc1_demo_init(void);

/**
 * Configures usart. This is needed to get messages back to
 * host PC. To write these images, printf like
 * function print_to_terminal is provided.
 */
void configure_usart(void);

/**
 * Configures pins for led and button
 * used in demo example.
 */
void configure_port_pins(void);

/**
 * Configures I2C for communication with
 * Sensirion shtc1 externsion board plugged to
 * extension pos
 */
void configure_i2c_master(void);

/**
 * Sends a string (printf-style) over the serial connection to the terminal.
 * (you can read it using any terminal application, look for
 * The used COM port can be found within the device manager called
 * "EDBG Virtual COM Port".)
 *
 * @param C-printf-style format string.
 */
void print_to_terminal(const char *format, ...)
        __attribute__ ((format (gnu_printf, 1, 2)));

#endif /* DEMO_TOOLS_H_ */
