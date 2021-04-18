// - - - - -
// DMXSerial - A Arduino library for sending and receiving DMX using the builtin serial hardware port.
// DMXSerial_magaavr.h: Hardware specific functions for MEGAAVR processors like 4809 used in Arduino Every.

// Copyright (c) 2011-2020 by Matthias Hertel, http://www.mathertel.de
// This work is licensed under a BSD style license. See http://www.mathertel.de/License.aspx
// - - - - -

// global variables and functions are prefixed with "_DMX_"

// ----- MegaAVR specific Hardware abstraction functions -----

#ifndef DMXSERIAL_MEGAAVR_H
#define DMXSERIAL_MEGAAVR_H

#if defined(DMXFORMAT)
#if defined(STM32F10X_MD) || defined(__STM32F1__) || defined(STM32F2XX)
#include "Arduino.h"
#include "DMXSerial.h"
#include <libmaple/usart.h>
//Arduino\hardware\Arduino_STM32\STM32F1\variants\generic_stm32f103c\board\board.h
#define _DMXSERIAL Serial2

int32_t _DMX_dmxDivider; // BAUD Devider factor for DMX speed.
int32_t _DMX_breakDivider; // BAUD Devider factor for BREAK speed.

/// Initialize the Hardware MUX and UART serial port.
void _DMX_init()
{
	_DMX_dmxDivider = DMXSPEED;
	//#define CYCLES_PER_MICROSECOND	(F_CPU / 1000000U)
	_DMX_breakDivider = 18000000/DMXSPEED; //us
	_DMXSERIAL.begin(_DMX_dmxDivider,SERIAL_8N2);
	usart_disable(USART2);
	// calculate BREAK speed Divider
	// disable interrupts during initialization
	// Setup port mux
	// Disable CLK2X, clock normal rate
	//Set up the rx & tx pins
	// enable interrupts again, restore SREG content
} // _DMX_init()


/// Initialize the Hardware UART serial port to the required mode.
void _DMX_setMode(DMXUARTMode mode)
{
	// disable interrupts during initialization
	// cli();
	nvic_globalirq_disable();
	usart_reg_map *regs = USART2_BASE;
	if (mode == DMXUARTMode::OFF) {
    // Disable transmitter and receiver
	// (USART_RXEN_bm | USART_TXEN_bm);
    // disable all interrupts
	// wait for TX completed
    /* TC bit must be high before disabling the USART */
    while((regs->CR1 & USART_CR1_UE) && !(regs->SR & USART_SR_TC));
    /* Disable UE */
    regs->CR1 &= ~((uint32)USART_CR1_UE);
	regs->CR1 &= ~((uint32)USART_CR1_RXNEIE);
	regs->CR1 &= ~((uint32)USART_CR1_TCIE);

  } else if (mode == DMXUARTMode::RONLY) {

    // assign the baud_divider, a.k.a. BAUD (USART Baud Rate Register)
    // accept data packets after first stop bit
    // Enable receiver only, normal speed
    // disable all interrupts
	regs->CR1 &= ~((uint32)USART_CR1_RXNEIE);
	regs->CR1 |= USART_CR1_RE;

  } else if (mode == DMXUARTMode::RDATA) {

	// assign the baud_divider, a.k.a. BAUD (USART Baud Rate Register)
    // accept data packets after first stop bit
	// Enable receiver only, normal speed
	// enable receive complete interrupt
	regs->CR1 |= ( USART_CR1_RE | USART_CR1_RXNEIE);
	regs->CR1 |= USART_CR1_UE;

  } else if (mode == DMXUARTMode::TBREAK) {
    // start UART with break settings, don't enable interrupts yet
	// no operation
	// disable all interrupts

	// assign the baud_divider, a.k.a. BAUD (USART Baud Rate Register)
    // Set USART mode of operation
    // Enable transmitter only, normal speed
    // is required again after disabling UART
    //  clear transmit complete flag
	// enable transmit complete interrupt
	// Wait for 1.5 frames before clearing the break condition
    // This will have different effects on our platforms, but should
    // ensure that we keep the break active for at least one frame.
    // We consider a full frame (1 start bit + 8 data bits bits +
    // 1 parity bit + 2 stop bits = 12 bits) for computation.
    // One bit time (in us) = 1000000/_baud
    // Twelve bits: 12000000/baud delay
    // 1.5 frames: 18000000/baud delay
	regs->CR1 |= USART_CR1_SBK;
	delayMicroseconds(_DMX_breakDivider);

  } else if (mode == DMXUARTMode::TDATA) {
    // switch to dmx data mode
	// disable all interrupts
    // assign the baud_divider, a.k.a. BAUD (USART Baud Rate Register)
    // send with 2 stop bits for compatibility
    // enable data register empty interrupt
	regs->CR1 |= (USART_CR1_TE | USART_CR1_TCIE);
	regs->CR1 |= USART_CR1_UE;
  } else if (mode == DMXUARTMode::TDONE) {
	regs->CR1 &= ~((uint32)USART_CR1_TCIE); // disable TCIE
	// assign the baud_divider, a.k.a. BAUD (USART Baud Rate Register)
	// send with 2 stop bits for compatibility
    //  clear transmit complete flag
    // enable transmit complete interrupt
  } // if

  // enable interrupts again, restore SREG content
  nvic_globalirq_enable();
} // _DMX_setMode()


// flush all incomming data packets in the queue
void _DMX_flush()
{
	usart_reg_map *regs = USART2_BASE;
    uint8_t voiddata =  regs->DR;
}


// send the next byte after current byte was sent completely.
inline void _DMX_writeByte(uint8_t data)
{
	// putting data into TXDATAL sends the data
	usart_reg_map *regs = USART2_BASE;
	regs->DR = data;
} // _DMX_writeByte


// This Interrupt Service Routine is called when a byte or frame error was received.
// In DMXController mode this interrupt is disabled and will not occur.
// In DMXReceiver mode when a byte or frame error was received

// Interrupt service routines that are called when the actual byte was sent.
// this interrupt occurs after data register was emptied by handing it over to the shift register.
extern "C" void __irq_usart2(void) {
  // put your interrupt handler code here:
	uint8_t rxferr = 0;
	usart_reg_map *regs = USART2_BASE;
    if ((regs->CR1 & USART_CR1_RXNEIE) && (regs->SR & USART_SR_RXNE)) {
			
		if( regs->SR & USART_SR_FE || regs->SR & USART_SR_PE ) {
           // framing error or parity error
           //read and throw away the data, this clears FE and PE as well
		   //check for break
		   rxferr = 1;
		} 
		_DMXReceived((uint8_t)regs->DR, rxferr);
    }
    /* TXE signifies readiness to send a byte to DR. */
    if ((regs->CR1 & USART_CR1_TXEIE) && (regs->SR & USART_SR_TXE)) {
        _DMXTransmitted(); 
    }
}

#endif
#endif

#endif
