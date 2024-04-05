# *****************************************************************************
# * Copyright by ams OSRAM AG                                                 *
# * All rights are reserved.                                                  *
# *                                                                           *
# * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
# * THE SOFTWARE.                                                             *
# *                                                                           *
# * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
# * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
# * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
# * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
# * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
# * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
# * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES LOSS OF USE,      *
# * DATA, OR PROFITS OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY      *
# * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
# * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
# * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
# *****************************************************************************
"""
"""

import __init__
import enum
import time
from aos_com.ic_com import IcCom
from typing import List

class Tmf882xDevice:
    """The basic Chess communication class.
       It offers application/bootloader-independent functionality to interact with Chess via I2C/SPI/GPIO via a FTDI module.
    """
    
    VERSION = 1.4
    """Version log 
    - 1.0 First working version
    - 1.1 Code cleanup, moved functions into tmf882x.tmf882x_pp
    - 1.2 Improved error handling with exceptions
    - 1.3 n.a.
    - 1.4 Added parameter to "open" that allows manual selection of the FTDI I2C channel
    """

    class Status(enum.IntEnum):
        """The status return code of a method. either OK (=0), or an error (<>0)"""
        OK = 0 
        """The function executed as expected."""
        DEV_ERROR = -1 
        """The TMF882X device had a protocol error (e.g. the device FLASH reported an error over I2C)."""
        APP_ERROR = -2 
        """The TMF882X device application had a protocol error (e.g. the device firmware reported an error over I2C)."""
        TIMEOUT_ERROR = -3
        """The TMF882X device did not respond in time (e.g. found no SPI device)."""
        UART_ERROR = -4
        """Something went wrong when opening or reading from UART."""
        OTHER_ERROR = -5
        """Something went wrong, but there's no specific error code."""

    I2C_SLAVE_ADDR = 0x41 
    """The default I2C address. Fixed for now, can be changed later. """

    class ExceptionLevel(enum.IntEnum):
        """The exception level that defines until where an error shall throw an exception. """
        OFF = 0
        """Do not throw exceptions."""
        FTDI = 1
        """Throw exceptions on FTDI level (e.g., I2C-TX failed)"""
        DEVICE = 2
        """Throw exceptions on device level (e.g., could not enable device, SPI device not found, UART RX timed out)"""
        APP = 3
        """Throw exceptions on application level (e.g., command timed out, application reported error)"""
    
    TMF882X_ENABLE = 0xe0
    TMF882X_ENABLE__ready__MASK = 1<<6
    TMF882X_ENABLE__wakeup__MASK = 1<<0
    TMF882X_WAKEUP = TMF882X_ENABLE__ready__MASK | TMF882X_ENABLE__wakeup__MASK
    
    TMF882X_INT_STATUS = 0xe1
    TMF882X_INT_ENAB   = 0xe2

    def __init__(self, ic_com : IcCom, log = False ):
        """The default constructor. It initializes the FTDI driver.
        Args:
            ic_com (IcCom): The communication class instance to talk i2c etc.
            log (bool, optional): Enable verbose driver outputs. False per default.
        """
        self.com = ic_com 
        self.register_buffer = [0xff] * 256 # A buffer of all i2c registers. Used for fast load-modify-write operations, and as quick buffer access.
        self.power_up_mask = 0x41       

    def _setError(self, message):
        """An error occurred - add it to the error list, which the host can later read out.

        Args:
            message (str): The errorr message
        """
        self.com.errors.append(message)

    def _log(self, message):
        """Log information"""
        self.com._log(message)

    def getAndResetErrors(self):
        """Get a list of all error status flags and messages, and erase the internal error list.

        Returns:
            list({'status':int, 'message':str}): A list of the status values and the corresponding error messages.
            list(): if no error occurred
        """
        errors = self.com.errors
        self.com.errors = list()
        return errors

    def open( self, i2c_speed:int = 1000000 ):
        """
        Open the communication.
        Args:
            i2c_speed (int, optional): Open I2C communication. Defaults to 1000000.
        Returns:
            Status: The status code (OK = 0, error != 0)..
        """
        status = self.com.i2cOpen(i2c_speed=i2c_speed)
        if status == self.com.I2C_OK:
            self.com.gpioSetDirection(self.com.enable_pin, 0)
        return status
    
    def close( self ):
        """
        Closes the communication.
        Args:
            i2c_speed (int, optional): Open I2C communication. Defaults to 1000000.
        Returns:
            Status: The status code (OK = 0, error != 0)..
        """
        return self.com.i2cClose()

    def enable(self, send_wake_up_sequence:bool= True) -> Status:
        """Enable the TMF882X.
        Args:
            send_wake_up_sequence (bool, optional): Send the I2C power-on sequence from a cold start. Defaults to True.
        Returns:
            Status: The status code (OK = 0, error != 0)..
        """
        self.com.gpioSet(self.com.enable_pin, self.com.enable_pin) #set Enable pin to output, and INT pin to input.
        
        if send_wake_up_sequence:
            time.sleep(0.010) # TODO: Change to a proper bring-up. For now wait for 10 milliseconds until the device comes up.
            #Initial wakeup is simpler 
            return self.powerUp()
        return self.Status.OK


    def isDeviceWakeup(self) -> bool:
        enable =  self.com.i2cTxRx(self.I2C_SLAVE_ADDR, [self.TMF882X_ENABLE], 1)[0]
        if (enable & self.TMF882X_WAKEUP ) == self.TMF882X_WAKEUP:
            return True
        else:
            return False

    def powerDown(self) -> Status:
        if self.isDeviceWakeup():
            self.power_up_mask = self.com.i2cTxRx(self.I2C_SLAVE_ADDR, [self.TMF882X_ENABLE],1)[0]
            self.com.i2cTx(self.I2C_SLAVE_ADDR, [self.TMF882X_ENABLE, (self.power_up_mask & 0xFE)])     # request a power down
            time.sleep(0.010) # TODO: Change to a proper bring-up. For now wait for 10 milliseconds until the device comes up.
            power_down =  self.com.i2cTxRx(self.I2C_SLAVE_ADDR, [self.TMF882X_ENABLE], 1)
            if len(power_down) < 1 or ( power_down[0] != 0x22 or power_down[0] == 0x02 ):
                self._setError("The device didn't power down as expected (ENABLE register value is: {}). please check you hardware setup.".format(power_down))
                return self.Status.DEV_ERROR
        return self.Status.OK

    def powerUp(self) -> Status:
        self.power_up_mask = self.com.i2cTxRx(self.I2C_SLAVE_ADDR, [self.TMF882X_ENABLE],1)[0]
        self.power_up_mask = self.power_up_mask | self.TMF882X_ENABLE__wakeup__MASK
        self.com.i2cTx(self.I2C_SLAVE_ADDR, [self.TMF882X_ENABLE, self.power_up_mask])     # request a power up = wakeup
        time.sleep(0.010) # TODO: Change to a proper bring-up. For now wait for 10 milliseconds until the device comes up.
        enable =  self.com.i2cTxRx(self.I2C_SLAVE_ADDR, [self.TMF882X_ENABLE], 1)
        enable =  self.com.i2cTxRx(self.I2C_SLAVE_ADDR, [self.TMF882X_ENABLE], 1)       # need to read twice to have HW change the return the correct value 
        if len(enable) < 1 or enable[0] & self.TMF882X_ENABLE__ready__MASK != self.TMF882X_ENABLE__ready__MASK:
            self._setError("The device didn't power up as expected (ENABLE register value is: {}). please check you hardware setup.".format(enable))
            return self.Status.DEV_ERROR
            self.com.i2cTx(self.I2C_SLAVE_ADDR, [self.TMF882X_ENABLE, enable[0] | self.TMF882X_ENABLE__wakeup__MASK])
        return self.Status.OK

    def disable(self):
        """Disable the TMF882X"""
        self.com.gpioSet(self.com.enable_pin, 0)

    def isIntPinPulledLow(self):
        """Check if the interrupt is pending, ie. if the INT pin is pulled low.
        Returns:
            bool: True if an interrupt is pending, False if it isn't 
        """
        level = self.com.gpioGet(self.com.interrupt_pin)
        return level == 0 # Open drain INT pin -> 0 == pending

    def readIntStatus(self) -> int:
        """ read the interrupt status register of TMF8x0x """
        intreg = self.com.i2cTxRx(self.I2C_SLAVE_ADDR, [self.TMF882X_INT_STATUS], 1 )
        if ( len(intreg) ):
            return intreg[0]
        self._setError("Cannot read the INT_STATUS register")
        return 0

    def clearIntStatus(self,bitMaskToClear):
        """ clear the interrupt status register of TMF8x0x 
         Args:
            bitMaskToClear: all bits set in this 8-bit mask will be cleared in the interrupt register 
        """
        self.com.i2cTx(self.I2C_SLAVE_ADDR, [self.TMF882X_INT_STATUS,bitMaskToClear] )

    def readIntEnable(self) -> int:
        """ read the interrupt enable register of TMF8x0x """
        enabreg = self.com.i2cTxRx(self.I2C_SLAVE_ADDR, [self.TMF882X_INT_ENAB], 1 )
        if ( len(enabreg) ):
            return enabreg[0]
        self._setError("Cannot read the INT_STATUS register")
        return 0
    
    def enableInt(self,bitMaskToEnable):
        """ enable all the interrupts that have the bit set in the parameter, all other interrupts will be disabled 
         Args:
            bitMaskToEnable: all bits set in this 8-bit mask will be enabled, all others disabled 
        """
        self.com.i2cTx(self.I2C_SLAVE_ADDR, [self.TMF882X_INT_ENAB,bitMaskToEnable] )

    def clearAndEnableInt(self,bitMaskToEnable):
        """
        Clear and enable given interrupt bits
        Args:
            bitMaskToEnable : all bits set in this 8-bit mask will be cleared and enabled, all others disabled
        """
        self.clearIntStatus(bitMaskToEnable)    # first clear any old pending interrupt
        self.enableInt(bitMaskToEnable)         # now clear it
        
    def readAndClearInt(self,bitMaskToCheck):
        """
        Check if given interrupt bits are set, if they are, clear them and return them
        Args:
            bitMaskToCheck (TYPE): bit mask for interrupts to check for
        Returns:
            clr (TYPE): set interrupt bits that also have been cleared
        """
        clr = self.readIntStatus() & bitMaskToCheck
        if ( clr ):
            self.clearIntStatus( clr )
        return clr

if __name__ == "__main__":
    print("This program is not intended for standalone operation. Include it into application programs.")        
    