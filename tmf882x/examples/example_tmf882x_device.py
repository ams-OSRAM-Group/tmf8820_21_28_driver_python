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
This is a simple example program to support development of tmf8820_device.py. Not used for automated testing.
"""
# set to "True" if you want to use the EVM controller board, "False" for the FT2232 controller board
use_evm = False

import __init__
if use_evm:
    from aos_com.evm_ftdi import EvmFtdi as Ftdi
else:
    from aos_com.ft2232_ftdi import Ft2232Ftdi as Ftdi
import time
from tmf882x.tmf882x_device import Tmf882xDevice 

if __name__ == "__main__":
    # instanciate FTDI communication
    com = Ftdi(log=False,exception_on_error=True)
    bw = Tmf882xDevice(ic_com=com, log=True)
   
    print("Open FTDI communication channel. Automated I2C channel search.")
    bw.open()

    print("Connect to TMF882X")
    bw.disable()
    time.sleep(0.01) # wait until the device is turned off for sure
    # Toggle the enable pin to ensure the device is rebooted
    print("Enable the device.")
    bw.enable()

    data = bw.com.i2cTxRx(bw.I2C_SLAVE_ADDR, [0x0], 0x10)
    print("Reading 16 registers: {}".format(list(map(hex, data))))
    print("Disable the device.")
    bw.disable()
