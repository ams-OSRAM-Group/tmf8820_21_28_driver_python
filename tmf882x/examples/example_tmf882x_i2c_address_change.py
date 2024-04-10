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
    
''' Example I2C address change.
- Open the communication
- Enable the device
- Download the RAM application 
- Start the application
- Change the I2C address to 0x6a
'''

# set to "True" if you want to use the EVM controller board, "False" for the FT2232 controller board
use_evm = False

import __init__
if use_evm:
    from aos_com.evm_ftdi import EvmFtdi as Ftdi
else:
    from aos_com.ft2232_ftdi import Ft2232Ftdi as Ftdi
import time
from tmf882x.tmf882x_app import Tmf882xApp
import time
import os

PATCH_FILE = os.path.dirname(__file__) + "/../hex_files/tmf8x2x_application_patch.hex"

if __name__ == "__main__":
    # instanciate FTDI communication
    com = Ftdi(log=False,exception_on_error=True)
    bw = Tmf882xApp(ic_com=com)
    if bw.open() != bw.Status.OK:
        print("Could not open the device")
    if bw.enable() != bw.Status.OK:
        print("Could not enable the device")

    if bw.downloadHexFile(PATCH_FILE) != bw.Status.OK:
        print("Could not download the application")
    if bw.startRamApp() != bw.Status.OK:
        print("Could not start the application")

    time.sleep(0.5) # give the application some time to fully start up.
    print("Application {} started".format(bw.getAppId()))

    if bw.loadConfig(bw.TMF8X2X_COM_CMD_STAT__cmd_load_config_page_common) != bw.Status.OK:
        print("could not load the configuration") 

    new_i2c_address = 0x6a
    TMF8X2X_COM_I2C_SLAVE_ADDRESS = 0x3B
    TMF8X2X_COM_I2C_ADDR_CHANGE = 0x3E
    TMF8X2X_COM_CMD_STAT__cmd_i2c_slave_adress = 0x21

    bw.register_buffer[TMF8X2X_COM_I2C_SLAVE_ADDRESS] = new_i2c_address << 1 # Set the I2C address, LSB is used for r/w bit.
    bw.register_buffer[TMF8X2X_COM_I2C_ADDR_CHANGE] = 0 # don't use GPIOs

    if bw.writeConfig() != bw.Status.OK:
        print("could not write the configuration") 

    # The I2C command to change the I2C address returns the status with the new I2C address. Wait here to ensure it changed.
    bw.com.i2cTx(bw.I2C_SLAVE_ADDR, [bw.TMF8X2X_COM_CMD_STAT, TMF8X2X_COM_CMD_STAT__cmd_i2c_slave_adress])
    time.sleep(0.1)
    print("Read registers 0-4 from the old I2C address: ", list(map(hex, bw.com.i2cTxRx(bw.I2C_SLAVE_ADDR, [0], 4))))
    print("Read registers 0-4 from the new I2C address: ", list(map(hex, bw.com.i2cTxRx(  new_i2c_address, [0], 4))))
    bw.I2C_SLAVE_ADDR = new_i2c_address
    print("Read APP-ID after address change: ", bw.getAppId())

    bw.disable()
    bw.close()
    errors = bw.getAndResetErrors()
    if len(errors) > 0:
        for error in errors:
            print("ERROR: {:s}".format(error))
    else:
        print("The program finished without errors")
