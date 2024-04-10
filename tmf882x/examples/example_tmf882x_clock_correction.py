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

"""Example program that runs the factory calibration for a defined SPAD map and operation mode. Dumps calibration status and crosstalk values."""

# set to "True" if you want to use the EVM controller board, "False" for the FT2232 controller board
use_evm = False

import __init__
if use_evm:
    from aos_com.evm_ftdi import EvmFtdi as Ftdi
else:
    from aos_com.ft2232_ftdi import Ft2232Ftdi as Ftdi
import os
import time
from tmf882x.tmf882x_app import Tmf882xApp

# to switch on logging set this to "True"
log : bool = True

# number of frames to receive during the test run
number_of_frames : int = 30

HEX_FILE = os.path.dirname(__file__) + "\\..\\hex_files\\tmf8x2x_application_patch.hex"

def doClockCorrection():
    com = None
    tofApp = None
    
    # create FTDI communication instance
    com = Ftdi(log=False,exception_on_error=True)
    tofApp = Tmf882xApp(ic_com=com)

    # open FTDI communication channel
    if ( Tmf882xApp.Status.OK != tofApp.open(i2c_speed=400000) ):
        raise RuntimeError( "Error opening FTDI device" )
    tofApp.enable()                                          
    time.sleep(0.1)

    tofApp.downloadAndStartApp(HEX_FILE)
    # give the application some time to fully start up.
    time.sleep(1.0) 

    # switch TMF8828 firmware to TMF8821 mode
    tofApp.setLegacyMode()
    # use long range accuracy for calibration
    tofApp.setLongRangeAccuracy()
    # 4x4 map, size 18x10, time-multiplexed, Normal Mode (44°x48°) 			
    map = Tmf882xApp.TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_7

    applicationVersion = tofApp.getAppId()
    print(f"Application {applicationVersion} started")
    print(f"Application Mode {tofApp.getAppMode():02X}")
    _, _, _, build = applicationVersion
    if build & 0x10: # bit4 in build byte set -> short range mode supported
        print("Application supports short range mode")
    else:
        print("Application does not support short range mode")

    tofApp.startMeasure()
    read_frames = 0
    while read_frames < number_of_frames:
        if ( tofApp.readAndClearInt( tofApp.TMF8X2X_APP_I2C_RESULT_IRQ_MASK ) ):
            read_frames = read_frames + 1
            frame = tofApp.readResult()            
            tofApp.applyClkCorrection(frame,log=True)
            data_list, _ = tofApp.getResultFields(frame)
            if log: 
                print( f"Frame[{read_frames:02}]:", data_list ) # regular result frame header
    tofApp.stopMeasure()

if __name__ == "__main__":
    doClockCorrection()
