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

HEX_FILE = os.path.dirname(__file__) + "\\..\\hex_files\\tmf8x2x_application_patch.hex"
# maximum time for each factory calibration run
FACTORY_CALIBRATION_TIMEOUT  = 8.0 # seconds 
# constants for parsing the calibration data list
CALIBRATION_STATUS_OFFSET = 0xB8
CALIBRATION_ITERATIONS_OFFSET = 0x06
CALIBRATION_ITERATIONS_SIZE = 2
CROSSTALK_OFFSET_CAPTURE_0 = 0x38
CROSSTALK_OFFSET_CAPTURE_1 = 0x90
CROSSTALK_LENGTH_PER_CAPTURE = 40
CROSSTALK_SIZE = 4
CROSSTALK_NUMBER_OF_CHANNELS_PER_CAPTURE = CROSSTALK_LENGTH_PER_CAPTURE // CROSSTALK_SIZE
# reference number of iterations for crosstalk measurement according to the optical design guide
CROSSTALK_REFERENCE_ITERATIONS_K = 550

def dumpCalibration(calData:list):
    calibrationIterations = int.from_bytes(bytes(calData[CALIBRATION_ITERATIONS_OFFSET:CALIBRATION_ITERATIONS_OFFSET+CALIBRATION_ITERATIONS_SIZE]),'little')
    # avoid divide by zero errors
    if calibrationIterations == 0:
        calibrationIterations = 1
    calibrationStatus = calData[CALIBRATION_STATUS_OFFSET]
    print("-")
    print(f"Calibration Iterations: {calibrationIterations}k")
    print(f"Calibration Status: 0x{calibrationStatus:02X}")
    print("Calibration successful" if calibrationStatus == 0x00 else f"Calibration failed with status 0x{calibrationStatus:02X}")
    print(f"Crosstalk Values:")

    channel = 0
    for captureOffset in (CROSSTALK_OFFSET_CAPTURE_0,CROSSTALK_OFFSET_CAPTURE_1):
        for offset in range(captureOffset,captureOffset+CROSSTALK_LENGTH_PER_CAPTURE,CROSSTALK_SIZE):
            # 32bit little endian representation
            crossTalk = int.from_bytes(bytes(calData[offset:offset+CROSSTALK_SIZE]),'little')
            print(f"Channel #{channel:02}: {crossTalk:12} | {crossTalk*CROSSTALK_REFERENCE_ITERATIONS_K//calibrationIterations:12} @ {CROSSTALK_REFERENCE_ITERATIONS_K}k {'(reference channel)' if channel % CROSSTALK_NUMBER_OF_CHANNELS_PER_CAPTURE == 0 else ''}")
            channel += 1

def doCalibration():
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

    tofApp.runFactoryCalibration(spad_map_id=map, kilo_iterations=4000, timeout=FACTORY_CALIBRATION_TIMEOUT)            
    calibrationData = tofApp.getFactoryCalibrationData()

    calibrationNr = 0
    for calData in calibrationData:
        dumpCalibration(calData)
        print(f"Calibration Data #{calibrationNr}: [ {', '.join(f'0x{x:02X}' for x in calData)} ]")
        calibrationNr += 1

if __name__ == "__main__":
    doCalibration()
