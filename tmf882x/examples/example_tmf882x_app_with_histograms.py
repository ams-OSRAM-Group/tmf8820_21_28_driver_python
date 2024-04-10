

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
    
''' Example interaction with a TMF882X application  
- Open the communication
- Enable the device
- Download the RAM application 
- Start the application
- Read the firmware application version
- Configure the application for histogram readout

- Start a measurement
- Read <n> results and write to CSV
- Stop a measurement
- Disable+close the device
- Read the error codes
'''

# set to "True" if you want to use the EVM controller board, "False" for the FT2232 controller board
use_evm = False

import __init__
if use_evm:
    from aos_com.evm_ftdi import EvmFtdi as Ftdi
else:
    from aos_com.ft2232_ftdi import Ft2232Ftdi as Ftdi
from tmf882x.tmf882x_app import Tmf882xApp
import time
import os



# number of measurement iterations
number_of_frames = 10

REGULAR_HEX_FILE =  os.path.dirname(__file__) + "\\..\\hex_files\\tmf8x2x_application_patch.hex"

if __name__ == "__main__":
    # set up FTDI communication
    print("Open ftdi communication channels")
    com = Ftdi(log=False,exception_on_error=True)

    print("Connect to TMF882X")
    tof = Tmf882xApp(ic_com=com) 
    tof.open()
    tof.disable()
    time.sleep(0.01)
    tof.enable()                                          
    tof.downloadAndStartApp(REGULAR_HEX_FILE)
    time.sleep(0.01) # give the application some time to fully start up.
    if tof.isAppRunning() == False:
        print("The application did not start up as expected...")
    print("[app_id, minor, patch, build] are: {}.".format(tof.getAppId()))

    tof.set8x8Mode()     # change to 8x8 mode
    tof.setLegacyMode()  # change back to legacy mode

    if tof.Status.OK == tof.configure(period_in_ms=100, kilo_iterations=1000, spad_map_id=Tmf882xApp.TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_7):
        print("Tmf882x configured")
    else:
        print("Tmf882x configuration failed")

    read = 0
    tof.startMeasure()
    while read < number_of_frames:
        if tof.isResultInterrupt():
            read = read + 1
            frame = tof.readResultInt()
            frame_corr = tof.applyClkCorrection( frame, apply_to_2nd_object=False )
            text = tof.getResultFieldsText( frame_corr )
            data, data_dict = tof.getResultFields( frame )
            data_corr, data_dict_corr = tof.getResultFields( frame_corr )
            print( "Uncorrected Data:", data )
            print( "Corrected Data:  ", data_corr )
            text = tof.getResultIntensityFieldsText( frame_corr )
            data, data_dict = tof.getResultIntensityFields( frame_corr )

    tof.stopMeasure()

    tof.set8x8Mode()     # change to 8x8 mode
    if tof.Status.OK == tof.configure(period_in_ms=100, kilo_iterations=125, spad_map_id=Tmf882xApp.TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_15): # SPAD map cannot be choosen for 8x8
        print("Tmf882x configured")
    else:
        print("Tmf882x configuration failed")

    read = 0
    tof.startMeasure()
    while read < number_of_frames:
        if tof.isResultInterrupt():
            read = read + 1
            frame = tof.readResultInt()
            data = tof.getResultFields( frame )
    
    tof.stopMeasure()

    # use legacy mode for histogram readout
    tof.setLegacyMode()
    if tof.Status.OK == tof.configure(period_in_ms=100, kilo_iterations=1000, spad_map_id=Tmf882xApp.TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_7, histograms=Tmf882xApp.TMF8X2X_RAW_HISTOGRAMS|Tmf882xApp.TMF8X2X_CALIBRATION_HISTOGRAMS):
        print("Tmf882x configured for histogram dumping")
    else:
        print("Tmf882x configuration failed")

    read = 0
    tof.startMeasure()
    while read < number_of_frames:
        if tof.isResultInterrupt():
            read = read + 1
            tof.readResultInt()
            print( f"Measurement #{read:02} complete." )
            print( "-" )
        if tof.isDiagnosticInterrupt():
            tof.readHistogramInt()
            if tof.histogram_is_complete:
                print( "Electrical Calibration Histograms:" if tof.histogram_id == tof.TMF8X2X_CALIBRATION_HISTOGRAMS_ID else "Measurement Histograms:" )
                channel = 0
                for hist in tof.histograms:
                    print( f"Channel #{channel:02}:{hist}" )
                    channel += 1
    
    tof.stopMeasure()
    tof.disable()
    tof.close()

    errors = tof.getAndResetErrors()
    if len(errors) > 0:
        print("The program finished with {} errors".format(len(errors)))
        for error in errors:
            print(error)
    else:
        print("The program finished without errors")
