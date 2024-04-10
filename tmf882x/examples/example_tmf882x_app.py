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
- Configure the application
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
from tmf882x.tmf882x_app import Tmf882xResultFrame
import time
import os
import csv


HEX_FILE =  os.path.dirname(__file__) + "\\..\\hex_files\\tmf8x2x_application_patch.hex"
CSV_FILE = "test_frames.csv"
   
def measure( tof:Tmf882xApp, csv_writer, number_of_frames:int, log:bool=False):
    """
    Start a measurement and read number_of_frames, then stop the measurement    

    Args:
        tof (Tmf882xApp): the tof instance.
        csv_writer (TYPE): the csv writer.
        log (bool, optional): DESCRIPTION. Defaults to False.
        number_of_frames (int): the number of frames to measure. Defaults to 100
        log (bool, optional): Whether to print to console or not. Defaults to False.
    """
    tof.startMeasure()
    read_frames = 0
    while read_frames < number_of_frames:
        if ( tof.readAndClearInt( tof.TMF8X2X_APP_I2C_RESULT_IRQ_MASK ) ):
            read_frames = read_frames + 1
            frame = tof.readResult()
            data_list, _ = tof.getResultFields( frame )
            row = [ "#obj" ] + data_list            
            csv_writer.writerow( row )
            if log: print( "Frame[{}]:".format(read_frames), row )                # regular result frame header
    tof.stopMeasure()

def configure( tof:Tmf882xApp, csv_writer, is_8x8_mode:bool, period_in_ms:int, kilo_iterations:int, spad_map_id:int=Tmf882xApp.TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_1, log:bool=False):  
    """
    Set 4x4 or 8x8 mode, and configure device.
    Args:
        tof (Tmf882xApp): the tof instance.
        csv_writer (TYPE): the csv writer.
        is_8x8_mode (bool): use in 8x8 or legacy mode. 
        period_in_ms (int): measurement period in ms. 
        kilo_iterations (int): kilo iterations.
        spad_map_id (int, optional): which SPAD to use 1..15. Defaults to Tmf882xApp.TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_1.
        log (bool, optional): Whether to print to console or not. Defaults to False.
    """
    if is_8x8_mode:
        row = [ "#Mode", "8x8"]
        csv_writer.writerow( row )
        tof.set8x8Mode()
    else:
        row = [ "#Mode", "3x3/4x4/3x6"]
        csv_writer.writerow( row )
        tof.setLegacyMode()
    tof.configure(period_in_ms=period_in_ms,kilo_iterations=kilo_iterations,spad_map_id=spad_map_id)


def execute(tof:Tmf882xApp, csv_writer, hex_file:str, number_of_frames:int,log:bool=False):
    """
    Enable tof, download image, configure for measurement in 4x4 mode, measure n-frames, 
    configure for 8x8 mode, measure n-frames, disable tof
    Args:
        tof (Tmf882xApp): the tof instance.
        csv_writer (TYPE): the csv writer.
        hex_file (str): name of the hexfile.
        number_of_frames (int): the number of frames to measure. 
        log (bool, optional): Whether to print to console or not. Defaults to False.
    """
    tof.enable()                                          
    tof.downloadAndStartApp(hex_file)

    time.sleep(0.5) # give the application some time to fully start up.
    print("Application {} started".format(tof.getAppId()))

    # configure and start measure in 3x3 mode
    configure(tof, csv_writer, is_8x8_mode=False, period_in_ms=33, kilo_iterations=537, spad_map_id=Tmf882xApp.TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_7, log=log)
    measure(tof, csv_writer, number_of_frames=number_of_frames, log=log)

    # configure and start measure in 8x8 mode
    configure(tof, csv_writer, is_8x8_mode=True, period_in_ms=66, kilo_iterations=128, spad_map_id=Tmf882xApp.TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_15, log=log)
    measure(tof, csv_writer, number_of_frames=number_of_frames, log=log)

    tof.disable()

if __name__ == "__main__":
    # instanciate FTDI communication
    com = Ftdi(log=False,exception_on_error=True)
    tof = Tmf882xApp(ic_com=com)

    # open CSV writer    
    f = open( CSV_FILE, 'w', encoding='UTF8', newline='' )
    f.write( "sep=,\n")    
    f.write( "Tmf882x test\n");
    csvout = csv.writer( f, delimiter=',')  
    # print to csv some nice headers for all columns
    frame = Tmf882xResultFrame()
    row = [ "#Obj" ] + tof.getResultFieldsText( frame )
    csvout.writerow( row )

    # open communication at host side                 
    if ( Tmf882xApp.Status.OK != tof.open(i2c_speed=400000) ):  # open FTDI communication channels
        raise RuntimeError( "Error open FTDI device" )

    # download  image and perform 30 measurements
    execute( tof, csvout, HEX_FILE, number_of_frames=30,log=True )

    # close communication at host side
    tof.close()

    # csv file close
    f.close()               

    errors = tof.getAndResetErrors()
    if len(errors) > 0:
        for error in errors:
            print("ERROR: {:s}".format(error))
    else:
        print("The program finished without errors")
