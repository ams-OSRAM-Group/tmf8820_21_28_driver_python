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

"""Example program that runs a complete measurement in 8x8 mode and employs the descattering filter."""

import __init__
import os
import time
from tmf882x.tmf882x_app import Tmf882xApp
from ctypes import CDLL

# set to "True" if you want to use the EVM controller board, "False" for the FT2232 controller board
use_evm : bool = True

# number of frames to receive during the test run for a complete 8x8 measurement
number_of_frames : int = 4

HEX_FILE = os.path.dirname(__file__) + "\\..\\hex_files\\tmf8x2x_application_patch.hex"

# number of rows in 8x8 mode, rows 8 .. 15 for second objects per pixel
ROWS = 16
# number of columns in 8x8 mode
COLS = 8

# number of rows for each result array (8x8)
RESULT_ARRAY_ROWS = 8

# 8x8 mode needs 4 subcaptures with 16 results each, two objects per pixel
NUMBER_OF_RESULTS_PER_RESULT_IDX = 32

# descatter filter threshold in percent
DESCATTER_THRESHOLD_PERCENT = 5

# maximum distance for the descattering filter
DESCATTER_MAX_DISTANCE_MM = 5000

# map capture result into an 8x16 array of distances, rows >=8 contain the second objects for each pixel -> offset 64
remap_table = [
    # capture 0, objects 0 
    56,    60,    40,    44,    24,    28,    8,    12,    57,    61,    41,    45,    25,    29,    9,    13, 
    # capture 0, objects 1
    56+64, 60+64, 40+64, 44+64, 24+64, 28+64, 8+64, 12+64, 57+64, 61+64, 41+64, 45+64, 25+64, 29+64, 9+64, 13+64, 

    # capture 1, objects 0 
    58,    62,    42,    46,    26,    30,    10,    14,    59,    63,    43,    47,    27,    31,    11,    15, 
    # capture 1, objects 1
    58+64, 62+64, 42+64, 46+64, 26+64, 30+64, 10+64, 14+64, 59+64, 63+64, 43+64, 47+64, 27+64, 31+64, 64+11, 15+64, 
    
    # capture 2, objects 0 
    48,    52,    32,    36,    16,    20,    0,    4,    49,    53,    33,    37,    17,    21,    1,    5,
    # capture 2, objects 1
    48+64, 52+64, 32+64, 36+64, 16+64, 20+64, 0+64, 4+64, 49+64, 53+64, 33+64, 37+64, 17+64, 21+64, 1+64, 5+64,

    # capture 3, objects 0 
    50,    54,    34,    38,    18,    22,    2,    6,    51,    55,    35,    39,    19,    23,    3,    7, 
    # capture 3, objects 1
    50+64, 54+64, 34+64, 38+64, 18+64, 22+64, 2+64, 6+64, 51+64, 55+64, 35+64, 39+64, 19+64, 23+64, 3+64, 7+64, 
    ]


# calculate logarithmic confidence encoding according to the data sheet
CONF_BREAKPOINT=40
EXP_GROWTH_RATE=1.053676
def calcLogarithmicConfidence( confidence: int ):
    exp_conf = 0
    if (confidence <= CONF_BREAKPOINT):
        exp_conf = confidence
    else:
    # exponential de-mapping
        steps = confidence - CONF_BREAKPOINT
        exp_conf = (CONF_BREAKPOINT*pow(EXP_GROWTH_RATE,steps))
    return int(exp_conf)    

def doDescatterFiltering():
    filterLib = None
    filterLibPath = __init__.TOF_PYTHON_ROOT_DIR + "/tmf882x/dll/tmf8xxx_descattering_filter.dll"
  
    try:
        filterLib = CDLL(filterLibPath)
    except Exception as e:
        print(f"{filterLibPath} loading error" ) 
        quit()

    # create FTDI communication instance
    com = None
    if use_evm:
        from aos_com.evm_ftdi import EvmFtdi
    else:
        from aos_com.ft2232_ftdi import Ft2232Ftdi
    if use_evm:
        com = EvmFtdi(log=False,exception_on_error=True)
    else:
        com = Ft2232Ftdi(log=False,exception_on_error=True)

    tofApp = Tmf882xApp(ic_com=com)

    # open FTDI communication channel
    if ( Tmf882xApp.Status.OK != tofApp.open(i2c_speed=400000) ):
        raise RuntimeError( "Error opening FTDI device" )
    tofApp.enable()                                          
    time.sleep(0.1)

    tofApp.downloadAndStartApp(HEX_FILE)
    # give the application some time to fully start up.
    time.sleep(1.0) 

    # switch TMF8828 firmware to 8x8 mode
    tofApp.set8x8Mode()
    # use long range accuracy for calibration
    tofApp.setLongRangeAccuracy()
    # 8x8 mode, user defined, time multiplexed 				
    currentMap = Tmf882xApp.TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_15

    applicationVersion = tofApp.getAppId()
    print(f"Application {applicationVersion} started")
    print(f"Application Mode {tofApp.getAppMode():02X}")
    _, _, _, build = applicationVersion
    if build & 0x10: # bit4 in build byte set -> short range mode supported
        print("Application supports short range mode")
    else:
        print("Application does not support short range mode")

    # configure sensor application firmware
    tofApp.configure(period_in_ms=66, kilo_iterations=128, spad_map_id=currentMap)
    # configure extended range for object confidence
    # 0x35 TMF8X2X_COM_ALG_SETTING_0 0   electrical_calibration     RW  0  # if set do report electrical calibration results
    # 0x35 TMF8X2X_COM_ALG_SETTING_0 1   statistics                 RW  0  # if set do report statistic information results
    # 0x35 TMF8X2X_COM_ALG_SETTING_0 2   distances                  RW  1  # if set do report distance results
    # 0x35 TMF8X2X_COM_ALG_SETTING_0 4:3 distance_mode              RW  0  # 0 ... 5m mode, 1 ... 2.5 m mode   
    # 0x35 TMF8X2X_COM_ALG_SETTING_0 6   diagnostics                RW  0  # if set do report also diagnostics in distance results
    # 0x35 TMF8X2X_COM_ALG_SETTING_0 7   increase_confidence_range  RW  0  # if set do enable increased range of the object zone confidences
    tofApp.configureResultReporting( 0x84 ) # distances | increase_confidence_range

    # configure descatter filter
    filterLib.descatterConfigure(DESCATTER_THRESHOLD_PERCENT,DESCATTER_MAX_DISTANCE_MM)
 
    # start continuous measurements
    tofApp.startMeasure()

    # capture 10 result sets with 8x8 pixels
    for capture in range(10):
    
        print(f"\n--- Result Set #{capture+1:02} ------------------------------------------")

        # wait end of current measurement sequence
        while True:
            if ( tofApp.readAndClearInt( tofApp.TMF8X2X_APP_I2C_RESULT_IRQ_MASK ) ):
                data_list, _ = tofApp.getResultFields(tofApp.readResult())
                result_number = data_list[3]
                if result_number % number_of_frames == number_of_frames - 1:
                    break           

        read_frames = 0

        # collect new set of measurement frames here
        # collect first, process later to get four consecutive frames for a single complete 8x8 measurement
        result_frames : tofApp.Tmf882xResultFrame = []

        while read_frames < number_of_frames:
            if ( tofApp.readAndClearInt( tofApp.TMF8X2X_APP_I2C_RESULT_IRQ_MASK ) ):
                frame = tofApp.readResult()            
                result_frames.append(frame)
                read_frames += 1

        # output data after filtering, 8x8 results with distances and confidences
        # up to two objects per pixel, second objects in rows 8 .. 15
        distances = [ 0 for i in range(ROWS*COLS) ] 
        confidences = [ 0 for i in range(ROWS*COLS) ] 

        # de-multiplex measurement data into an measurement data array
        for frame in result_frames:        
            data_list, _ = tofApp.getResultFields(frame)
            result_number = data_list[3]
            result_set_index = result_number % number_of_frames # 4 whole measurements of 16 pixels for 8x8 pixel result
            # skip results from unused channels 
            measurement_results = data_list[11:27] + data_list[29:45] + data_list[47:63] + data_list[65:81]
            print( f"Frame: Result Number: {result_number:03} Result Set Index: {result_set_index} {measurement_results}",  ) 

            for i in range(0,len(measurement_results),2):
                entry = remap_table[result_set_index*NUMBER_OF_RESULTS_PER_RESULT_IDX + i//2]
                confidences[entry] = calcLogarithmicConfidence(measurement_results[i])
                distances[entry] = measurement_results[i+1]

        # clear all learned filter data, do this before processing each result set!
        filterLib.descatterReset()

        # train descatter filter
        for i in range(len(distances)):
            filterLib.descatteraddObjectPeak(distances[i],confidences[i])

        print("\nDistances 1 --------------------------------------")

        # distances marked with the prefix "x" were removed by the descattering filter
        idx=0
        for row in range(ROWS):
            print(f"{row%RESULT_ARRAY_ROWS:02} ",end="")
            for dist in distances[row*COLS:row*COLS+RESULT_ARRAY_ROWS]:
                if filterLib.descatterIsScatteringPeak(distances[idx],confidences[idx]) == 1:
                    # mark scatter peak
                    print("x",end="")
                else:
                    print(" ",end="")
                print(f"{dist:04} ",end="")
                idx += 1
            print()
            if row == RESULT_ARRAY_ROWS-1:
                print("Distances 2 --------------------------------------")


        print("\nConfidences 1 ------------------------------------")

        for row in range(ROWS):
            print(f"{row%RESULT_ARRAY_ROWS:02} ",end="")
            for conf in confidences[row*COLS:row*COLS+RESULT_ARRAY_ROWS]:
                print(f"{conf:05} ",end="")
            print()
            if row == RESULT_ARRAY_ROWS-1:
                print("Confidences 2 ------------------------------------")
    
    # we're done, stop measurements
    tofApp.stopMeasure()

if __name__ == "__main__":
    doDescatterFiltering()
