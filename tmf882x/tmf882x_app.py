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

"""The TMF882X application class and support classes to interface the TMF882X application as a host driver would."""

import __init__
import time
from typing import List
import os
from aos_com.ic_com import IcCom
from tmf882x.tmf882x_device import Tmf882xDevice
from intelhex import IntelHex
import ctypes

# set to "True" if you want to use the EVM controller board, "False" for the FT2232 controller board    
use_evm = False

REGULAR_HEX_FILE =  os.path.dirname(__file__) + "\\hex_files\\tmf8x2x_application_patch.hex"

# constants for all classes
TMF882X_APP_MAX_ZONES = 18                                  # time-multiplexe maximum of 18 zones produce a result
TMF882X_MAX_NUMBER_OF_RESULTS = TMF882X_APP_MAX_ZONES * 2   # 2 objects per zone
TMF882X_BINS = 128                                          # how many bytes are in a histogram sub-packet

class Tmf882xHeader(ctypes.LittleEndianStructure):
    """The tmf882x generic header """
    def __init__(self):
        self.cid_rid: ctypes.c_uint8
        self.tid: ctypes.c_uint8
        self.payload: ctypes.c_uint16
        super().__init__()

    _pack_ = 1
    _fields_ = [ 
        ("cid_rid", ctypes.c_uint8),
        ("tid", ctypes.c_uint8),
        ("payload", ctypes.c_uint16),
    ]

class Tmf882xResultFrameHeader(ctypes.LittleEndianStructure):
    """The tmf882x result frame header """
    def __init__(self):
        self.resultNumber: ctypes.c_uint8
        self.temperature: ctypes.c_uint8
        self.numberValidResults: ctypes.c_uint8
        self.reserved0: ctypes.c_uint8
        self.ambientLight: ctypes.c_uint32
        self.photonCount: ctypes.c_uint32 
        self.referencePhotonCount: ctypes.c_uint32
        self.systemTicks: ctypes.c_uint32
        super().__init__()

    _pack_ = 1
    _fields_ = [ 
        ("resultNumber", ctypes.c_uint8),
        ("temperature", ctypes.c_uint8),
        ("numberValidResults", ctypes.c_uint8),
        ("reserved0", ctypes.c_uint8),
        ("ambientLight", ctypes.c_uint32),
        ("photonCount", ctypes.c_uint32),
        ("referencePhotonCount", ctypes.c_uint32),
        ("systemTicks", ctypes.c_uint32),
    ]
    
class Tmf882xResultFramePayload(ctypes.LittleEndianStructure):
    """The tmf882x result frame payload """
    def __init__(self):
        self.confidence: ctypes.c_uint8
        self.distanceInMm: ctypes.c_uint16
        super().__init__()

    _pack_ = 1
    _fields_ = [ 
        ("confidence", ctypes.c_uint8),
        ("distanceInMm", ctypes.c_uint16),
    ]
    
class Tmf882xResultFrame(ctypes.LittleEndianStructure):
    def __init__(self):
        self.header: Tmf882xHeader
        """ The generic header for all communication """
        self.resultHeader: Tmf882xResultFrameHeader
        """The result header."""
        self.object1:List[Tmf882xResultFramePayload]
        """The list of first objects per zone."""
        self.object2:List[Tmf882xResultFramePayload]
        """The list of 2nd objects per zone."""
        super().__init__()
        
    _pack_ = 1
    _fields_ = [ 
        ("header", Tmf882xHeader),
        ("resultHeader", Tmf882xResultFrameHeader),
        ("object1", Tmf882xResultFramePayload*TMF882X_APP_MAX_ZONES),
        ("object2", Tmf882xResultFramePayload*TMF882X_APP_MAX_ZONES),
    ]
    
    def decode(self, raw_data:bytearray):
        """ Decode a raw data frame into this frame structure instance. """
        assert ctypes.sizeof(self) <= len(raw_data), "The data is too small"
        ctypes.memmove(ctypes.pointer(self), bytes(raw_data), ctypes.sizeof(self))

    def sizeBytes(self) -> int:
        """Get the size of the frame in bytes. """
        return ctypes.sizeof(self)

class Tmf882xSubHeader(ctypes.LittleEndianStructure):
    """The tmf882x generic header """
    def __init__(self):
        self.number: ctypes.c_uint8
        self.payload: ctypes.c_uint8
        self.config_id: ctypes.c_uint8
        super().__init__()

    _pack_ = 1
    _fields_ = [ 
        ("number", ctypes.c_uint8),
        ("payload", ctypes.c_uint8),
        ("config_id", ctypes.c_uint8),
    ]

class Tmf882xRawHistogramFrame(ctypes.LittleEndianStructure):
    def __init__(self):
        self.header: Tmf882xHeader
        """ The generic header for all communication """
        self.subHeader: Tmf882xSubHeader
        """The sub-packet header (for packets that span more than one packet)."""
        self.data:List[ctypes.c_uint8]
        """The list of histogram bytes."""
        super().__init__()
        
    _pack_ = 1
    _fields_ = [ 
        ("header", Tmf882xHeader),
        ("subHeader", Tmf882xSubHeader),
        ("data", ctypes.c_uint8*TMF882X_BINS),
    ]

    def decode(self, raw_data:bytearray):
        """ Decode a raw data frame into this frame structure instance. """
        assert ctypes.sizeof(self) <= len(raw_data), "The data is too small"
        ctypes.memmove(ctypes.pointer(self), bytes(raw_data), ctypes.sizeof(self))

    def sizeBytes(self) -> int:
        """Get the size of the frame in bytes. """
        return ctypes.sizeof(self)         
    

class Tmf882xApp(Tmf882xDevice):
    """The TMF882X application class to interface the TMF882X application as a host driver would.

    Args:
        Tmf882xDevice: the base class to interact with the device.
    """

    # Version log 
    # 1.0 first working version
    # 1.1 Added patch download and RAM App start.
    # 1.2 Added histogram dumping and clock correction
    # 1.3 Added factory calibration 
    VERSION = 1.3

    TMF8X2X_COM_CMD_STAT = 0x08

    TMF8X2X_COM_CMD_STAT__bl_cmd_ramremap = 0x11 # Bootloader command to remap the vector table into RAM (Start RAM application).
    TMF8X2X_COM_CMD_STAT__bl_cmd_romremap = 0x12 # Bootloader command to remap the vector table into ROM (Start ROM application).
    TMF8X2X_COM_CMD_STAT__bl_cmd_debug = 0x18 # Bootloader command to unlock the SWD pins.
    TMF8X2X_COM_CMD_STAT__bl_cmd_r_ram = 0x40 # Read from BL RAM.
    TMF8X2X_COM_CMD_STAT__bl_cmd_w_ram = 0x41 # Write to BL RAM.
    TMF8X2X_COM_CMD_STAT__bl_cmd_addr_ram = 0x43 # Write to BL RAM.

    # interrupt bits
    TMF8X2X_APP_I2C_ANY_IRQ_MASK                      =  0x01        # any of below interrupts has occured 
    TMF8X2X_APP_I2C_RESULT_IRQ_MASK                   =  0x02        # a measurement result is ready for readout 
    TMF8X2X_APP_I2C_ALT_RESULT_IRQ_MASK               =  0x04        # used for statistics and electrical calibration results 
    TMF8X2X_APP_I2C_RAW_HISTOGRAM_IRQ_MASK            =  0x08        # a raw histogram is ready for readout 
    TMF8X2X_APP_I2C_BREAKPOINT_IRQ_MASK               =  0x10        # a breakpoint has been hit 
    TMF8X2X_APP_I2C_CMD_DONE_IRQ_MASK                 =  0x20        # a received I2C command has been handled (successfully or failed) 
    TMF8X2X_APP_I2C_ERROR_IRQ_MASK                    =  0x40        # one of the <status> registers has been set to a non-zero value 


    TMF8X2X_COM_CMD_STAT__stat_ok = 0x0 # Everything is okay
    TMF8X2X_COM_CMD_STAT__stat_accepted = 0x1 # Everything is okay too, send sop to halt ongoing command
    TMF8X2X_COM_CMD_STAT__cmd_measure = 0x10                        # Start a measurement
    TMF8X2X_COM_CMD_STAT__cmd_write_config_page = 0x15              # Write the active config page
    TMF8X2X_COM_CMD_STAT__cmd_load_config_page_common = 0x16        # Load the common config page
    TMF8X2X_COM_CMD_STAT__cmd_load_config_page_spad_1 = 0x17        # load SPAD page 1
    TMF8X2X_COM_CMD_STAT__cmd_load_config_page_spad_2 = 0x18        # load SPAD page 2 (time-multiplexed)
    TMF8X2X_COM_CMD_STAT__cmd_load_config_page_factory_calib = 0x19 # load a factory calibration page
    TMF8X2X_COM_CMD_STAT__cmd_load_config_page_diagnostics = 0x1A   # load diagnostic page
    TMF8X2X_COM_CMD_STAT__cmd_factory_calibration_reset = 0x1F      # reset factory calibration state
    TMF8X2X_COM_CMD_STAT__cmd_factory_calibration = 0x20            # do a factory calibration
    TMF8X2X_COM_CMD_STAT__cmd_stop = 0xff # Stop a measurement
    TMF8828_COM_CMD_STAT__cmd_stat__CMD_SWITCH_TMF8821_MODE =  0x65 # Switch to 3x3/3x6/4x4 mode. The device will need to be re-configured after this command. Only supported if 8x8_measurements = 1.  
    TMF8828_COM_CMD_STAT__cmd_stat__CMD_SWITCH_TMF8828_MODE =  0x6C # Switch to 8x8 mode. The device will need to be re-configured after this command. Only supported if 8x8_measurements = 1. 

    TMF8X2X_BL_MAX_DATA_SIZE = 0x80 # Number of bytes that can be written or read with one BL command

    TMF8X2X_COM_APP_ID = 0x0
    TMF8X2X_COM_APP_ID__application = 0x3
    TMF8X2X_COM_APP_ID__bootloader = 0x80
    
    # the mode registers and its values
    TMF8828_COM_TMF8828_MODE                           = 0x10 # mode register is either 0x00 == tmf8820/1 or 0x08 == tmf8828                 
    TMF8828_COM_TMF8828_MODE__mode__TMF8821            = 0    # the device is operating in 3x3/3x6/4x4 (TMF8820/TMF8821) mode       
    TMF8828_COM_TMF8828_MODE__mode__TMF8828            = 8

    TMF8X2X_HISTOGRAM_ID                = 0x80 # identifying that this is a histogram frame
    TMF8X2X_RAW_HISTOGRAM_ID            = 0x81
    TMF8X2X_CALIBRATION_HISTOGRAMS_ID   = 0x82 # identifying electrical calibration histograms

    TMF8X2X_NO_HISTOGRAMS               = 0         # no histogram is produced by device
    TMF8X2X_RAW_HISTOGRAMS              = 1         # bit-mask for raw histogram dumping
    TMF8X2X_CALIBRATION_HISTOGRAMS      = 2         # bit-mask for electrical calibration histogram dumping

    TMF8X2X_COM_CONFIG_RESULT = 0x20
    TMF8X2X_COM_CONFIG_RESULT_LAST = 0xdf
    TMF8X2X_COM_FC_START = 0x24
    TMF8X2X_COM_CONFIG_FACTORY_CALIB__factory_calibration_size   = ((Tmf882xDevice.TMF882X_ENABLE)-(TMF8X2X_COM_FC_START)) 

    # common config page - some registers
    TMF8X2X_COM_PERIOD_MS_LSB = 0x24
    TMF8X2X_COM_PERIOD_MS_MSB = 0x25
    TMF8X2X_COM_KILO_ITERATIONS_LSB = 0x26    
    TMF8X2X_COM_KILO_ITERATIONS_MSB = 0x27
    TMF8X2X_COM_CONFIDENCE_THRESHOLD = 0x30
    TMF8X2X_COM_SPAD_MAP_ID = 0x34    
    TMF8X2X_COM_ALG_SETTING_0 = 0x35
    TMF8X2X_COM_HIST_DUMP = 0x39            # 0 ... all off, 1 ... raw histograms, 2 ... ec histograms]

    TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_1 = 1 # 3x3 map, size 14x6 		1. Normal Mode (29°x29°)  			
    TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_2 = 2 # 3x3 map, size 14x9 		2. Macro Mode (29°x43,5°)  			
    TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_3 = 3 # 3x3 map, size 14x9		3. Macro Mode (29°x43,5°) 			
    TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_4 = 4 # 4x4 map, size 14x9		4. Time-multiplexed, Normal/Macro Mode (29°x43,5°)	
    TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_5 = 5 # 4x4 map, size 14x9		5. Time-multiplexed, Normal/Macro Mode (29°x43,5°)	
    TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_6 = 6 # 3x3 map, size 18x10		6. Normal Mode (44°x48°) 			
    TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_7 = 7 # 4x4 map, size 18x10		7. Time-multiplexed, Normal Mode (44°x48°) 			
    TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_8 = 8 # 9 zones map, size 14x9	8. Normal/Macro Mode (29°x43,5°) 	
    TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_9 = 9 # 9 zones map, size 14x9	9. Normal/Macro Mode (29°x43,5°) 	
    TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_10 = 10 # 3x6 map, size 18x12       10. Time-multiplexed, (29°x57°)  	
    TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_11 = 11 # 3x3 map, size 14x6        11. Checkerboard, Normal Mode(29°x29°) 				
    TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_12 = 12 # 3x3 map, size 14x6        12. Reverse-Checkerboard, Normal Mode(29°x29°) 				
    TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_13 = 13 # 4x4 map, size 18x8        13. Time-multiplexed, Narrow Mode (29°x39°) 				
    TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_14 = 14 # user defined, not time multiplexed 				
    TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_15 = 15 # 8x8 mode, user defined, time multiplexed 				

    TMF8X2X_COM_RESULT_FRAME_SIZE = 128+4                   # Result + header
    TMF8X2X_COM_HISTOGRAM_FRAME_SIZE = 128+4+3              # data + header + sub-header
    TMF8X2X_MAX_CLK_CORRECTION_PAIRS = 4                    # how far appart the clock correction values are (== period * this_value) 
    TMF8X2X_CLK_CORRECTION_FACTOR   = 5                     # tmf882x clock ticks are in [0.2] microseconds, host ticks are in [1] microseconds
    TMF882X_CHANNELS = 10                                   # the tmf882x has native (not time-muliplexed 10 channels)
    TMF882X_CFG_IDX_FIELD = 2                               # index of the configuraiton index field in the sub-header

    # factory calibration starts at address 0x24 == TMF8X2X_COM_FC_START
    TMF882X_FC_KITERS = 0x06                # little endian - 16 bits
    TMF882X_FC_KITERS_SIZE = 2              # 16 bits -> 2 bytes
    TMF882X_FC_XTALK_AMPLITUDE_Z0_0 = 0x38  # configuration 0 (each zone is 4 bytes)
    TMF882X_FC_XTALK_AMPLITUDE_Z0_1 = 0x90  # configuration 1 (each zone is 4 bytes)
    TMF882X_FC_STATUS = 0xB8                # 0 == ok 
    TMF882X_FC_MIN_XTALK_AMPLITUDE = 6      # any value below is an error
    
    TMF882X_CROSSTALK_LENGTH_PER_CAPTURE = 40 # number of bytes for each calibration crosstalk block
    TMF882X_CROSSTALK_SIZE = 4                # number of bytes for each crosstalk entry
    TMF882X_CROSSTALK_NUMBER_OF_CHANNELS_PER_CAPTURE = TMF882X_CROSSTALK_LENGTH_PER_CAPTURE // TMF882X_CROSSTALK_SIZE
    # reference number of iterations for crosstalk measurement according to the optical design guide
    TMF882X_CROSSTALK_REFERENCE_ITERATIONS_K = 550

    # number of calibration data sets for each operation mode
    TMF8820_CALIBRATION_ITERATIONS = 1
    TMF8828_CALIBRATION_ITERATIONS = 4

    # constants for short range / long range selection
    TMF8X2X_REG_ACTIVE_RANGE = 0x19
    TMF8X2X_SHORT_RANGE_ACCURACY = 0x6e
    TMF8X2X_LONG_RANGE_ACCURACY = 0x6f
    TMF8X2X_ACCURACY_MODES_NOT_SUPPORTED = 0x00

    def __init__(self, ic_com:IcCom, log:bool=False ):
        """The default constructor. It initializes the TMF882X driver.
        Args:
            ic_com (IcCom): The communication class instance to talk i2c etc.
            log (bool, optional): Enable verbose driver outputs. False per default.
        """
        super().__init__(ic_com=ic_com,log=log)
        # use for clock correction xx pairs
        self.host_ticks = [ 0 for _ in range(Tmf882xApp.TMF8X2X_MAX_CLK_CORRECTION_PAIRS)]
        self.tmf882x_ticks = [ 0 for _ in range(Tmf882xApp.TMF8X2X_MAX_CLK_CORRECTION_PAIRS)]
        self.ticks_idx = 0
        self.histogram_is_complete = False
        self.histogram_id = 0
        self.histograms = [[0 for _ in range(TMF882X_BINS)] for _ in range(Tmf882xApp.TMF882X_CHANNELS)]
        self.histogram_header = [ "Bin_{}".format(i) for i in range(TMF882X_BINS) ]                         # make a nice header for e.g. CSV files
        self.mode = None            # do not know the mode
        self.factory_calibration : list = list()

    def _addClkCorrectionPair( self, host_tick : int, tmf882x_tick : int ):
        """ Add a host + device clock pair for clock correction
        Args:
            host_tick(int): The host tick at I2C read-out.
            tmf882x_tick(int): The device tick as part of the results structure.
        """
        if tmf882x_tick != 0:            # tmf882x ticks are only valid if lsb is 1, see datasheet for more details 
            self.host_ticks[ self.ticks_idx ] = host_tick
            self.tmf882x_ticks[ self.ticks_idx ] = tmf882x_tick
            self.ticks_idx = self.ticks_idx + 1
            if self.ticks_idx >= Tmf882xApp.TMF8X2X_MAX_CLK_CORRECTION_PAIRS :
                self.ticks_idx = 0          # wrap around

    def _correctDistance( self, distance_in_mm : int, log:bool=False ) -> int:
        """ Apply distance correction based on the stored clock correction pairs.
        Args:
            distance_in_mm(int): the measured distance
        Return:
            int: the corrected distance in mm
        """
        oldest_idx = self.ticks_idx                   # this is the oldest tick value index (the one that would be overwritten next)
        if oldest_idx > 0:
            newest_idx = oldest_idx - 1               # this is the newest tick value index (the one that was last written)
        else:
            newest_idx = Tmf882xApp.TMF8X2X_MAX_CLK_CORRECTION_PAIRS - 1 # take care of wrap over
        if ( self.tmf882x_ticks[ oldest_idx ] != 0 and self.tmf882x_ticks[ newest_idx ] != 0 ):     # valid pair of ticks
            denominator = self.tmf882x_ticks[ newest_idx ] - self.tmf882x_ticks[ oldest_idx ]
            nominator = self.host_ticks[ newest_idx ] - self.host_ticks[ oldest_idx ]
            if denominator == 0 :             # avoid diff by zero
                raise RuntimeError( "Division by zero, never should have the same device ticks for 2 measurements ")
            msg = "HostTicks={}, DeviceTicks={}, Ratio is = {}, Distance was {}mm".format( Tmf882xApp.TMF8X2X_CLK_CORRECTION_FACTOR * nominator, denominator, ( Tmf882xApp.TMF8X2X_CLK_CORRECTION_FACTOR * float(nominator) ) / float( denominator ), distance_in_mm )
            distance_in_mm = distance_in_mm * ( Tmf882xApp.TMF8X2X_CLK_CORRECTION_FACTOR * float(nominator) ) / float( denominator )
            if log: print( "{} is corrected to {}mm".format( msg, distance_in_mm ))
        return int(distance_in_mm + 0.5)            # round and truncate 

    def isAppRunning(self):
        """Check if the application is running.

        Returns:
            bool: True if the application is running, False if not
        """
        val = self.com.i2cTxRx(self.I2C_SLAVE_ADDR, [self.TMF8X2X_COM_APP_ID], 1)
        return val and val[0] == self.TMF8X2X_COM_APP_ID__application

    def getAppId(self):
        """Get the application version.

        Returns:
            [int, int, int, int]: app_id, minor, patch, build
        """
        val = self.com.i2cTxRx(self.I2C_SLAVE_ADDR, [self.TMF8X2X_COM_APP_ID], 4)
        return list(map(int,val))

    def getAppMode(self):
        """The the application mode (TMF8820/TMF8828)
        
        Returns:
            int: 8 if application is in TMF8828 mode, 0 otherwise
        """
        application_mode = self.com.i2cTxRx(self.I2C_SLAVE_ADDR, [self.TMF8828_COM_TMF8828_MODE], 1 )[0]
        return self.TMF8828_COM_TMF8828_MODE__mode__TMF8828 if application_mode == self.TMF8828_COM_TMF8828_MODE__mode__TMF8828 \
            else self.TMF8828_COM_TMF8828_MODE__mode__TMF8821

    def _checkRegister(self, regAddr:int, expected:int, timeout:float=0.010 ):
        """
        Check that the given register reads back with the expected value within the given time
        Args:
            regAddr (int): DESCRIPTION.
            expected (int): DESCRIPTION.
            timeout (float, optional): DESCRIPTION. Defaults to 0.010.
        Returns:
            Status: The status code (OK = 0, error != 0).
        """
        max_time = time.time() + timeout
        while True:
            rxed = self.com.i2cTxRx( self.I2C_SLAVE_ADDR, [regAddr], 1 )
            if not rxed:
                self._setError("Read register {} failed.".format(regAddr))
                return self.Status.DEV_ERROR
            if rxed[0] == expected:
                return self.Status.OK                                   
            if ( time.time() > max_time):
                self._setError("Read register {} timed out, expected value {}, read value {}".format(regAddr, expected, rxed[0]))
                return self.Status.TIMEOUT_ERROR

    def _sendCommand(self, cmd: int, timeout: float = 20e-3) -> Tmf882xDevice.Status:
        """Send a command to the TMF882X application, and check if it's accepted.
        Args:
            cmd (int): The command that the device shall execute.
            timeout (float, optional): _description_. Defaults to 20e-3.
        Returns:
            Status: The status code (OK = 0, error != 0).
        """
        self.com.i2cTx(self.I2C_SLAVE_ADDR, [self.TMF8X2X_COM_CMD_STAT, cmd])
        expected = self.TMF8X2X_COM_CMD_STAT__stat_ok
        if ( cmd == self.TMF8X2X_COM_CMD_STAT__cmd_measure ):
            expected = self.TMF8X2X_COM_CMD_STAT__stat_accepted
        return self._checkRegister(regAddr=self.TMF8X2X_COM_CMD_STAT, expected=expected, timeout=timeout )
        
    def loadConfig(self, config_page_cmd:int, timeout: float = 20e-3) -> Tmf882xDevice.Status:
        """Load the I2C configuration from the device into the register_buffer.  
        Args:
            config_page_cmd (int: The config page command to be loaded (e.g. TMF8X2X_COM_CMD_STAT__cmd_load_config_page_common)
            timeout (float, optional): The maximum time we allow the application to load the configuration. Defaults to 20e-3 == 20ms.
        Returns:
            Status: The status code (OK = 0, error != 0).
        """
        payload_addr = self.TMF8X2X_COM_CONFIG_RESULT
        number_regs =  self.TMF8X2X_COM_CONFIG_RESULT_LAST- self.TMF8X2X_COM_CONFIG_RESULT + 1 # all 
        # Tell the device to load the config, and wait until the firmware has load it.
        status = self._sendCommand(config_page_cmd, timeout)
        if status != self.Status.OK:
            return status # sendCommand already created an error entry.
        # Now read the data via I2C, and store it in th i2C buffer for easy access.
        val  = self.com.i2cTxRx(self.I2C_SLAVE_ADDR, [payload_addr], number_regs)
        if len(val) != number_regs:
            self._setError("Reading the loaded config failed")
            return self.Status.DEV_ERROR
        self.register_buffer[payload_addr:payload_addr+number_regs] = val
        return self.Status.OK
    
    def writeConfig(self, timeout: float = 40e-3) -> Tmf882xDevice.Status:
        """Write the I2C configuration from the register_buffer onto the device.  
        Args:
            timeout (float, optional): The maximum time we allow the application to write the configuration. Defaults to 20e-3 == 20ms.
        Returns:
            Status: The status code (OK = 0, error != 0).
        """
        payload_addr = self.TMF8X2X_COM_CONFIG_RESULT
        number_regs =  self.TMF8X2X_COM_CONFIG_RESULT_LAST- self.TMF8X2X_COM_CONFIG_RESULT + 1 # all 
        self.com.i2cTx(self.I2C_SLAVE_ADDR, [payload_addr] + self.register_buffer[payload_addr:payload_addr+number_regs])
        return self._sendCommand(self.TMF8X2X_COM_CMD_STAT__cmd_write_config_page, timeout)

    def configure(self,period_in_ms:int=33,kilo_iterations:int=537,spad_map_id:int=1,confidence_threshold:int=6,histograms:int=TMF8X2X_NO_HISTOGRAMS):
        """
        Function to configure some measurement parameters
        Args:
            period_in_ms (int): measurement period in milli-seconds. Defaults to 33.
            kilo_iterations (int, optional): Kilo-Iterations per measurement. Defaults to 537.
            spad_map_id (int, optional): Select one of the predefined maps (1..13). Defaults to 1.
            confidence_threshold (int, optional): Only if confidence for a target is equal or higher to this, 
            it will be reproted as an object. Defaults to 6.
            histograms(int,optional): whether histograms dumping is enabled or disabled. Defaults to no histograms.
        Returns:
            Status: The status code (OK = 0, error != 0).
        """
        status = self.loadConfig(self.TMF8X2X_COM_CMD_STAT__cmd_load_config_page_common)
        self.register_buffer[self.TMF8X2X_COM_PERIOD_MS_LSB] = period_in_ms % 256 
        self.register_buffer[self.TMF8X2X_COM_PERIOD_MS_MSB] = period_in_ms // 256
        self.register_buffer[self.TMF8X2X_COM_KILO_ITERATIONS_LSB] = kilo_iterations % 256
        self.register_buffer[self.TMF8X2X_COM_KILO_ITERATIONS_MSB] = kilo_iterations // 256
        self.register_buffer[self.TMF8X2X_COM_SPAD_MAP_ID] = spad_map_id
        self.register_buffer[self.TMF8X2X_COM_CONFIDENCE_THRESHOLD] = confidence_threshold
        self.register_buffer[self.TMF8X2X_COM_HIST_DUMP] = histograms
        if ( status == self.Status.OK ):
            return self.writeConfig()
    
    def configureResultReporting(self,config:int):
        # 0x35 TMF8X2X_COM_ALG_SETTING_0 0   electrical_calibration     RW  0  # if set do report electrical calibration results
        # 0x35 TMF8X2X_COM_ALG_SETTING_0 1   statistics                 RW  0  # if set do report statistic information results
        # 0x35 TMF8X2X_COM_ALG_SETTING_0 2   distances                  RW  1  # if set do report distance results
        # 0x35 TMF8X2X_COM_ALG_SETTING_0 4:3 distance_mode              RW  0  # 0 ... 5m mode, 1 ... 2.5 m mode   
        # 0x35 TMF8X2X_COM_ALG_SETTING_0 6   diagnostics                RW  0  # if set do report also diagnostics in distance results
        # 0x35 TMF8X2X_COM_ALG_SETTING_0 7   increase_confidence_range  RW  0  # if set do enable increased range of the object zone confidences
        """
        Function to configure the reporting of results
        Args:
            config (int): configuration (see description of options above)
        Returns:
            Status: The status code (OK = 0, error != 0).
        """
        status = self.loadConfig(self.TMF8X2X_COM_CMD_STAT__cmd_load_config_page_common)
        self.register_buffer[self.TMF8X2X_COM_ALG_SETTING_0] = config
        if ( status == self.Status.OK ):
            return self.writeConfig()
            
    def getAccuracy(self):
        """
        Get the current accuracy mode (long range, short range, accuracy switching not supported).
        
        Returns:
            int: the value from the range status register
        """
        return self.com.i2cTxRx(self.I2C_SLAVE_ADDR, [self.TMF8X2X_REG_ACTIVE_RANGE], 1)[0]

    def setLongRangeAccuracy(self):
        """
        Switch the target to long range accuracy mode.

        Returns:
            Status: The status code (OK = 0, error != 0).
        """
        return self._sendCommand(cmd=self.TMF8X2X_LONG_RANGE_ACCURACY,timeout=0.1)

    def setShortRangeAccuracy(self):
        """Switch the target to short range accuracy mode.

        Returns:
            Status: The status code (OK = 0, error != 0).
        """
        return self._sendCommand(cmd=self.TMF8X2X_SHORT_RANGE_ACCURACY,timeout=0.1)

    def runFactoryCalibration(self, kilo_iterations:int=4000, spad_map_id:int=1, timeout:float=4.0 ):
        """Function to perform factory calibration for a specified SPAD map
        Args:
            kilo_iterations (int, optional): Kilo-Iterations per measurement. Defaults to 4000.
            spad_map_id (int, optional): Select one of the predefined maps (1..13). Defaults to 1.
            timout (float,optional): how long to wait for factory calibration completion
        Returns:
            Status: The status code (OK = 0, error != 0).
        """
        if self.Status.OK == self.configure(kilo_iterations=kilo_iterations,spad_map_id=spad_map_id):
            if self.mode == self.TMF8828_COM_TMF8828_MODE__mode__TMF8821:
                return self._sendCommand(self.TMF8X2X_COM_CMD_STAT__cmd_factory_calibration,timeout=timeout)
            else:
                status = self._sendCommand(self.TMF8X2X_COM_CMD_STAT__cmd_factory_calibration_reset,timeout=timeout)
                if status != self.Status.OK: return status
                for i in range(self.TMF8828_CALIBRATION_ITERATIONS):
                    status = self._sendCommand(self.TMF8X2X_COM_CMD_STAT__cmd_factory_calibration,timeout=timeout)
                    if status != self.Status.OK:
                        break
                return status
        else:
            return self.Status.DEV_ERROR

    def readFactoryCalibration(self, timeout:float=0.020):
        """
        Read back the factory calibration from the device
        
        Args:
            timeout (float, optional): command timeout in seconds

        Returns:
            Status: The status code (OK = 0, error != 0).
        """
        self.factory_calibration = []
        calibration_iterations = self.TMF8820_CALIBRATION_ITERATIONS

        if self.mode == self.TMF8828_COM_TMF8828_MODE__mode__TMF8828:
            calibration_iterations = self.TMF8828_CALIBRATION_ITERATIONS
            status = self._sendCommand(self.TMF8X2X_COM_CMD_STAT__cmd_factory_calibration_reset,timeout=timeout)
            if status != self.Status.OK: return status

        for i in range(calibration_iterations):
            status = self._sendCommand(self.TMF8X2X_COM_CMD_STAT__cmd_load_config_page_factory_calib, timeout=timeout)
            if status != self.Status.OK: return status
            val = self.com.i2cTxRx(self.I2C_SLAVE_ADDR, [self.TMF8X2X_COM_FC_START], self.TMF8X2X_COM_CONFIG_FACTORY_CALIB__factory_calibration_size)
            if len(val) != self.TMF8X2X_COM_CONFIG_FACTORY_CALIB__factory_calibration_size:
                self._setError("Reading the factory calibration failed")
                return self.Status.DEV_ERROR
            status = self._sendCommand(self.TMF8X2X_COM_CMD_STAT__cmd_write_config_page, timeout=timeout)
            if status != self.Status.OK: return status
            self.factory_calibration.append(list(val))

        return self.Status.OK

    def getFactoryCalibrationData(self):
        """
        Provide the factory calibration data

        Returns:
            list(list) or Status: List of factory calibration records. A single record for TMF8820, TMF8821. Four records for TMF8828. Or an error enumeration
        """
        if (self.readFactoryCalibration() == self.Status.OK ):
            return self.factory_calibration
        else:
            return self.Status.DEV_ERROR

    def _dumpSingleCalibration(self, cal : list ):
        """
        Dump a single calibration record
        
        Args:
            cal (list): List of calibration values.
        """
        if len(cal) == self.TMF8X2X_COM_CONFIG_FACTORY_CALIB__factory_calibration_size:
            calibrationIterations = int.from_bytes(bytes(cal[self.TMF882X_FC_KITERS:self.TMF882X_FC_KITERS+self.TMF882X_FC_KITERS_SIZE]),'little')
            # avoid divide by zero errors
            if calibrationIterations == 0:
                calibrationIterations = 1
            calibrationStatus = cal[self.TMF882X_FC_STATUS]
            print("-")
            print(f"Calibration Iterations: {calibrationIterations}k")
            print(f"Calibration Status: 0x{calibrationStatus:02X}")
            print("Calibration successful" if calibrationStatus == 0x00 else f"Calibration failed with status 0x{calibrationStatus:02X}")
            print(f"Crosstalk Values:")

            channel = 0
            for captureOffset in (self.TMF882X_FC_XTALK_AMPLITUDE_Z0_0,self.TMF882X_FC_XTALK_AMPLITUDE_Z0_1):
                for offset in range(captureOffset,captureOffset+self.TMF882X_CROSSTALK_LENGTH_PER_CAPTURE,self.TMF882X_CROSSTALK_SIZE):
                    # 32bit little endian representation
                    crossTalk = int.from_bytes(bytes(cal[offset:offset+self.TMF882X_CROSSTALK_SIZE]),'little')
                    print(f"Channel #{channel:02}: {crossTalk:12} | {crossTalk*self.TMF882X_CROSSTALK_REFERENCE_ITERATIONS_K//calibrationIterations:12} @ {self.TMF882X_CROSSTALK_REFERENCE_ITERATIONS_K}k {'(reference channel)' if channel % self.TMF882X_CROSSTALK_NUMBER_OF_CHANNELS_PER_CAPTURE == 0 else ''}")
                    channel += 1

    def dumpFactoryCalibration(self):
        """Dump all factory calibration records in a nice format"""
        for cal in self.factory_calibration:
            self._dumpSingleCalibration(cal)     

    def loadFactoryCalibration(self, fc : list, timeout:float=0.010):
        """
        Write a previously collected factory calibration back to the sensor
        
        Args:
            fc (list(list)): List of calibration records (TMF8828) or list of calibration values.
            timeout (float, optional): command timeout in seconds
        Returns:
            Status: The status code (OK = 0, error != 0).
        """
        if self.mode == self.TMF8828_COM_TMF8828_MODE__mode__TMF8821:
            # make the reg-addr + 4 byte header
            _data =  [ self.TMF8X2X_COM_CONFIG_RESULT, self.TMF8X2X_COM_CMD_STAT__cmd_load_config_page_factory_calib, 0x00, 0x00, self.TMF8X2X_COM_CONFIG_FACTORY_CALIB__factory_calibration_size ]
            _data = _data + fc[0]
            status = self.com.i2cTx(self.I2C_SLAVE_ADDR, _data)
            if status == self.Status.OK:
                status = self._sendCommand(self.TMF8X2X_COM_CMD_STAT__cmd_write_config_page, timeout=timeout)
            return status
        else:
            status = self._sendCommand(self.TMF8X2X_COM_CMD_STAT__cmd_factory_calibration_reset,timeout=timeout)
            if status != self.Status.OK: return status
            for i in range(self.TMF8828_CALIBRATION_ITERATIONS):
                status = self._sendCommand(self.TMF8X2X_COM_CMD_STAT__cmd_load_config_page_factory_calib, timeout=timeout)
                if status != self.Status.OK: return status
                status = self.com.i2cTx(self.I2C_SLAVE_ADDR, [self.TMF8X2X_COM_FC_START] + fc[i])
                if status != self.Status.OK: return status
                status = self._sendCommand(self.TMF8X2X_COM_CMD_STAT__cmd_write_config_page, timeout=timeout)
                if status != self.Status.OK: return status            
            return status

    def _setMode(self, modeCmd:int, modeValue:int, timeout:float=0.020 ):
        """
        Function to change mode to 8x8 or 3x3/4x4/3x6
        Args:
            modeCmd (int): operation mode.
            modeValue (int): operation mode value .
            timeout (float, optional): command timeout. Defaults to 0.020.
        Returns:
            Status: The status code (OK = 0, error != 0).
        """
        status = self._sendCommand(modeCmd)
        if ( status ==  self.Status.OK ):
            status = self._checkRegister(regAddr=self.TMF8828_COM_TMF8828_MODE, expected=modeValue, timeout=timeout)
            if status == self.Status.OK:
                self.mode = modeValue       # mode has successfully changed
        return status

    def setLegacyMode(self, timeout:float=0.020):
        """
        Function to change into legacy mode 3x3,4x4 or 3x6
        Args:
            timeout (float, optional): after this time if switch did not happen, this is an severe error. Defaults to 0.020.
        Returns:
            Status: The status code (OK = 0, error != 0).
        """
        return self._setMode( modeCmd=self.TMF8828_COM_CMD_STAT__cmd_stat__CMD_SWITCH_TMF8821_MODE, modeValue=self.TMF8828_COM_TMF8828_MODE__mode__TMF8821, timeout=timeout )

    def set8x8Mode(self, timeout:float=0.020):
        """
        Function to change into 8x8 mode
        Args:
            timeout (float, optional): after this time if switch did not happen, this is an severe error. Defaults to 0.020.
        Returns:
            Status: The status code (OK = 0, error != 0).
        """
        return self._setMode( modeCmd=self.TMF8828_COM_CMD_STAT__cmd_stat__CMD_SWITCH_TMF8828_MODE, modeValue=self.TMF8828_COM_TMF8828_MODE__mode__TMF8828, timeout=timeout )
        
    def startMeasure(self, timeout: float = 20e-3):
        """
        Functions clears and enables result interrupts, and starts a measurement.
        Args:
            timeout (float, optional): command timeout. Defaults to 0.020.        
        Returns:
            Status: The status code (OK = 0, error != 0).
        """
        self.host_ticks = [ 0 for _ in range(Tmf882xApp.TMF8X2X_MAX_CLK_CORRECTION_PAIRS)]
        self.tmf882x_ticks = [ 0 for _ in range(Tmf882xApp.TMF8X2X_MAX_CLK_CORRECTION_PAIRS)]
        self.clearAndEnableInt(self.TMF8X2X_APP_I2C_RESULT_IRQ_MASK | self.TMF8X2X_APP_I2C_RAW_HISTOGRAM_IRQ_MASK)
        return self._sendCommand(self.TMF8X2X_COM_CMD_STAT__cmd_measure,timeout=timeout)
        
    def stopMeasure(self, timeout: float = 20e-3):
        """
        Function stops a measurement and disables all interrupts. 
        Args:
            timeout (float, optional): command timeout. Defaults to 0.020.        
        Returns:
            Status: The status code (OK = 0, error != 0).
        """
        status = self._sendCommand(self.TMF8X2X_COM_CMD_STAT__cmd_stop, timeout=timeout)
        self.enableInt(0)           # enable no interrupt == disable all
        return status

    def isDiagnosticInterrupt(self)->bool:
        """
        Check if the diagnostic interrupt bit is set in the INT_STATUS register
        Returns:
            bool: True if the diagnostic interrupt has been triggered
        """
        return self.readIntStatus() & self.TMF8X2X_APP_I2C_RAW_HISTOGRAM_IRQ_MASK 
    
    def isResultInterrupt(self)->bool:
        """
        Check if the result interrupt bit is set in the INT_STATUS register
        Returns:
            bool: True if the result interrupt has been triggered
        """
        return self.readIntStatus() & self.TMF8X2X_APP_I2C_RESULT_IRQ_MASK 

    def readResult(self) -> Tmf882xResultFrame:
        """
        Function reads a single result frame 
        Returns:
            the read in result frame
        """
        fn_name = "readResult"
        frame = Tmf882xResultFrame()
        raw = self.com.i2cTxRx( self.I2C_SLAVE_ADDR,[self.TMF8X2X_COM_CONFIG_RESULT],self.TMF8X2X_COM_RESULT_FRAME_SIZE)
        host_timestamp = int( time.time() * 1000 * 1000 )                                   # need a close as possible timestamp from the host
        if ( len(raw) == 0 or raw[0] != self.TMF8X2X_COM_CMD_STAT__cmd_measure):            # measure results must start with a 0x10 as cid_rid
            raw = bytearray(self.TMF8X2X_COM_RESULT_FRAME_SIZE)                             # make a frame full of 0s
            self._setError("{} Not a frame".format(fn_name))
        frame.decode(raw)
        self._addClkCorrectionPair( host_timestamp, frame.resultHeader.systemTicks )         # add the clk correction pair, does not do any correction yet, 
        return frame

    def applyClkCorrection(self, frame:Tmf882xResultFrame, apply_to_2nd_object:bool=True, log:bool=False) -> Tmf882xResultFrame:
        """Apply clock correction to all 1st objects and to 2nd objects too
        Args:
            frame(Tmf882xResultFrame): a measurenment result frame, to which the clock corrected distance is applied
            apply_to_2nd_object(bool,True): if clock correction shall be applied to the 2nd object too (do not apply if 2nd object is intensity)
        Returns:
            the clock corrected frame
        """
        frame2 = frame
        for i in range(TMF882X_APP_MAX_ZONES):
            frame2.object1[i].distanceInMm = self._correctDistance(frame.object1[i].distanceInMm,log)
        if apply_to_2nd_object:
            for i in range(TMF882X_APP_MAX_ZONES):
                frame2.object2[i].distanceInMm = self._correctDistance(frame.object2[i].distanceInMm,log)
        return frame2

    def readHistogram(self) -> Tmf882xRawHistogramFrame:
        """
        Function reads a single histogram frame 
        Returns:
            the read in histogram frame
        """
        fn_name = "readHistogram"
        frame = Tmf882xRawHistogramFrame()
        raw = self.com.i2cTxRx( self.I2C_SLAVE_ADDR,[self.TMF8X2X_COM_CONFIG_RESULT],self.TMF8X2X_COM_HISTOGRAM_FRAME_SIZE)
        if ( len(raw) == 0 or ( raw[0] & Tmf882xApp.TMF8X2X_HISTOGRAM_ID ) != Tmf882xApp.TMF8X2X_HISTOGRAM_ID ):           # histograms must have the multi-packet bit set
            raw = bytearray(self.TMF8X2X_COM_HISTOGRAM_FRAME_SIZE)         # make a frame full of 0s
            self._setError("{} Not a frame".format(fn_name))
            print( "ERROR not a frame ")
        frame.decode(raw)
        # now sum up the histograms, find the channel number, and which of the 3 bytes it is.
        channel = frame.subHeader.number % Tmf882xApp.TMF882X_CHANNELS      # the subHeader number 0 == channel 0, 1 == channel 1, ... 10 == channel 0 (mid-byte), 11 == channel 1 (mid-byte), ...
        factor = ( frame.subHeader.number // Tmf882xApp.TMF882X_CHANNELS )
        if factor == 1:
            factor = 256                # mid-byte
        elif factor == 2:
            factor = 256*256            # msbyte
        else:
            factor = 1                  #lsbyte
        if frame.subHeader.number == 0:                             # new set of histograms start, clear the content of histogram buffer 
            self.histograms = [[0 for _ in range(TMF882X_BINS)] for _ in range(Tmf882xApp.TMF882X_CHANNELS)]
            self.histogram_is_complete = False
        for bin in range(TMF882X_BINS):
            self.histograms[channel][bin] = self.histograms[channel][bin] + factor * frame.data[bin]      # sum up the 3 bytes
        if frame.subHeader.number == Tmf882xApp.TMF882X_CHANNELS * 3 - 1:
            self.histogram_is_complete = True
            self.histogram_id = frame.header.cid_rid
        return frame

    def readResultInt(self, timeout:float=1.0) -> Tmf882xResultFrame:
        """
        Read a result if the interrupt is set, and return the result frame
        Args:
            timeout (float, optional): How long to wait for an interrupt to occur. Defaults to 1.0.
        Returns:
            Tmf882xResultFrame: result frame read in 
        """
        max_time = time.time() + timeout
        while True:
            interrupt = self.readAndClearInt(self.TMF8X2X_APP_I2C_RESULT_IRQ_MASK)
            if ( interrupt == self.TMF8X2X_APP_I2C_RESULT_IRQ_MASK ):
                return self.readResult() 
            if ( time.time() > max_time ):
                raise RuntimeError("TMF882x Error: readResultInt did timeout" )

    def readHistogramInt(self, timeout:float=1.0):
        """
        Read a result if the interrupt is set, and return the result frame
        Args:
            timeout (float, optional): How long to wait for an interrupt to occur. Defaults to 1.0.
        Returns:
            the read in histogram frame
        """
        max_time = time.time() + timeout
        while True:
            interrupt = self.readAndClearInt(self.TMF8X2X_APP_I2C_RAW_HISTOGRAM_IRQ_MASK)
            if ( interrupt == self.TMF8X2X_APP_I2C_RAW_HISTOGRAM_IRQ_MASK ):
                return self.readHistogram() 
            if ( time.time() > max_time ):
                raise RuntimeError("TMF882x Error: readHistogramInt did timeout" )

    def getHistogramFieldsText(self,frame:Tmf882xRawHistogramFrame) -> list:
        """
        Generate a header row for each histogram field later printing e.g. to CSV
        Args:
            frame (Tmf882xRawHistogramFrame): the histogram frame
        Returns:
            list: The result frame header fields.
        """
        header = []
        for i in range(ctypes.sizeof(frame.data)):
            header.append("Data{}".format(i))
        return header

    def getResultFieldsText(self,frame:Tmf882xResultFrame) -> list:
        """
        Generate a header row for each result field later printing e.g. to CSV
        Args:
            frame (Tmf882xResultFrame): the result frame
        Returns:
            list: The result frame header fields.
        """
        header = []
        for field_name, field_type in frame.header._fields_:
            header.append(field_name)
        for field_name, field_type in frame.resultHeader._fields_:
            header.append(field_name)
        idx = 0
        for object1 in frame.object1:
            for field_name, field_type in Tmf882xResultFramePayload._fields_:
                txt = ("Object_1_Pixel_{}_{}".format(idx,field_name))
                header.append(txt)
            idx = idx + 1
        idx = 0
        for object2 in frame.object2:
            for field_name, field_type in Tmf882xResultFramePayload._fields_:
                txt = ("Object_2_Pixel_{}_{}".format(idx,field_name))
                header.append(txt)
            idx = idx + 1
        return header
    
    def getResultFields(self, frame:Tmf882xResultFrame ):
        """
        Generate a data row for each result field later printing e.g. to CSV
        Args:
            frame (Tmf882xResultFrame): the result frame
        Returns:
            list,dict: The result frame data fields as list and as a dictionary.
        """
        objects = {}
        row = []
        for field_name, field_type in frame.header._fields_:
            row.append(getattr( frame.header, field_name ))
        for field_name, field_type in frame.resultHeader._fields_:
            row.append(getattr( frame.resultHeader, field_name ))
        idx = 0
        objects["object1"] = []
        objects["object2"] = []
        for object1 in frame.object1:
            for field_name, field_type in Tmf882xResultFramePayload._fields_:
                row.append(getattr( object1, field_name))
            if object1.confidence > 0:                                                      # only if there is an target detected in this zone
                obj = {
                    "confidence" : object1.confidence,
                    "distance_mm" : object1.distanceInMm,
                    "channel" : 1 + (idx % (Tmf882xApp.TMF882X_CHANNELS-1) ),                   # there are 9 channels, starting from 1..9
                    "ch_target_idx" : 1,                                                    # if there is just one object per channel this is always the first object
                    "sub_capture" :( idx // (Tmf882xApp.TMF882X_CHANNELS-1))              # either sub-capture 0 (==objects[0..9]) or 1 (==objects[10..17]) 
                }
                objects["object1"].append( obj )
            idx = idx + 1
        idx = 0
        for object2 in frame.object2:
            for field_name, field_type in Tmf882xResultFramePayload._fields_:
                row.append(getattr( object2, field_name))
            if object2.confidence > 0:                                                      # only if there is an target detected in this zone
                obj = {
                    "confidence" : object2.confidence,
                    "distance_mm" : object2.distanceInMm,
                    "channel" : 1 + (idx % (Tmf882xApp.TMF882X_CHANNELS-1) ),                   # there are 9 channels, starting from 1..9
                    "ch_target_idx" : 2,                                                    # 2nd object is always comming after the first, so if there is an object here it is 2nd
                    "sub_capture" :( idx // (Tmf882xApp.TMF882X_CHANNELS-1))              # either sub-capture 0 (==objects[0..9]) or 1 (==objects[10..17]) 
                }
                objects["object2"].append( obj )
            idx = idx + 1
        return row, objects

    def getResultIntensityFieldsText(self,frame:Tmf882xResultFrame) -> list:
        """
        Generate a header row for each result field later printing e.g. to CSV
        Args:
            frame (Tmf882xResultFrame): the result frame
        Returns:
            list: The result frame header fields .
        """
        header = []
        for field_name, field_type in frame.header._fields_:
            header.append(field_name)
        for field_name, field_type in frame.resultHeader._fields_:
            header.append(field_name)
        idx = 0
        for object1 in frame.object1:
            for field_name, field_type in Tmf882xResultFramePayload._fields_:
                txt = ("Object_1_Pixel_{}_{}".format(idx,field_name))
                header.append(txt)
            idx = idx + 1
        idx = 0
        for object2 in frame.object2:
            txt = ("Object_2_Pixel_{}_intensity".format(idx))
            header.append(txt)
            idx = idx + 1
        return header

    def getResultIntensityFields(self, frame:Tmf882xResultFrame ):
        """
        Generate a data row for each result field later printing e.g. to CSV
        Args:
            frame (Tmf882xResultFrame): the result frame
        Returns:
            list, dict: The result frame data fields as list and as a dictionary.
        """
        objects = {}
        row = []
        for field_name, field_type in frame.header._fields_:
            row.append(getattr( frame.header, field_name ))
        for field_name, field_type in frame.resultHeader._fields_:
            row.append(getattr( frame.resultHeader, field_name ))
        idx = 0
        for object1 in frame.object1:
            for field_name, field_type in Tmf882xResultFramePayload._fields_:
                row.append(getattr( object1, field_name))
            if object1.confidence > 0:                                                      # only if there is an target detected in this zone
                obj = {}
                obj["confidence"] = object1.confidence
                obj["distance_mm"] = object1.distanceInMm
                obj["channel"] = 1 + (idx % (Tmf882xApp.TMF882X_CHANNELS-1) )               # there are 9 channels, starting from 1..9
                obj["ch_target_idx"] = 1                                                    # if there is just one object per channel this is always the first object
                obj["sub_capture"] = ( idx // (Tmf882xApp.TMF882X_CHANNELS-1))              # either sub-capture 0 (==objects[0..9]) or 1 (==objects[10..17]) 
                objects.update(obj)
            idx = idx + 1
        idx = 0
        for intensity in frame.object2:
            val1 = getattr( intensity, 'confidence' )
            val2 = getattr( intensity, 'distanceInMm' )
            value = val1 * 256*256 + val2
            row.append(value)
            idx = idx + 1
        return row, objects

    #------------------------------------------ bootloader below ----------------------------------------------

    @staticmethod
    def _computeBootloaderChecksum(data: List[int]) -> int:
        """Compute the bootloader checksum over an array.
        Args:
            data (List[int]): The array to compute the checksum over
        Returns:
            int: the checksum
        """
        return 0xff ^ sum(data) & 0xff

    @staticmethod
    def _appendChecksumToFrame(frame: List[int]) -> None:
        """Append a checksum it to the I2C frame.
        Args:
            frame (List[int]): The I2C command frame.
        """
        #The frame checksum is computed over all but the register byte
        checksum = Tmf882xApp._computeBootloaderChecksum(frame[1:])
        frame.append(checksum)


    def _bootloaderSendCommand(self, cmd:int, payload:List[int] = [], response_payload_len: int = 0, timeout:float=0.02):
        """
        Send a command with payload, and read back response_payload_len bytes.
        
        Args:
            cmd (int): command code
            payload (List[int]): command payload
            response_payload_len: length of the response payload after command execution
            timeout (float, optional): command timeout in seconds
        Returns:
            Status: The status code (OK = 0, error != 0).
        """
        # The write frame consists of a command register address, command, payload len, payload, and crc
        write_frame = [self.TMF8X2X_COM_CMD_STAT, cmd, len(payload)] + payload
        # The read frame is the command register address 
        read_frame = [self.TMF8X2X_COM_CMD_STAT]
        self._appendChecksumToFrame(write_frame)
        self.com.i2cTx(self.I2C_SLAVE_ADDR, write_frame)
        
        max_time = time.time() + timeout
        while time.time() < max_time:
            # read back status + payload_len + payload + crc
            response = self.com.i2cTxRx(self.I2C_SLAVE_ADDR, read_frame, 3 + response_payload_len)
            if len(response) != 3 + response_payload_len:
                self._setError("The application did not accept frame {}. Response is {}.".format(write_frame, response))
                return self.Status.APP_ERROR, []
            if response[0] != cmd:
                #response is ready, check if the frame is okay.
                cmd_status = response[0]
                actual_payload_len = response[1]
                payload = response[1:-1]
                checksum = response[-1]

                # Collect errors, and report at once.
                error = ""
                if cmd_status != self.TMF8X2X_COM_CMD_STAT__stat_ok:
                    error += "The bootloader returned cmd_status {}.".format(cmd_status)
                if actual_payload_len != response_payload_len:
                    error += "The bootloader payload response length should be {}, is {}.".format(actual_payload_len, response_payload_len)
                if self._computeBootloaderChecksum(payload) != checksum:
                    error += "The checksum {} does not match to the frame content.".format(checksum)
                
                if error:
                    self._setError("{}\n Write Frame: {}, Read Frame: {}, Response {}.".format(error, write_frame, read_frame, response))
                    return self.Status.APP_ERROR, bytearray()  
                else:
                    #every check passed, return payload data.
                    return self.Status.OK, payload
        # timed out
        self._setError("The bootloader frame {} timed out after {}s.".format(write_frame, timeout))
        return self.Status.TIMEOUT_ERROR, []
        
    def _bootLoaderDownloadData(self, target_address: int, data: bytearray,  timeout: float = 20e-3) -> Tmf882xDevice.Status:
        """Load a data chunk to the target at a specific address.
        Args:
            target_address (int): The address on the target.
            data (bytearray): The data to be written onto the target. 
        Returns:
            Status: The status code (OK = 0, error != 0).
        """
        # 16-bit RAM address in little endian format
        target_address_bytes = [target_address & 0xff, (target_address >> 8) & 0xff]
        #First, send the target RAM address (little endian)
        status, _ = self._bootloaderSendCommand(self.TMF8X2X_COM_CMD_STAT__bl_cmd_addr_ram, target_address_bytes)
        if status != self.Status.OK:
            self._setError("Setting RAM address {} failed.".format(target_address))
            return status
        
        #Set the maximum chunk size that can be transferred at once.
        max_chunk_len = self.TMF8X2X_BL_MAX_DATA_SIZE
        # Split the big bytearray into smaller chunks that can be transferred with single I2C bulk writes.
        for data_idx in range(0,len(data), max_chunk_len):
            payload_data = data[data_idx: data_idx + max_chunk_len]
            self._log("Loading address 0x{:x} chunk with {} bytes.".format(target_address + data_idx, len(payload_data)))
            # Write the payload of one chunk
            status, _ = self._bootloaderSendCommand(self.TMF8X2X_COM_CMD_STAT__bl_cmd_w_ram, list(payload_data))
            if status != self.Status.OK:
                self._setError("Writing RAM chunk {} failed.".format(payload_data))
                return status  
        return self.Status.OK
        
    def downloadHexFile(self, hex_file: str, timeout:float = 20e-3) ->Tmf882xDevice.Status:
        """Download a application/patch hex file to the device.
           To run the application, call 
        Args:
            hex_file (str): The firmware/patch to load.
            timeout (float, optional): The timeout for the device to respond on a command. Defaults to 20e-3 == 20ms.
        Returns:
            Status: The status code (OK = 0, error != 0).
        """
        segments = []
        try:
            intel_hex = IntelHex()
            intel_hex.fromfile(hex_file, format='hex')
            # Load the segments.
            segments = intel_hex.segments()
            if len(segments) != 1:
                self._log("Warning - Expecting only 1 segment, but found {}".format(len(segments)))
        except Exception as e:
            self._setError("Error with hex file {}: {}".format(hex_file, str(e)))
            return self.Status.OTHER_ERROR
        for start_segment, end_segment in segments:
            self._log("Loading SYS image segment start: {:x}, end: {:x}".format(start_segment, end_segment))
            status = self._bootLoaderDownloadData(start_segment, intel_hex.tobinarray(start= start_segment, size= end_segment - start_segment))
            if status != self.Status.OK:
                return status
        return self.Status.OK
    
    def startRamApp(self, timeout: float = 20e-3) -> Tmf882xDevice.Status:
        """Start the ROM application from the bootloader.
        Args:
            timeout (float, optional): The communication timeout. Defaults to 20e-3.
        Returns:
            Status: The status code (OK = 0, error != 0).
        """
        ram_remap_cmd = [self.TMF8X2X_COM_CMD_STAT, self.TMF8X2X_COM_CMD_STAT__bl_cmd_ramremap, 0x00]
        self._appendChecksumToFrame(ram_remap_cmd)
        self.com.i2cTx(self.I2C_SLAVE_ADDR, ram_remap_cmd)
            
        max_time = time.time() + timeout
        while True:                                     # make do-while loop, for debugging nicer
            if self.isAppRunning() == True:
                self.mode = self.getAppMode()
                return self.Status.OK
            if (time.time() > max_time):
                self._setError("The application did not start within {} seconds".format(timeout))
                return self.Status.TIMEOUT_ERROR

    def downloadAndStartApp(self, hex_file: str, timeout:float=0.020 ) -> Tmf882xDevice.Status:
        """
        Convenience function: does download a hex file and start the downloaded applicaiton
        Args:
            hex_file (str): The firmware/patch to load.
            timeout (float, optional): Wait time in communication before give up. Defaults to 0.020.
        Returns:
            Status: The status code (OK = 0, error != 0).
        """
        status = self.downloadHexFile(hex_file,timeout)
        if self.Status.OK == status:
            return self.startRamApp(timeout)
        else:
            return status

if __name__ == "__main__":
    print("This program is not intended for standalone operation. Include it into application programs.")        
