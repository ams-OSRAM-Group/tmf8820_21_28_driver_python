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
- test the functions of the Tmf882xApp class. 
- run optical measurements in 3x3,4x4 and 8x8 mode.
- check factory calibration.
- test in short range and long range accuracy mode.
"""

import __init__
import pytest
import sys
from tmf882x.tmf882x_app import Tmf882xApp
import time
import os
import pathlib

# set to "True" if you want to use the EVM controller board, "False" for the FT2232 controller board
use_evm = True

spad_maps_3x3 : list = [ 
    Tmf882xApp.TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_1, 
    Tmf882xApp.TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_2,
    Tmf882xApp.TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_3,
    Tmf882xApp.TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_6,
    Tmf882xApp.TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_11,
    Tmf882xApp.TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_12
]

spad_maps_4x4 : list = [ 
    Tmf882xApp.TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_4, 
    Tmf882xApp.TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_5,
    Tmf882xApp.TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_7,
    Tmf882xApp.TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_13
]

class TestTmf8820App:

    log : bool                    = False
    number_of_frames : int        = 10
    # minimum and maximum allowed distance from sensor to target to match you requirements    
    minimum_target_distance : int = 300 # update to match your test setup, no target within the first 400mm to allow for factory calibration
    maximum_target_distance : int = 500 # update to match your test setup

    I2C_SLAVE_ADDR : int         = 0x41
    REG_CALIBRATION_STATUS       = 0x07
    HEX_FILE = os.path.dirname(__file__) + "\\..\\..\\..\\firmware\\tmf8x2x_fw_application_patch\\RAM_patch\\tmf8x2x_application_patch.hex"
    FACTORY_CALIBRATION_TIMEOUT  = 8.0 # seconds 

    active_spad_map : int = 0
    short_range_mode_active : bool = False

    def setup_class(self):
        # create FTDI communication instance
        if use_evm:
            from aos_com.evm_ftdi import EvmFtdi as Ftdi
        else:
            from aos_com.ft2232_ftdi import Ft2232Ftdi as Ftdi

        self.com = Ftdi(log=False,exception_on_error=True)
        self.tof_app = Tmf882xApp(ic_com=self.com)

        # open communication at host side                 
        if ( Tmf882xApp.Status.OK != self.tof_app.open(i2c_speed=400000) ):  # open FTDI communication channels
            raise RuntimeError( "Error open FTDI device" )

        self.tof_app.enable()                                          
        self.tof_app.downloadAndStartApp(self.HEX_FILE)

        time.sleep(1.0) # give the application some time to fully start up.
        if self.log: 
            application_version = self.tof_app.getAppId()
            print(f"Application {application_version} started")
            print(f"Application Mode {self.tof_app.getAppMode():02X}")
            _, _, _, build = application_version
            if build & 0x10: # bit4 in build byte set -> short range mode supported
                print("Application supports short range mode")
            else:
                print("Application does not support short range mode")

    def teardown_class(self):
        self.tof_app.disable()
        self.tof_app.close()

    def show_distances(self, measurement_data:list, nr_of_columns: int ):        
        print(f"----------------  SPAD Map#{self.active_spad_map} Short Range Mode:{self.short_range_mode_active}")
        print(f"[RAW]{measurement_data}")
        current_column = 0
        for _, distance in zip(measurement_data[::2],measurement_data[1::2]):
            current_column += 1 
            print( f"{distance:03} ", end="" if current_column % nr_of_columns else "\n")
        print("----------------")

    def check_data(self, data):
        """Check if the measured distances are within the predefined boundaries"""
        columns = 3

        if self.active_spad_map in spad_maps_3x3:
            data = data[11:29]

        if ( ( self.active_spad_map in spad_maps_4x4 ) or ( self.active_spad_map == Tmf882xApp.TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_15 ) ):
            data = data[11:27] + data[29:45]
            columns = 4

        if self.log:
            self.show_distances(measurement_data=data,nr_of_columns=columns)

        distance_counter = 0
        for _, distance in zip(data[::2],data[1::2]):
            if distance > 0: # not all zones have to report a distance all the time
                assert ( ( distance >= self.minimum_target_distance ) and ( distance <= self.maximum_target_distance ) ), \
                    f"Target distance out of range {self.minimum_target_distance}|{distance}|{self.maximum_target_distance}, Distance# {distance_counter} SPAD Map: {self.active_spad_map}"
            distance_counter += 1

    def measure(self, check_data : bool = True):
        """Start a measurement and read number_of_frames, then stop the measurement"""
        assert self.is_calibrated() == True, "Device not calibrated"
        self.tof_app.startMeasure()
        read_frames = 0
        while read_frames < self.number_of_frames:
            if ( self.tof_app.readAndClearInt( self.tof_app.TMF8X2X_APP_I2C_RESULT_IRQ_MASK ) ):
                read_frames = read_frames + 1
                frame = self.tof_app.readResult()
                # disable clock correction for this test setup
                # self.tof_app.applyClkCorrection(frame)
                data_list, _ = self.tof_app.getResultFields(frame)
                if self.log: 
                    print( f"Frame[{read_frames:02}]:", data_list ) # regular result frame header
                if check_data:
                    self.check_data(data_list)
        self.tof_app.stopMeasure()
        assert self.is_calibrated() == True, "Device not calibrated"

    def is_calibrated(self):
        """Check the calibration status register to determine if the target is calibrated"""
        cal_status = self.com.i2cTxRx( self.I2C_SLAVE_ADDR, [ self.REG_CALIBRATION_STATUS ], 1 )[0]
        if self.log:
            print(f"#CALSTATUS:{cal_status}")
        return cal_status == 0

    # @pytest.mark.skip(reason="deactivated for now to save time")
    @pytest.mark.parametrize("short_range_mode", [False,True])
    @pytest.mark.parametrize("spad_map", spad_maps_3x3 + spad_maps_4x4)
    def test_execute_legacy(self,short_range_mode, spad_map):
        """Run tests in 3x3 mode and 4x4 mode for all SPAD maps and in short and long range mode"""
        # disable these checks for TMF8821 firmware
        # assert self.tof_app.setLegacyMode() == self.tof_app.Status.OK
        # assert self.tof_app.TMF8828_COM_TMF8828_MODE__mode__TMF8821 == self.tof_app.getAppMode(), "Could not switch to legacy mode (TMF8820/21)"
        if short_range_mode:
            self.short_range_mode_active = True
            self.tof_app.setShortRangeAccuracy()
            assert self.tof_app.getAccuracy() == self.tof_app.TMF8X2X_SHORT_RANGE_ACCURACY, "Could not switch to short range mode"
        else:
            self.short_range_mode_active = False
            self.tof_app.setLongRangeAccuracy()
            assert self.tof_app.getAccuracy() == self.tof_app.TMF8X2X_LONG_RANGE_ACCURACY, "Could not switch to long range mode"
        
        # save SPAD map for result interpretation
        self.active_spad_map = spad_map
        assert self.tof_app.runFactoryCalibration(spad_map_id=spad_map, kilo_iterations=4000, timeout=self.FACTORY_CALIBRATION_TIMEOUT) == self.tof_app.Status.OK, "Factory calibration failed"
        calibration_data = self.tof_app.getFactoryCalibrationData()
        assert calibration_data != self.tof_app.Status.DEV_ERROR, "Factory calibration data invalid"
        if self.log:
            print(f"#CAL:{calibration_data}")
            self.tof_app.dumpFactoryCalibration()
        assert self.tof_app.configure(period_in_ms=66, kilo_iterations=550, spad_map_id=spad_map) == self.tof_app.Status.OK, "Device configuration failed"
        assert self.tof_app.loadFactoryCalibration(calibration_data) == self.tof_app.Status.OK, "Loading calibration data failed"
        self.measure()
        errors = self.tof_app.getAndResetErrors()
        assert len(errors) == 0

    # disable these tests on Jenkins (TM8821 workspace) for TMF8821 firmware
    @pytest.mark.skipif(str(pathlib.Path(__file__).parent.resolve()).find("TMF8821") != -1,reason="skip TMF8828 test for TMF8821 firmware")
    @pytest.mark.parametrize("short_range_mode", [False,True])
    def test_execute_8x8(self,short_range_mode):
        """Run tests in 8x8 mode in short and long range mode"""
        self.tof_app.set8x8Mode()
        assert self.tof_app.TMF8828_COM_TMF8828_MODE__mode__TMF8828 == self.tof_app.getAppMode(), "Could not switch to TMF8828 mode"
        if short_range_mode:
            self.short_range_mode_active = True
            self.tof_app.setShortRangeAccuracy()
            assert self.tof_app.getAccuracy() == self.tof_app.TMF8X2X_SHORT_RANGE_ACCURACY, "Could not switch to short range mode"
        else:
            self.short_range_mode_active = False
            self.tof_app.setLongRangeAccuracy()
            assert self.tof_app.getAccuracy() == self.tof_app.TMF8X2X_LONG_RANGE_ACCURACY, "Could not switch to long range mode"
        self.active_spad_map = Tmf882xApp.TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_15
        assert self.tof_app.runFactoryCalibration(spad_map_id=self.active_spad_map, kilo_iterations=125, timeout=self.FACTORY_CALIBRATION_TIMEOUT) == self.tof_app.Status.OK, "Factory calibration failed"
        calibration_data = self.tof_app.getFactoryCalibrationData()
        assert calibration_data != self.tof_app.Status.DEV_ERROR, "Factory calibration data invalid"
        if self.log:
            print("#CAL:",calibration_data)
            self.tof_app.dumpFactoryCalibration()
        assert self.tof_app.configure(period_in_ms=66, kilo_iterations=125, spad_map_id=self.active_spad_map) == self.tof_app.Status.OK, "Device configuration failed"
        assert self.tof_app.loadFactoryCalibration(calibration_data) == self.tof_app.Status.OK, "Loading calibration data failed"
        self.measure()
        errors = self.tof_app.getAndResetErrors()
        assert len(errors) == 0

    @pytest.mark.parametrize("spad_map_id", [1,7,15])
    @pytest.mark.parametrize("k_iters", [125, 550])
    @pytest.mark.parametrize("period_ms", [1, 20])
    def test_stopStartMeasure(self,spad_map_id, k_iters, period_ms):
        """ [PROJ1128-4277] Fast stop + start a measurement very fast, and check for errors."""

        supports_8x8_measurements = ((self.tof_app.getAppId()[3] & 0x8) == 0x8) # TMF8X2X_COM_BUILD_TYPE.8x8_measurements == 1
        number_stop_starts = 200 # stop+start N times

        if supports_8x8_measurements:
            if spad_map_id == Tmf882xApp.TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_15:
                self.tof_app.set8x8Mode() # Should already be in 8x8 mode, but better be safe than sorry.
            else:
                self.tof_app.setLegacyMode() # Need to be in legacy mode for 3x3/4x4 measurements.
        else:
            if spad_map_id == Tmf882xApp.TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__map_no_15:
                pytest.skip("Do not test custom SPAD maps with 4x4 patch.")

        assert self.tof_app.configure(period_in_ms=period_ms, kilo_iterations=k_iters, spad_map_id=spad_map_id) == self.tof_app.Status.OK, "Device configuration failed"
        for i in range(number_stop_starts):
            self.tof_app.startMeasure() == self.tof_app.Status.OK
            time.sleep(0.003) # wait for a short period to ensure the BDV correction has finished.
            measure_status = self.tof_app.com.i2cTxRx(self.tof_app.I2C_SLAVE_ADDR, [0x5], 1)[0] # read TMF8X2X_COM_MEASURE_STATUS
            assert measure_status == 0x00, "Error in interation {}".format(i)
            self.tof_app.stopMeasure() == self.tof_app.Status.OK
            #time.sleep(i * 0.001)
        errors = self.tof_app.getAndResetErrors()
        assert len(errors) == 0

if __name__ == "__main__":
    # call pytest here, so we can call it directly.
    sys.exit(pytest.main(["-s", "-x", __file__]))
