import os
import pathlib
import pvaccess as pva
import numpy as np
import queue
import time
import threading
import signal
import json
import configparser

import subprocess

from mctoptics import util
from mctoptics import log
from epics import PV
from collections import OrderedDict

data_path = pathlib.Path(__file__).parent / 'data'

EPSILON = 0.1

class MCTOptics():
    """ Class for controlling TXM optics via EPICS

        Parameters
        ----------
        args : dict
            Dictionary of pv variables.
    """

    def __init__(self, pv_files, macros):

        # init pvs
        self.config_pvs = {}
        self.control_pvs = {}
        self.pv_prefixes = {}
        
        if not isinstance(pv_files, list):
            pv_files = [pv_files]
        for pv_file in pv_files:
            self.read_pv_file(pv_file, macros)
        self.show_pvs()

        #Define PVs we will need from the sample x-y-z motors, which is on another IOC

        lens_sample_x_pv_name = self.control_pvs['LensSampleX'].pvname
        self.control_pvs['LensSampleXPosition']      = PV(lens_sample_x_pv_name + '.VAL')
        lens_sample_y_pv_name = self.control_pvs['LensSampleY'].pvname
        self.control_pvs['LensSampleYPosition']      = PV(lens_sample_y_pv_name + '.VAL')
        lens_sample_z_pv_name = self.control_pvs['LensSampleZ'].pvname
        self.control_pvs['LensSampleZPosition']      = PV(lens_sample_z_pv_name + '.VAL')

        camera0_rotation_pv_name = self.control_pvs['Camera0Rotation'].pvname
        self.control_pvs['Camera0RotationPosition']  = PV(camera0_rotation_pv_name + '.VAL')

        camera1_rotation_pv_name = self.control_pvs['Camera1Rotation'].pvname
        self.control_pvs['Camera1RotationPosition']  = PV(camera1_rotation_pv_name + '.VAL')


        lens0_focus_pv_name = self.control_pvs['Lens0Focus'].pvname
        self.control_pvs['Lens0FocusPosition']  = PV(lens0_focus_pv_name + '.VAL')
        lens1_focus_pv_name = self.control_pvs['Lens1Focus'].pvname
        self.control_pvs['Lens1FocusPosition']  = PV(lens1_focus_pv_name + '.VAL')
        lens2_focus_pv_name = self.control_pvs['Lens2Focus'].pvname
        self.control_pvs['Lens2FocusPosition']  = PV(lens2_focus_pv_name + '.VAL')

        #Define PVs from the camera IOC that we will need
        prefix = self.pv_prefixes['Camera0']
        camera_prefix = prefix + 'cam1:'

        self.control_pvs['Cam0AcquireTime']          = PV(camera_prefix + 'AcquireTime')
        self.control_pvs['Cam0ArraySizeXRBV']        = PV(camera_prefix + 'ArraySizeX_RBV')
        self.control_pvs['Cam0ArraySizeYRBV']        = PV(camera_prefix + 'ArraySizeY_RBV')
        self.control_pvs['Cam0Acquire']              = PV(camera_prefix + 'Acquire')
        self.control_pvs['Cam0MaxSizeXRBV']          = PV(camera_prefix + 'MaxSizeX_RBV')
        self.control_pvs['Cam0MaxSizeYRBV']          = PV(camera_prefix + 'MaxSizeY_RBV')
        self.control_pvs['Cam0MinX']                 = PV(camera_prefix + 'MinX')
        self.control_pvs['Cam0MinY']                 = PV(camera_prefix + 'MinY')
        self.control_pvs['Cam0SizeX']                = PV(camera_prefix + 'SizeX')
        self.control_pvs['Cam0SizeY']                = PV(camera_prefix + 'SizeY')
        self.control_pvs['Cam0SizeXRBV']             = PV(camera_prefix + 'SizeX_RBV')
        self.control_pvs['Cam0SizeYRBV']             = PV(camera_prefix + 'SizeY_RBV')
        self.control_pvs['Cam0MinXRBV']              = PV(camera_prefix + 'MinX_RBV')
        self.control_pvs['Cam0MinYRBV']              = PV(camera_prefix + 'MinY_RBV')
        self.control_pvs['Cam0ConvertPixelFormat']   = PV(camera_prefix + 'ConvertPixelFormat')
        self.control_pvs['Cam0PixelFormat']          = PV(camera_prefix + 'PixelFormat')
        self.control_pvs['Cam0GC_AdcBitDepth']       = PV(camera_prefix + 'GC_AdcBitDepth')
        self.control_pvs['Cam0BinX']                 = PV(camera_prefix + 'BinX')
        self.control_pvs['Cam0BinY']                 = PV(camera_prefix + 'BinY')
        self.control_pvs['Cam0BinXRBV']              = PV(camera_prefix + 'BinX_RBV')
        self.control_pvs['Cam0BinYRBV']              = PV(camera_prefix + 'BinY_RBV')

        prefix = self.pv_prefixes['Camera1']
        camera_prefix = prefix + 'cam1:'

        self.control_pvs['Cam1AcquireTime']          = PV(camera_prefix + 'AcquireTime')
        self.control_pvs['Cam1ArraySizeXRBV']        = PV(camera_prefix + 'ArraySizeX_RBV')
        self.control_pvs['Cam1ArraySizeYRBV']        = PV(camera_prefix + 'ArraySizeY_RBV')
        self.control_pvs['Cam1Acquire']              = PV(camera_prefix + 'Acquire')
        self.control_pvs['Cam1MaxSizeXRBV']          = PV(camera_prefix + 'MaxSizeX_RBV')
        self.control_pvs['Cam1MaxSizeYRBV']          = PV(camera_prefix + 'MaxSizeY_RBV')
        self.control_pvs['Cam1MinX']                 = PV(camera_prefix + 'MinX')
        self.control_pvs['Cam1MinY']                 = PV(camera_prefix + 'MinY')
        self.control_pvs['Cam1SizeX']                = PV(camera_prefix + 'SizeX')
        self.control_pvs['Cam1SizeY']                = PV(camera_prefix + 'SizeY')
        self.control_pvs['Cam1SizeXRBV']             = PV(camera_prefix + 'SizeX_RBV')
        self.control_pvs['Cam1SizeYRBV']             = PV(camera_prefix + 'SizeY_RBV')
        self.control_pvs['Cam1MinXRBV']              = PV(camera_prefix + 'MinX_RBV')
        self.control_pvs['Cam1MinYRBV']              = PV(camera_prefix + 'MinY_RBV')
        self.control_pvs['Cam1ConvertPixelFormat']   = PV(camera_prefix + 'ConvertPixelFormat')
        self.control_pvs['Cam1PixelFormat']          = PV(camera_prefix + 'PixelFormat')
        self.control_pvs['Cam1GC_AdcBitDepth']       = PV(camera_prefix + 'GC_AdcBitDepth')
        self.control_pvs['Cam1BinX']                 = PV(camera_prefix + 'BinX')
        self.control_pvs['Cam1BinY']                 = PV(camera_prefix + 'BinY')
        self.control_pvs['Cam1BinXRBV']              = PV(camera_prefix + 'BinX_RBV')
        self.control_pvs['Cam1BinYRBV']              = PV(camera_prefix + 'BinY_RBV')

        prefix = self.pv_prefixes['OverlayPlugin0']
        self.control_pvs['OP0EnableCallbacks'] = PV(prefix + 'EnableCallbacks')
        self.control_pvs['OP01Use']            = PV(prefix + '1:Use')        
        self.control_pvs['OP01CenterX']        = PV(prefix + '1:CenterX')        
        self.control_pvs['OP01CenterY']        = PV(prefix + '1:CenterY')        


        prefix = self.pv_prefixes['OverlayPlugin1']
        self.control_pvs['OP1EnableCallbacks'] = PV(prefix + 'EnableCallbacks')
        self.control_pvs['OP11Use']            = PV(prefix + '1:Use')        
        self.control_pvs['OP11CenterX']        = PV(prefix + '1:CenterX')        
        self.control_pvs['OP11CenterY']        = PV(prefix + '1:CenterY')       

        self.epics_pvs = {**self.config_pvs, **self.control_pvs}

        # Wait 1 second for all PVs to connect
        time.sleep(2)

        ########################### VN
        # shall we run a sync() here?
        self.lens_cur = self.epics_pvs['LensSelect'].get()
        self.camera_cur = self.epics_pvs['CameraSelect'].get()
        ########################### VN
        
        # print(self.epics_pvs)
        for epics_pv in ('LensSelect', 'CameraSelect', 'CrossSelect', 'Sync', 'Cut', 'Camera0Bit', 'Camera1Bit', 'CameraBinning'):
            self.epics_pvs[epics_pv].add_callback(self.pv_callback)
        for epics_pv in ('Sync', 'Cut'):
            self.epics_pvs[epics_pv].put(0)

        # Start the watchdog timer thread
        thread = threading.Thread(target=self.reset_watchdog, args=(), daemon=True)
        thread.start()

        # Synch CameraSelected with CameraSelect
        camera_select = self.epics_pvs['CameraSelect'].get()
        if(camera_select == 0):
            self.epics_pvs['CameraSelected'].put(0)
        elif(camera_select == 1):
            self.epics_pvs['CameraSelected'].put(1)

        # Synch BitSelect with actual camera 0/1 status
        self.sync_bit_select(0)
        self.sync_bit_select(1)

        # Synch binning with actual camera select status
        self.sync_binning_select(str(camera_select))

        self.epics_pvs['MCTStatus'].put('Done')

        log.setup_custom_logger("./mctoptics.log")
        print("./mctoptics.log")

    def sync_binning_select(self, camera_id):

        # if the camera is collecting images stop it
        camera_acquire = 0
        if self.control_pvs['Cam'+camera_id+'Acquire'].get() == 1:
            camera_acquire = 1
            log.info('stopping the camera')
            self.control_pvs['Cam'+camera_id+'Acquire'].put('Done')
            self.wait_pv(self.epics_pvs['Cam'+camera_id+'Acquire'], 0)

        bin_x = self.epics_pvs['Cam'+camera_id+'BinX'].get()
        if bin_x == 1 or bin_x == 2 or bin_x == 4:
            self.epics_pvs['Cam'+camera_id+'BinY'].put(bin_x, wait=True)
            self.epics_pvs['CameraBinning'].put(int(np.log2(bin_x)))
            log.info('mctOptics: Set camera %s binning to %s', camera_id, self.epics_pvs['CameraBinning'].get(as_string=True))

        # if the camera was collecting images start it again
        if camera_acquire == 1:
            time.sleep(1)
            log.info('restarting the camera')
            self.control_pvs['Cam'+camera_id+'Acquire'].put('Acquire')
            self.wait_pv(self.epics_pvs['Cam'+camera_id+'Acquire'], 1)

    def sync_bit_select(self, camera_id):
        # ConvertPixelFormat 2bmSP1:    2bmSP2:
        # ============================================
        # STATE  0:          None        None
        # STATE  1:          Mono8       Mono8
        # STATE  2:          Mono16      Mono16
        # STATE  3:          Raw16       Raw16
        # STATE  4:          RGB8        RGB8
        # STATE  5:          RGB16       RGB16 
        cam_pixel_format = self.control_pvs['Cam'+str(camera_id)+'PixelFormat'].get()
        if cam_pixel_format == 0:
            self.epics_pvs['Camera'+str(camera_id)+'Bit'].put(0)
        elif cam_pixel_format == 1:
            self.epics_pvs['Camera'+str(camera_id)+'Bit'].put(3)
        elif cam_pixel_format == 2:
            self.epics_pvs['Camera'+str(camera_id)+'Bit'].put(1)
        elif cam_pixel_format == 3:
            self.epics_pvs['Camera'+str(camera_id)+'Bit'].put(2)




    def reset_watchdog(self):
        """Sets the watchdog timer to 5 every 3 seconds"""
        while True:
            self.epics_pvs['Watchdog'].put(5)
            time.sleep(3)

        log.setup_custom_logger("./mctoptics.log")

    def read_pv_file(self, pv_file_name, macros):
        """Reads a file containing a list of EPICS PVs to be used by MCTOptics.


        Parameters
        ----------
        pv_file_name : str
          Name of the file to read
        macros: dict
          Dictionary of macro substitution to perform when reading the file
        """

        pv_file = open(pv_file_name)
        lines = pv_file.read()
        pv_file.close()
        lines = lines.splitlines()
        for line in lines:
            is_config_pv = True
            if line.find('#controlPV') != -1:
                line = line.replace('#controlPV', '')
                is_config_pv = False
            line = line.lstrip()
            # Skip lines starting with #
            if line.startswith('#'):
                continue
            # Skip blank lines
            if line == '':
                continue
            pvname = line
            # Do macro substitution on the pvName
            for key in macros:
                pvname = pvname.replace(key, macros[key])
            # Replace macros in dictionary key with nothing
            dictentry = line
            for key in macros:
                dictentry = dictentry.replace(key, '')

            epics_pv = PV(pvname)

            if is_config_pv:
                self.config_pvs[dictentry] = epics_pv
            else:
                self.control_pvs[dictentry] = epics_pv
            # if dictentry.find('PVAPName') != -1:
            #     pvname = epics_pv.value
            #     key = dictentry.replace('PVAPName', '')
            #     self.control_pvs[key] = PV(pvname)
            if dictentry.find('PVName') != -1:
                pvname = epics_pv.value
                key = dictentry.replace('PVName', '')
                if (pvname != ''): 
                    print(pvname, key)
                    self.control_pvs[key] = PV(pvname)
            if dictentry.find('PVPrefix') != -1:
                pvprefix = epics_pv.value
                key = dictentry.replace('PVPrefix', '')
                self.pv_prefixes[key] = pvprefix

    def show_pvs(self):
        """Prints the current values of all EPICS PVs in use.

        The values are printed in three sections:

        - config_pvs : The PVs that are part of the scan configuration and
          are saved by save_configuration()

        - control_pvs : The PVs that are used for EPICS control and status,
          but are not saved by save_configuration()

        - pv_prefixes : The prefixes for PVs that are used for the areaDetector camera,
          file plugin, etc.
        """

        print('configPVS:')
        for config_pv in self.config_pvs:
            print(config_pv, ':', self.config_pvs[config_pv].get(as_string=True))

        print('')
        print('controlPVS:')
        for control_pv in self.control_pvs:
            print(control_pv, ':', self.control_pvs[control_pv].get(as_string=True))

        print('')
        print('pv_prefixes:')
        for pv_prefix in self.pv_prefixes:
            print(pv_prefix, ':', self.pv_prefixes[pv_prefix])

    def pv_callback(self, pvname=None, value=None, char_value=None, **kw):
        """Callback function that is called by pyEpics when certain EPICS PVs are changed
        """

        log.debug('pv_callback pvName=%s, value=%s, char_value=%s', pvname, value, char_value)
        if (pvname.find('LensSelect') != -1) and ((value == 0) or (value == 1) or (value == 2)):
            thread = threading.Thread(target=self.lens_select, args=())
            thread.start()
        elif (pvname.find('CameraSelect') != -1) and ((value == 0) or (value == 1)):
            thread = threading.Thread(target=self.camera_select, args=())
            thread.start()
        elif (pvname.find('CrossSelect') != -1) and ((value == 0) or (value == 1)):
            thread = threading.Thread(target=self.cross_select, args=())
            thread.start()
        elif (pvname.find('Sync') != -1) and (value == 1):
            thread = threading.Thread(target=self.sync, args=())
            thread.start()
        elif (pvname.find('Cut') != -1) and (value ==1):
            thread = threading.Thread(target=self.crop_detector, args=())
            thread.start()     

        elif (pvname.find('Camera0Bit') != -1) and ((value == 0) or (value == 1) or (value == 2) or (value == 3)):
            thread = threading.Thread(target=self.camera_bit, args=(0,))
            thread.start()
        elif (pvname.find('Camera1Bit') != -1) and ((value == 0) or (value == 1) or (value == 2) or (value == 3)):
            thread = threading.Thread(target=self.camera_bit, args=(1,))
            thread.start()
        elif (pvname.find('CameraBinning') != -1) and ((value == 0) or (value == 1) or (value == 2)):
            thread = threading.Thread(target=self.camera_binning, args=())
            thread.start()
    
    def take_lens_offsets(self, lens, cam):
        if lens == 0:
            return 0,0,0
        x = self.epics_pvs['Camera'+str(cam)+'Lens'+str(lens)+'XOffset'].get()
        y = self.epics_pvs['Camera'+str(cam)+'Lens'+str(lens)+'YOffset'].get()
        z = self.epics_pvs['Camera'+str(cam)+'Lens'+str(lens)+'ZOffset'].get()
        return x,y,z

    def take_camera_rotation(self, lens, cam):

        return self.epics_pvs['Camera'+str(cam)+'Lens'+str(lens)+'Rotation'].get()


    def lens_select(self):
        """Callback function that is called by pyEpics when the Optique Peter lens select button is pressed.
        
        The callback handles the following tasks:

        - Store the current camera rotation position for the current lens in the corresponding CameraXLenxYRotation PV.

        - Take offsets of the current lens.

        - Take new lens id and its offsets.

        - Take camera id and its rotation for the new lens.

        - Move sample X-Y-Z motors with respect to relative offsets.

        - Set the correct camera rotation for the lens in use.

        - Move to the new selected lens.

        - Update detector magnification and tube lens PVs using the data stored in the lens.json file.

        - Update scintillator information using the data stored in the scintillator.json file.

        """
        if self.epics_pvs['MCTStatus'].get(as_string=True) != 'Done':
            self.epics_pvs['LensSelect'].put(self.lens_cur)
            return
        
        # Store the current camera rotation position for the current lens in the corresponding CameraXLenxYRotation PV
        rotation_cur = self.control_pvs['Camera'+str(self.camera_cur)+'RotationPosition'].get()
        self.epics_pvs['Camera'+str(self.camera_cur)+'Lens'+str(self.lens_cur)+'Rotation'].put(rotation_cur)

        # Take offsets of the current lens
        x_cur,y_cur,z_cur = self.take_lens_offsets(self.lens_cur, self.camera_cur)            
        # Take new lens id and its offsets
        lens_select = self.epics_pvs['LensSelect'].get()
        x_select,y_select,z_select = self.take_lens_offsets(lens_select, self.camera_cur)
        # Take camera id and its rotation for the new lens
        camera_rotation = self.take_camera_rotation(lens_select, self.camera_cur)        

        # Update current lens
        self.lens_cur = lens_select

        # Move sample X-Y-Z motors with respect to relative offsets       
        x = self.epics_pvs['LensSampleXPosition'].get()
        y = self.epics_pvs['LensSampleYPosition'].get()
        z = self.epics_pvs['LensSampleZPosition'].get()
        x_new = x + x_select - x_cur
        y_new = y + y_select - y_cur
        z_new = z + z_select - z_cur
        log.info('move X from %f to %f', x, x_new)
        log.info('move Y from %f to %f', y, y_new)
        log.info('move Z from %f to %f', z, z_new)
        log.info('move camera %s rotation to %f' %(self.camera_cur, camera_rotation))        
        self.epics_pvs['LensSampleXPosition'].put(x_new)
        self.epics_pvs['LensSampleYPosition'].put(y_new)
        self.epics_pvs['LensSampleZPosition'].put(z_new)
        # Set the correct camera rotation for the lens in use.
        self.epics_pvs['Camera'+str(self.camera_cur)+'RotationPosition'].put(camera_rotation) # no wait, assuming the lens movement is the slowest part

        lens_pos0 = self.epics_pvs['Camera'+str(self.camera_cur)+'LensPos0'].get()
        lens_pos1 = self.epics_pvs['Camera'+str(self.camera_cur)+'LensPos1'].get()
        lens_pos2 = self.epics_pvs['Camera'+str(self.camera_cur)+'LensPos2'].get()

        lens_select = self.epics_pvs['LensSelect'].get()
        lens_name = 'None'

        log.info('Changing Optique Peter lens')
        lens_positions = [lens_pos0, lens_pos1, lens_pos2]

        lens_index = self.epics_pvs['LensSelect'].get()
        lens_name  = self.epics_pvs['Lens' + str(lens_index) + 'Name'].get()
        self.epics_pvs['MCTStatus'].put('Moving to lens: ' + lens_name)
        # Move to the new selected lens
        self.epics_pvs['LensMotor'].put(lens_positions[lens_index], wait=True, timeout=120)
        message = 'Lens selected: ' + lens_name
        log.info(message)
        self.epics_pvs['MCTStatus'].put(message)

        # Update detector magnification and tube lens PVs using the data stored in the lens.json file
        lens_name = lens_name.upper().replace("X", "")
        with open(os.path.join(data_path, 'lens.json')) as json_file:
            lens_lookup = json.load(json_file)
        
        try:

            magnification          = str(lens_lookup[lens_name]['magnification'])
            tube_lens              = lens_lookup[lens_name]['tube_lens']

            # update tomoScan PVs
            self.epics_pvs['CameraObjective'].put(magnification)
            self.epics_pvs['CameraTubeLength'].put(tube_lens)

            detector_pixel_size    = self.epics_pvs['DetectorPixelSize'].get()

            binning = self.epics_pvs['CameraBinning'].get()
            log.info('mctOptics: camera %s binning is set to %d', str(self.camera_cur), pow(2,binning))
            image_pixel_size = float(detector_pixel_size)/float(magnification)*pow(2,binning)
            self.epics_pvs['ImagePixelSize'].put(image_pixel_size)
        except KeyError as e:
            log.error('Lens called %s is not defined. Please add it to the ./data/lens.json file' % e)
            log.error('Failed to update: Camera objective')
            log.error('Failed to update: Camera tube length')
            log.error('Failed to update: Image pixel size')

        # Update scintillator information using the data stored in the scintillator.json file
        with open(os.path.join(data_path, 'scintillator.json')) as json_file:
            scintillator_lookup = json.load(json_file)

        try:           
            scintillator_type      = scintillator_lookup[str(lens_index)]['scintillator_type']
            scintillator_thickness = scintillator_lookup[str(lens_index)]['scintillator_thickness']
            self.epics_pvs['ScintillatorType'].put(scintillator_type)
            self.epics_pvs['ScintillatorThickness'].put(scintillator_thickness)
        except KeyError as e:
            log.error('Scintillator called %s is not defined. Please add it to the ./data/scintillator.json file' % e)
            log.error('Failed to update: Scintillator type')
            log.error('Failed to update: Scintillator thickness')

        self.epics_pvs['MCTStatus'].put('Done')
        
    def camera_select(self):
        """Callback function that is called by pyEpics when the Optique Peter camera select button is pressed.
        
        The callback handles the following tasks:

        - Store the previous camera rotation position for the current lens in the corresponding CameraXLenxYRotation PV.

        - Move the camera selector motor.

        - Stop acquistion of the camera not in use.

        - Store the previous camera (X) focus motor positions of all 3 objectives in the corresponding CameraXLensYFocus PVs.
          This is needed in case there was some adjustiment.

        - Restore the new camera (Y) focus positions store in CameraXLensYFocus to all 3 objectives.

        - Run lens_select() so the Sample X-Y-Z are updated to the new camera-lens combination.

        - Set the correct camera rotation for the lens in use.

        - Update detector pixel size, magnification and image pixel size PVs using the data stored in the camera.json file.

        """
        if self.epics_pvs['MCTStatus'].get(as_string=True) != 'Done':
            self.epics_pvs['CameraSelect'].put(self.camera_cur)
            return
            
        self.epics_pvs['MCTStatus'].put('Changing Optique Peter camera')
        # store the current camera rotation position for the current lens in the CameraXLenxYRotation PV
        rotation_cur = self.control_pvs['Camera'+str(self.camera_cur)+'RotationPosition'].get()
        self.epics_pvs['Camera'+str(self.camera_cur)+'Lens'+str(self.lens_cur)+'Rotation'].put(rotation_cur)

        log.info('Changing Optique Peter camera')
        self.epics_pvs['MCTStatus'].put('Changing Optique Peter camera')
        self.epics_pvs['CameraSelected'].put(2)

        camera_select = self.epics_pvs['CameraSelect'].get()
        camera_name = 'None'
        self.camera_cur = camera_select
        self.update_suggested_scan_param()

        if(camera_select == 0):
            camera_name = self.epics_pvs['Camera0Name'].get()
            self.epics_pvs['MCTStatus'].put('Camera selected: 0')
            camera_pos0 = self.epics_pvs['CameraPos0'].get()
            # move the camera selector motor
            self.epics_pvs['CameraMotor'].put(camera_pos0, wait=True, timeout=120)
            self.epics_pvs['CameraSelected'].put(0)
            # stop acquistion of the camera not in use
            self.epics_pvs['Cam1Acquire'].put(0) 
            # Store the previous camera (X) focus motor positions of all 3 objectives in the corresponding CameraXLensYFocus PVs.
            # This is needed in case there was some adjustiment.
            lens0_focus_pos = self.epics_pvs['Lens0FocusPosition'].get()
            lens1_focus_pos = self.epics_pvs['Lens1FocusPosition'].get()
            lens2_focus_pos = self.epics_pvs['Lens2FocusPosition'].get()
            self.epics_pvs['Camera1Lens0Focus'].put(lens0_focus_pos, wait=True)
            self.epics_pvs['Camera1Lens1Focus'].put(lens1_focus_pos, wait=True)
            self.epics_pvs['Camera1Lens2Focus'].put(lens2_focus_pos, wait=True)
                    
        elif(camera_select == 1):
            camera_name = self.epics_pvs['Camera1Name'].get()
            self.epics_pvs['MCTStatus'].put('Camera selected: 1')
            camera_pos1 = self.epics_pvs['CameraPos1'].get()
            # move the Camera selector motor
            self.epics_pvs['CameraMotor'].put(camera_pos1, wait=True, timeout=120)
            self.epics_pvs['CameraSelected'].put(1)
            # stop acquistion of the camera not in use
            self.epics_pvs['Cam0Acquire'].put(0) 
            # Store the previous camera (X) focus motor positions of all 3 objectives in the corresponding CameraXLensYFocus PVs.
            # This is needed in case there was some adjustiment.
            lens0_focus_pos = self.epics_pvs['Lens0FocusPosition'].get()
            lens1_focus_pos = self.epics_pvs['Lens1FocusPosition'].get()
            lens2_focus_pos = self.epics_pvs['Lens2FocusPosition'].get()
            self.epics_pvs['Camera0Lens0Focus'].put(lens0_focus_pos, wait=True)
            self.epics_pvs['Camera0Lens1Focus'].put(lens1_focus_pos, wait=True)
            self.epics_pvs['Camera0Lens2Focus'].put(lens2_focus_pos, wait=True)

        
        # Take the stored focus position of the selected camera and move the focus position for all 3 lenses
        lens0_focus_pos = self.epics_pvs['Camera'+str(camera_select)+'Lens0Focus'].get()
        lens1_focus_pos = self.epics_pvs['Camera'+str(camera_select)+'Lens1Focus'].get()
        lens2_focus_pos = self.epics_pvs['Camera'+str(camera_select)+'Lens2Focus'].get()
        self.epics_pvs['Lens0FocusPosition'].put(lens0_focus_pos, wait=True) 
        self.epics_pvs['Lens1FocusPosition'].put(lens1_focus_pos, wait=True) 
        self.epics_pvs['Lens2FocusPosition'].put(lens2_focus_pos, wait=True) 

        # Run lens_select() so the Sample X-Y-Z and the lens rotation are updated to the new camera-lens combination
        self.lens_select()

        # Set the correct camera rotation for the lens in use
        camera_rotation = self.take_camera_rotation(self.lens_cur, camera_select)        
        self.control_pvs['Camera'+str(camera_select)+'RotationPosition'].put(camera_rotation, wait=True)

        # Synch binning with actual camera select status
        self.sync_binning_select(str(camera_select))

        log.info('Camera: %s selected', camera_name)

        # Update detector pixel size, magnification and image pixel size PVs using the data stored in the camera.json file
        camera_name = camera_name.upper()
        with open(os.path.join(data_path, 'camera.json')) as json_file:
            camera_lookup = json.load(json_file)

        try:
            detector_pixel_size = camera_lookup[camera_name]['detector_pixel_size']
            # update tomoScan PVs
            self.epics_pvs['DetectorPixelSize'].put(detector_pixel_size)

            magnification = self.epics_pvs['CameraObjective'].get()
            magnification = magnification.upper().replace("X", "") # just in case there was a manual entry ...
            binning = self.epics_pvs['CameraBinning'].get()
            log.info('mctOptics: camera %s binning is set to %d', str(camera_select), pow(2,binning))
            image_pixel_size = float(detector_pixel_size)/float(magnification)*pow(2,binning)
            self.epics_pvs['ImagePixelSize'].put(image_pixel_size)
        except KeyError as e:
            log.error('Camera called %s is not defined. Please add it to the ./data/camera.json file' % e)
            log.error('Failed to update: Detector pixel size')
            log.error('Failed to update: Image pixel size')
    
        self.epics_pvs['MCTStatus'].put('Done')

    def cross_select(self):
        """Plot the cross in imageJ.
        """

        if(self.epics_pvs['CameraSelect'].get() == 0):
            if (self.epics_pvs['CrossSelect'].get() == 0):
                sizex = int(self.epics_pvs['Cam0ArraySizeXRBV'].get())
                sizey = int(self.epics_pvs['Cam0ArraySizeYRBV'].get())
                self.epics_pvs['OP01CenterX'].put(sizex//2)
                self.epics_pvs['OP01CenterY'].put(sizey//2)
                self.control_pvs['OP01Use'].put(1)
                log.info('Cross on camera 0 at %d %d is enable' % (sizex//2,sizey//2))
            else:
                self.control_pvs['OP01Use'].put(0)
                log.info('Cross on camera 0 is disabled')
        elif(self.epics_pvs['CameraSelect'].get() == 1):
            if (self.epics_pvs['CrossSelect'].get() == 0):
                sizex = int(self.epics_pvs['Cam1ArraySizeXRBV'].get())
                sizey = int(self.epics_pvs['Cam1ArraySizeYRBV'].get())
                self.epics_pvs['OP11CenterX'].put(sizex//2)
                self.epics_pvs['OP11CenterY'].put(sizey//2)
                self.control_pvs['OP11Use'].put(1)
                log.info('Cross on camera 1 at %d %d is enable' % (sizex//2,sizey//2))
            else:
                self.control_pvs['OP11Use'].put(0)
                log.info('Cross on camera 1 is disabled')

    def sync(self):
        """
        sync the optique peter selector with the actual motor positon
        """

        self.epics_pvs['MCTStatus'].put('Sync in progress')

        log.info('Sync camera')
        camera_pos0 = self.epics_pvs['CameraPos0'].get()
        camera_pos1 = self.epics_pvs['CameraPos1'].get()
        camera_motor_position = self.epics_pvs['CameraMotor'].get()

        is_camera_pos0 = np.isclose(camera_motor_position, camera_pos0, atol=1e-01)
        is_camera_pos1 = np.isclose(camera_motor_position, camera_pos1, atol=1e-01)

        log.info('Camera motor positon %s:',  camera_motor_position)
        log.info('Pos 0 %s: %s',  camera_pos0, is_camera_pos0)
        log.info('Pos 1 %s: %s',  camera_pos1, is_camera_pos1)

        if is_camera_pos0:
            camera_select_sync = 0
        elif is_camera_pos1:
            camera_select_sync = 1
        else:
            camera_select_sync = -1
            log.error('Sync camera failed: check camera select motor position')

        camera_select = self.epics_pvs['CameraSelect'].get()
        if camera_select != camera_select_sync:
            self.epics_pvs['CameraSelect'].put(camera_select_sync)
            log.warning('Sync camera: done')
        else:
            log.warning('Sync camera: not required, selector is already in the correct position')

        log.info('Sync lens')
        lens_pos0 = self.epics_pvs['Camera0LensPos0'].get()
        lens_pos1 = self.epics_pvs['Camera0LensPos1'].get()
        lens_pos2 = self.epics_pvs['Camera0LensPos2'].get()
        lens_motor_position = self.epics_pvs['LensMotor'].get()

        is_lens_pos0 = np.isclose(lens_motor_position, lens_pos0, atol=1e-01)
        is_lens_pos1 = np.isclose(lens_motor_position, lens_pos1, atol=1e-01)
        is_lens_pos2 = np.isclose(lens_motor_position, lens_pos2, atol=1e-01)

        log.info('Lens motor positon %s:',  lens_motor_position)
        log.info('Lens 0 %s: %s',  lens_pos0, is_lens_pos0)
        log.info('Lens 1 %s: %s',  lens_pos1, is_lens_pos1)
        log.info('Lens 2 %s: %s',  lens_pos2, is_lens_pos2)

        if is_lens_pos0:
            lens_select_sync = 0
        elif is_lens_pos1:
            lens_select_sync = 1
        elif is_lens_pos2:
            lens_select_sync = 2
        else:
            lens_select_sync = -1
            log.error('Sync lens failed: check lens select motor position')

        lens_select = self.epics_pvs['LensSelect'].get()
        if ((lens_select != lens_select_sync) and (lens_select_sync != -1)):
            self.epics_pvs['LensSelect'].put(lens_select_sync)
            log.warning('Sync lens: done')
        else:
            log.warning('Sync lens: not required, selector is already in the correct position')

        if((camera_select_sync == -1) or (lens_select_sync == -1)):
            self.epics_pvs['MCTStatus'].put('Sync error: check log for details')
        else:
            self.epics_pvs['MCTStatus'].put('Done')

        self.epics_pvs['Sync'].put('Done')

    def crop_detector(self):
        """crop detector sizes"""


        camera_select = str(self.epics_pvs['CameraSelect'].get())
        state = self.epics_pvs['Cam'+camera_select+'Acquire'].get()
        self.epics_pvs['Cam'+camera_select+'Acquire'].put(0,wait=True)

        maxsizex = self.epics_pvs['Cam'+camera_select+'MaxSizeXRBV'].get()
        self.epics_pvs['Cam'+camera_select+'MinX'].put(0,wait=True)        
        
        maxsizey = self.epics_pvs['Cam'+camera_select+'MaxSizeYRBV'].get()
        self.epics_pvs['Cam'+camera_select+'MinY'].put(0,wait=True)        
        
        left = self.epics_pvs['CutLeft'].get()
        top = self.epics_pvs['CutTop'].get()
        
        right = self.epics_pvs['CutRight'].get()        
        self.epics_pvs['Cam'+camera_select+'SizeX'].put(maxsizex-left-right,wait=True)
        sizex = self.epics_pvs['Cam'+camera_select+'SizeXRBV'].get()
        right = maxsizex - left - sizex
        self.epics_pvs['CutRight'].put(right,wait=True)

        bottom = self.epics_pvs['CutBottom'].get()
        self.epics_pvs['Cam'+camera_select+'SizeY'].put(maxsizey-top-bottom,wait=True)
        sizey = self.epics_pvs['Cam'+camera_select+'SizeYRBV'].get()
        bottom = maxsizey - top - sizey
        self.epics_pvs['CutBottom'].put(bottom,wait=True)

        self.epics_pvs['Cam'+camera_select+'MinX'].put(left,wait=True)        
        left = self.epics_pvs['Cam'+camera_select+'MinXRBV'].get()
        self.epics_pvs['CutLeft'].put(left,wait=True)

        self.epics_pvs['Cam'+camera_select+'MinY'].put(top,wait=True)        
        top = self.epics_pvs['Cam'+camera_select+'MinYRBV'].get()
        self.epics_pvs['CutTop'].put(top,wait=True)                
    

        self.epics_pvs['Cam'+camera_select+'Acquire'].put(state)  
        self.cross_select()      
        self.epics_pvs['Cut'].put(0,wait=True)

        self.update_suggested_scan_param()

    def update_suggested_scan_param(self):
        camera_select = str(self.epics_pvs['CameraSelect'].get())
        image_size_x = self.epics_pvs['Cam'+camera_select+'ArraySizeXRBV'].get()
        if image_size_x > 0:
            suggested_angles = 1500.0 / 2448.0 * image_size_x
            suggested_angle_step = 180.0 / suggested_angles
            self.epics_pvs['SuggestedAngles'].put(suggested_angles)
            self.epics_pvs['SuggestedAngleStep'].put(suggested_angle_step)
        else:
            log.error("mctOptics: Suggested scan parameters failed to update: Check camera: %s" % str(camera_select))

    def camera_bit(self, camera_id):
        
        bit_selected = self.epics_pvs['Camera'+str(camera_id)+'Bit'].get()
        log.info("mctOptics: Set camera %s to %s" % (str(camera_id), self.epics_pvs['Camera'+str(camera_id)+'Bit'].get(as_string=True)))
        
        # $(P)$(R)CameraXBit
        # ======================================
        # STATE  0: 8-bit")
        # STATE  1: 10-bit")
        # STATE  2: 12-bit")
        # STATE  3: 16-bit")

        # GC_AdcBitDepth 2bmSP1: 2bmSP2:
        # ================================
        # STATE  0:      Bit8     Bit8
        # STATE  1:      Bit10    Bit10
        # STATE  2:      Bit12    Bit12

        # PixelFormat 2bmSP1:     2bmSP2:
        # ======================================
        # STATE  0:   Mono8        Mono8  
        # STATE  1:   Mono16       Mono16 
        # STATE  2:   Mono12Packed Mono10Packed 
        # STATE  3:   Mono12p      Mono12Packed 
        # STATE  4:     N/A        Mono10p
        # STATE  5:     N/A        Mono12p

        # ConvertPixelFormat 2bmSP1:    2bmSP2:
        # ============================================
        # STATE  0:          None        None
        # STATE  1:          Mono8       Mono8
        # STATE  2:          Mono16      Mono16
        # STATE  3:          Raw16       Raw16
        # STATE  4:          RGB8        RGB8
        # STATE  5:          RGB16       RGB16 

        # if the cameara IOC is down, do nothing
        if (self.control_pvs['Cam'+str(camera_id)+'GC_AdcBitDepth'].info == None):
            log.error("mctOptics: Camera %s IOC is down", str(camera_id))
            return
        # if the camera is collecting images stop it
        camera_acquire = 0
        if self.control_pvs['Cam'+str(camera_id)+'Acquire'].get() == 1:
            camera_acquire = 1
            log.info('stopping the camera')
            self.control_pvs['Cam'+str(camera_id)+'Acquire'].put('Done')
            self.wait_pv(self.epics_pvs['Cam'+str(camera_id)+'Acquire'], 0)
        if (bit_selected == 0):
            self.control_pvs['Cam'+str(camera_id)+'GC_AdcBitDepth'].put(0, wait=True)
            self.control_pvs['Cam'+str(camera_id)+'PixelFormat'].put(0, wait=True)
            self.control_pvs['Cam'+str(camera_id)+'ConvertPixelFormat'].put(1, wait=True)
        elif (bit_selected == 1):
            self.control_pvs['Cam'+str(camera_id)+'GC_AdcBitDepth'].put(1, wait=True)
            self.control_pvs['Cam'+str(camera_id)+'PixelFormat'].put(2, wait=True)
            self.control_pvs['Cam'+str(camera_id)+'ConvertPixelFormat'].put(2, wait=True)
        elif (bit_selected == 2):
            self.control_pvs['Cam'+str(camera_id)+'GC_AdcBitDepth'].put(2, wait=True)
            if camera_id == 1:
                self.control_pvs['Cam'+str(camera_id)+'PixelFormat'].put(3, wait=True)
            else:
                self.control_pvs['Cam'+str(camera_id)+'PixelFormat'].put(2, wait=True)
            self.control_pvs['Cam'+str(camera_id)+'ConvertPixelFormat'].put(2, wait=True)
        elif (bit_selected == 3):
            self.control_pvs['Cam'+str(camera_id)+'GC_AdcBitDepth'].put(2, wait=True)
            self.control_pvs['Cam'+str(camera_id)+'PixelFormat'].put(1, wait=True)
            self.control_pvs['Cam'+str(camera_id)+'ConvertPixelFormat'].put(2, wait=True)
        # if the camera was collecting images start it again
        if camera_acquire == 1:
            time.sleep(1)
            log.info('restarting the camera')
            self.control_pvs['Cam'+str(camera_id)+'Acquire'].put('Acquire', wait=True)

    def camera_binning(self):
        """camera binning"""

        camera_select = str(self.epics_pvs['CameraSelect'].get())
        # if the camera is collecting images stop it
        camera_acquire = 0
        if self.control_pvs['Cam'+camera_select+'Acquire'].get() == 1:
            camera_acquire = 1
            log.info('stopping the camera')
            self.control_pvs['Cam'+camera_select+'Acquire'].put('Done')
            self.wait_pv(self.epics_pvs['Cam'+camera_select+'Acquire'], 0)

        binning = self.epics_pvs['CameraBinning'].get()
        log.info('mctOptics: Set camera %s binning to %d', camera_select, pow(2,binning))

        self.epics_pvs['Cam'+camera_select+'BinX'].put(pow(2,binning),wait=True)
        self.epics_pvs['Cam'+camera_select+'BinY'].put(pow(2,binning),wait=True)    

        # if the camera was collecting images start it again
        if camera_acquire == 1:
            time.sleep(1)
            log.info('restarting the camera')
            self.control_pvs['Cam'+str(camera_select)+'Acquire'].put('Acquire')
            self.wait_pv(self.epics_pvs['Cam'+camera_select+'Acquire'], 1)

        detector_pixel_size    = self.epics_pvs['DetectorPixelSize'].get()
        log.info('binning correction of the image pixel size')
        magnification = self.epics_pvs['CameraObjective'].get()
        magnification = magnification.upper().replace("X", "") # just in case there was a manual entry ...
        binning = self.epics_pvs['CameraBinning'].get()
        log.info('mctOptics: camera %s binning is set to %d', str(self.camera_cur), pow(2,binning))
        image_pixel_size = float(detector_pixel_size)/float(magnification)*pow(2,binning)
        self.epics_pvs['ImagePixelSize'].put(image_pixel_size)

    def wait_pv(self, epics_pv, wait_val, timeout=-1):
        """Wait on a pv to be a value until max_timeout (default forever)
           delay for pv to change
        """

        time.sleep(.01)
        start_time = time.time()
        while True:
            pv_val = epics_pv.get()
            if isinstance(pv_val, float):
                if abs(pv_val - wait_val) < EPSILON:
                    return True
            if pv_val != wait_val:
                if timeout > -1:
                    current_time = time.time()
                    diff_time = current_time - start_time
                    if diff_time >= timeout:
                        log.error('  *** ERROR: DROPPED IMAGES ***')
                        log.error('  *** wait_pv(%s, %d, %5.2f reached max timeout. Return False',
                                      epics_pv.pvname, wait_val, timeout)
                        return False
                time.sleep(.01)
            else:
                return True
