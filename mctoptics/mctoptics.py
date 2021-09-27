import pvaccess as pva
import numpy as np
import queue
import time
import h5py
import threading
import signal

from mctoptics import util
from mctoptics import log
from epics import PV


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

        prefix = self.pv_prefixes['TomoScan']
        self.control_pvs['CameraObjective']   = PV(prefix + 'CameraObjective')
        self.control_pvs['DetectorPixelSize'] = PV(prefix + 'DetectorPixelSize')
                 
        self.epics_pvs = {**self.config_pvs, **self.control_pvs}

        # print(self.epics_pvs)
        for epics_pv in ('LensSelect', 'CameraSelect', "ShutterSelect"):
            self.epics_pvs[epics_pv].add_callback(self.pv_callback)

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
        elif (pvname.find('ShutterSelect') != -1) and ((value == 0) or (value == 1)):
            thread = threading.Thread(target=self.shutter_select, args=())
            thread.start()

    def lens_select(self):
        """Moves the Optique Peter lens.
        """

        print(self.epics_pvs['CameraObjective'])
        if (self.epics_pvs['LensLock'].value == 1):
            lens_pos0 = self.epics_pvs['LensPos0'].value
            lens_pos1 = self.epics_pvs['LensPos1'].value
            lens_pos2 = self.epics_pvs['LensPos2'].value

            lens_select = self.epics_pvs['LensSelect'].value
            lens_name = 'None'

            log.info('Changing Optique Peter lens')

            if(self.epics_pvs['LensSelect'].value == 0):
                lens_name = self.epics_pvs['LensName0'].value
                self.epics_pvs['LensMotor'].put(lens_pos0, wait=True)
            elif(self.epics_pvs['LensSelect'].value == 1):
                lens_name = self.epics_pvs['LensName1'].value
                self.epics_pvs['LensMotor'].put(lens_pos1, wait=True)
            elif(self.epics_pvs['LensSelect'].value == 2):
                lens_name = self.epics_pvs['LensName2'].value
                self.epics_pvs['LensMotor'].put(lens_pos2, wait=True)
            log.info('Lens: %s selected', lens_name)
            self.epics_pvs['CameraObjective'].put(lens_name)
        else:
            log.error('Changing Optique Peter lens: Locked')

    def camera_select(self):
        """Moves the Optique Peter camera.
        """
        
        if (self.epics_pvs['CameraLock'].value == 1):
            camera_pos0 = self.epics_pvs['CameraPos0'].value
            camera_pos1 = self.epics_pvs['CameraPos1'].value

            camera_select = self.epics_pvs['CameraSelect'].value
            camera_name = 'None'

            log.info('Changing Optique Peter camera')

            if(self.epics_pvs['CameraSelect'].value == 0):
                camera_name = self.epics_pvs['CameraName0'].value
                self.epics_pvs['CameraMotor'].put(camera_pos0, wait=True)
            elif(self.epics_pvs['CameraSelect'].value == 1):
                camera_name = self.epics_pvs['CameraName1'].value
                self.epics_pvs['CameraMotor'].put(camera_pos1, wait=True)

            camera_pixel_size = -2
            if camera_name == 'Adimec':
                camera_pixel_size = 5.5
            elif camera_name == 'Flir':
                camera_pixel_size = 3.45

            self.control_pvs['DetectorPixelSize'].put(camera_pixel_size)
            log.info('Camera: %s selected', camera_name)

        else:
            log.error('Changing Optique Peter camera: Locked')

    def shutter_select(self):
        """Moves the Optique Peter shutter.
        """

        if (self.epics_pvs['ShutterLock'].value == 1):
            shutter_pos0 = self.epics_pvs['ShutterPos0'].value
            shutter_pos1 = self.epics_pvs['ShutterPos1'].value

            shutter_select = self.epics_pvs['ShutterSelect'].value
            shutter_name = 'None'

            log.info('Fast shutter')

            if(self.epics_pvs['ShutterSelect'].value == 0):
                shutter_name = self.epics_pvs['ShutterName0'].value
                self.epics_pvs['ShutterMotor'].put(shutter_pos0, wait=True)
            elif(self.epics_pvs['ShutterSelect'].value == 1):
                shutter_name = self.epics_pvs['ShutterName1'].value
                self.epics_pvs['ShutterMotor'].put(shutter_pos1, wait=True)

            log.info('Shutter: %s', shutter_name)
        else:
            log.error('Shutter: Locked')