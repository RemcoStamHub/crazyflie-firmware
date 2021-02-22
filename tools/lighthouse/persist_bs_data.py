#!/usr/bin/env python3

#  ,---------,       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Crazyflie control firmware
#
#  Copyright (C) 2020 Bitcraze AB
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, in version 3.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program. If not, see <http://www.gnu.org/licenses/>.
#
#
#  Persist geometry and calibration data in the Crazyflie storage.
#
#  This script uploads geometry and calibration data to a crazyflie and
#  writes the data to persistant memory to make it available after
#  re-boot.
#
#  This script is a temporary solution until there is support
#  in the client.


import logging
import time

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.mem import LighthouseBsCalibration
from cflib.crazyflie.mem import LighthouseBsGeometry
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

uri = 'radio://0/100/2M/E7E7E7E703'


class WriteMem:
    def __init__(self, uri, geos, calibs):
        self.data_written = False
        self.result_received = False

        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            mems = scf.cf.mem.get_mems(MemoryElement.TYPE_LH)

            count = len(mems)
            if count != 1:
                raise Exception('Unexpected nr of memories found:', count)

            lh_mem = mems[0]

            for bs, geo in geos.items():
                self.data_written = False
                print('Write geoetry', bs, 'to RAM')
                lh_mem.write_geo_data(bs, geo, self._data_written, write_failed_cb=self._data_failed)

                while not self.data_written:
                    time.sleep(0.1)

            for bs, calib in calibs.items():
                self.data_written = False
                print('Write calibration', bs, 'to RAM')
                lh_mem.write_calib_data(bs, calib, self._data_written, write_failed_cb=self._data_failed)

                while not self.data_written:
                    time.sleep(0.1)

            print('Persist data')
            scf.cf.loc.receivedLocationPacket.add_callback(self._data_persisted)
            scf.cf.loc.send_lh_persist_data_packet(list(range(16)), list(range(16)))

            while not self.result_received:
                time.sleep(0.1)


    def _data_written(self, mem, addr):
        self.data_written = True

    def _data_failed(self, mem, addr):
        raise Exception('Write to RAM failed')

    def _data_persisted(self, data):
        if (data.data):
            print('Data persisted')
        else:
            raise Exception("Write to storage failed")

        self.result_received = True


geo0 = LighthouseBsGeometry()
geo0.origin = [2.73545634,0.59442165,2.43717962]
geo0.rotation_matrix = [[-0.7463173 , 0.0977276 ,-0.65837664], [-0.14741476,-0.988866  , 0.02032072], [-0.64906038, 0.11222014, 0.75241429], ]
geo0.valid = True

geo1 = LighthouseBsGeometry()
geo1.origin = [-1.91572345,-0.22066576, 2.37044577]
geo1.rotation_matrix = [[ 0.73640365,-0.12335235, 0.66520212], [ 0.17914481, 0.9836941 ,-0.01590782], [-0.65239313, 0.13088209, 0.74649386], ]
geo1.valid = True

calib0 = LighthouseBsCalibration()
calib0.sweeps[0].tilt = -0.04730224609375
calib0.sweeps[0].phase = 0.0
calib0.sweeps[0].curve = 0.08477783203125
calib0.sweeps[0].gibphase = 2.4375
calib0.sweeps[0].gibmag = -0.00467681884765625
calib0.sweeps[0].ogeephase = 0.37548828125
calib0.sweeps[0].ogeemag = -0.2259521484375
calib0.sweeps[1].tilt = 0.04510498046875
calib0.sweeps[1].phase = -0.006591796875
calib0.sweeps[1].curve = 0.342529296875
calib0.sweeps[1].gibphase = 0.049346923828125
calib0.sweeps[1].gibmag = 0.0035724639892578125
calib0.sweeps[1].ogeephase = 0.96875
calib0.sweeps[1].ogeemag = -0.324951171875
calib0.valid = True

calib1 = LighthouseBsCalibration()
calib1.sweeps[0].tilt = -0.053497314453125
calib1.sweeps[0].phase = 0.0
calib1.sweeps[0].curve = 0.16259765625
calib1.sweeps[0].gibphase = 2.650390625
calib1.sweeps[0].gibmag = -0.004192352294921875
calib1.sweeps[0].ogeephase = 0.52587890625
calib1.sweeps[0].ogeemag = -0.343017578125
calib1.sweeps[1].tilt = 0.043487548828125
calib1.sweeps[1].phase = -0.007122039794921875
calib1.sweeps[1].curve = 0.2548828125
calib1.sweeps[1].gibphase = 0.315185546875
calib1.sweeps[1].gibmag = 0.0024089813232421875
calib1.sweeps[1].ogeephase = 1.216796875
calib1.sweeps[1].ogeemag = -0.33935546875
calib1.valid = True


logging.basicConfig(level=logging.ERROR)
cflib.crtp.init_drivers(enable_debug_driver=False)

WriteMem(uri,
    {
        0: geo0,
        1: geo1,
    },
    {
        0: calib0,
        1: calib1,
    })
