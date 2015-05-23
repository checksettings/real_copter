#!/usr/bin/env python

# basics
import rospy
import tf

from ADXL345 import ADXL345
from L3GD20 import L3GD20
from HMC5883L import HMC5883L

from math import sqrt
from math import sin
from math import cos
from math import atan
from math import atan2
from math import pi

if __name__ == "__main__":
    rospy.init_node('copter', anonymous=True)

    rate = rospy.Rate(50.0)

    try:
        adxl345 = ADXL345()
        l3gd20 = L3GD20(busId=1, slaveAddr=0x6b, scale="500dps")
        gyro = dict()
        gyro['x'] = 0.0
        gyro['y'] = 0.0
        gyro['z'] = 0.0

        hmc5883l = HMC5883L(gauss = 4.7, declination = (-2,5))

        br = tf.TransformBroadcaster()

        accelerator = adxl345.getAxes(False)
        accelerator_norm=sqrt((accelerator['x']**2)+(accelerator['y']**2)+(accelerator['z']**2))
        accelerator['x'] /= accelerator_norm
        accelerator['y'] /= accelerator_norm
        accelerator['z'] /= accelerator_norm

        mag = hmc5883l.getAxes()
        mag_norm=sqrt((mag['x']**2)+(mag['y']**2)+(mag['z']**2))
        mag_n = dict()                
        mag_n['y'] = mag['x'] /mag_norm
        mag_n['x'] = -mag['y'] /mag_norm
        mag_n['z'] = mag['z'] /mag_norm
        
        comp_roll  = atan2(accelerator['y'], sqrt(accelerator['x'] ** 2 + accelerator['z'] ** 2))
        comp_pitch = atan2(-accelerator['x'], sqrt(accelerator['y'] ** 2 + accelerator['z'] ** 2))
        comp_yaw = atan2( (-mag_n['y']*cos(comp_roll) + mag_n['z']*sin(comp_roll) ) , (mag_n['x']*cos(comp_pitch) + mag_n['y']*sin(comp_pitch)*sin(comp_roll)+ mag_n['z']*sin(comp_pitch)*cos(comp_roll)) ) 

        while not rospy.is_shutdown():
            accelerator = adxl345.getAxes(False)
            dxyz = l3gd20.Get_CalOut_Value()
            mag = hmc5883l.getAxes()

            gyro['x'] += dxyz[0] * 1.0/20.0 * pi / 180.0
            gyro['y'] += dxyz[1] * 1.0/20.0 * pi / 180.0
            gyro['z'] += dxyz[2] * 1.0/20.0 * pi / 180.0
            try:

                accelerator_norm=sqrt((accelerator['x']**2)+(accelerator['y']**2)+(accelerator['z']**2))
                accelerator['x'] /= accelerator_norm
                accelerator['y'] /= accelerator_norm
                accelerator['z'] /= accelerator_norm

                acc_roll  = atan2(accelerator['y'], sqrt(accelerator['x'] ** 2 + accelerator['z'] ** 2))
                acc_pitch = atan2(-accelerator['x'], sqrt(accelerator['y'] ** 2 + accelerator['z'] ** 2))

                mag_norm=sqrt((mag['x']**2)+(mag['y']**2)+(mag['z']**2))
                mag_n = dict()                
                mag_n['y'] = mag['x'] /mag_norm
                mag_n['x'] = -mag['y'] /mag_norm
                mag_n['z'] = mag['z'] /mag_norm


                mag_yaw = atan2( (-mag_n['y']*cos(acc_roll) + mag_n['z']*sin(acc_roll) ) , (mag_n['x']*cos(acc_pitch) + mag_n['y']*sin(acc_pitch)*sin(acc_roll)+ mag_n['z']*sin(acc_pitch)*cos(acc_roll)) ) 

                acc_q = tf.transformations.quaternion_from_euler(acc_roll, acc_pitch, 0.0)
                br.sendTransform((0.0, 1.0, 0.0),
                                 (acc_q[0], acc_q[1], acc_q[2], acc_q[3]),
                                 rospy.Time.now(),
                                 "acc",
                                 "map")

                gyro_q = tf.transformations.quaternion_from_euler(gyro['x'], gyro['y'], gyro['z'] + pi / 2.0)
                br.sendTransform((0.0, 0.5, 0.0),
                                 (gyro_q[0], gyro_q[1], gyro_q[2], gyro_q[3]),
                                 rospy.Time.now(),
                                 "gyro",
                                 "map")

                mag_q = tf.transformations.quaternion_from_euler(0.0, 0.0, mag_yaw)
                br.sendTransform((0.0, 1.5, 0.0),
                                 (mag_q[0], mag_q[1], mag_q[2], mag_q[3]),
                                 rospy.Time.now(),
                                 "mag",
                                 "map")

                comp_roll  = 0.93 * (comp_roll  - dxyz[1] * 1.0/20.0 * pi / 180.0) + 0.07 * acc_roll
                comp_pitch = 0.93 * (comp_pitch + dxyz[0] * 1.0/20.0 * pi / 180.0) + 0.07 * acc_pitch
                comp_yaw   = 0.93 * (comp_yaw   + dxyz[2] * 1.0/20.0 * pi / 180.0) + 0.07 * mag_yaw

                imu_q = tf.transformations.quaternion_from_euler(comp_roll, comp_pitch, comp_yaw)
                br.sendTransform((0.0, 0.0, 0.0),
                                 (imu_q[0], imu_q[1], imu_q[2], imu_q[3]),
                                 rospy.Time.now(),
                                 "imu",
                                 "map")



            except ZeroDivisionError as e:
                rospy.logerr(e)

            rate.sleep()

#        rospy.spin()

    except KeyboardInterrupt:
        manager.stop()
        pass
    except rospy.ROSInterruptException:
        manager.stop()
        pass
