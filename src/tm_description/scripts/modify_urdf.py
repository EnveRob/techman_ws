
import os
import shutil
import sys

import xml.etree.cElementTree as ET

import rospy

# from _modify_urdf import *
from _modify_urdf import modify_urdf
from _modify_urdf import urdf_DH_from_tm_DH
from _modify_urdf import xyzrpys_from_urdf_DH
from tm_msgs.srv import AskItem


def _gen_urdf():
    rospy.init_node('modify_urdf')

    ###############################################################################################
    # example: generate a new_model file (xxxooo.urdf), base on tm5-900-norminal model
    # syntax : python3 modify_urdf.py original_model new_model
    # [key-in] original_model: tm5-900  , [key-in] new_model: xxxooo
    # [key-in] shell cmd $ python3 modify_urdf.py tm5-900 xxxooo
    ###############################################################################################

    if len(sys.argv) < 3:
        print('Incorrect syntax! at least 2 parameters are required')
        print('You can try: python3 modify_urdf.py tm5-900 test')
        return

    original_model = sys.argv[1]
    new_model = sys.argv[2]
    specific_w = ''
    # specific keyword default
    overwrite = False
    nominal_model_restore = False
    tm5_900_nominal_restore = False
    tm5_700_nominal_restore = False
    tm12_nominal_restore = False
    tm14_nominal_restore = False
    tm_model = 'reference'
    ###############################################################################################
    # You can restore some nominal kinematic parameters by using specific keyword settings
    if len(sys.argv) == 4:
        specific_w = sys.argv[3].upper()
    if new_model == 'tm5-900-nominal' or specific_w == '-K59':
        tm_model = 'tm5-900-nominal'
        nominal_model_restore = True
        tm5_900_nominal_restore = True
    elif new_model == 'tm5-700-nominal' or specific_w == '-K57':
        tm_model = 'tm5-700-nominal'
        nominal_model_restore = True
        tm5_700_nominal_restore = True
    elif new_model == 'tm12-nominal' or specific_w == '-K12':
        tm_model = 'tm12-nominal'
        nominal_model_restore = True
        tm12_nominal_restore = True
    elif new_model == 'tm14-nominal' or specific_w == '-K14':
        tm_model = 'tm14-nominal'
        nominal_model_restore = True
        tm14_nominal_restore = True        
    else:
        nominal_model_restore = False
    if nominal_model_restore is True:
        message_s0 = 'Notice! You have chosen to restore a ' + tm_model + ' urdf model file'
        rospy.loginfo('%s!' % message_s0)
    if specific_w == '-OW':
        overwrite = True
        message_s1 = 'Notice!!! You have chosen to overwrite the original ' + tm_model + ' file'
        rospy.loginfo('%s!' % message_s1)
    ###############################################################################################

    rospy.wait_for_service('tm_driver/ask_item')
    ask_item = rospy.ServiceProxy('tm_driver/ask_item', AskItem)

    # Notice !!! You must have finished to run the driver to connect to youur TM Robot before.
    # [svr] (ask_item) -> id:dh (DHTable),id:dd (DeltaDH)
    res_dh = ask_item('dh', 'DHTable', 1.0)
    res_dd = ask_item('dd', 'DeltaDH', 1.0)

    if not res_dh.value.startswith('DHTable={') or not res_dh.value.endswith('}'):
        rospy.logerr('stop service, invalid parameters dh')
        return
    if not res_dd.value.startswith('DeltaDH={') or not res_dd.value.endswith('}'):
        rospy.logerr('stop service, invalid parameters delta_dh')
        return

    if not nominal_model_restore or overwrite:
        rospy.loginfo('loading the correction kinematics parameters from your TM Robot')
        if specific_w == '-VAL':
            rospy.loginfo(res_dh.value)
            rospy.loginfo(res_dd.value)

    dh_strs = res_dh.value[9:-1].split(',')
    dd_strs = res_dd.value[9:-1].split(',')

    ###############################################################################################
    # You can restore some nominal kinematic parameters by using specific keyword settings
    if nominal_model_restore is True:
        if tm5_900_nominal_restore is True:
            rospy.loginfo('Restore with TM5-900 nominal kinematics parameters')
            res_dh = 'DHTable={0,-90,0,145.2,0,-270,270,-90,0,429,0,0,-180,180,0,0,411.5,0,0,-155,155,90,90,0,-122.3,0,-180,180,0,90,0,106,0,-180,180,0,0,0,113.15,0,-270,270}'
            res_dd = 'DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}'
        elif tm5_700_nominal_restore is True:
            rospy.loginfo('Restore with TM5-700 nominal kinematics parameters')
            res_dh = 'DHTable={0,-90,0,145.2,0,-270,270,-90,0,329,0,0,-180,180,0,0,311.5,0,0,-155,155,90,90,0,-122.3,0,-180,180,0,90,0,106,0,-180,180,0,0,0,113.15,0,-270,270}'
            res_dd = 'DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}'
        elif tm12_nominal_restore is True:
            rospy.loginfo('Restore with TM12 nominal kinematics parameters')
            res_dh = 'DHTable={0,-90,0,165.2,0,-270,270,-90,0,636.1,0,0,-180,180,0,0,557.9,0,0,-166,166,90,90,0,-156.3,0,-180,180,0,90,0,106,0,-180,180,0,0,0,113.15,0,-270,270}'
            res_dd = 'DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}'
        elif tm14_nominal_restore is True:
            rospy.loginfo('Restore with TM14 nominal kinematics parameters')
            res_dh = 'DHTable={0,-90,0,165.2,0,-270,270,-90,0,536.1,0,0,-180,180,0,0,457.9,0,0,-166,166,90,90,0,-156.3,0,-180,180,0,90,0,106,0,-180,180,0,0,0,113.15,0,-270,270}'
            res_dd = 'DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}'
        else:
            # Example: TM5-900 nominal kinematics parameters
            rospy.loginfo('Restore with TM5-900 nominal kinematics parameters')
            res_dh = 'DHTable={0,-90,0,145.2,0,-270,270,-90,0,429,0,0,-180,180,0,0,411.5,0,0,-155,155,90,90,0,-122.3,0,-180,180,0,90,0,106,0,-180,180,0,0,0,113.15,0,-270,270}'
            res_dd = 'DeltaDH={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}'
        rospy.loginfo(res_dh)
        rospy.loginfo(res_dd)
        dh_strs = res_dh[9:-1].split(',')
        dd_strs = res_dd[9:-1].split(',')
    ###############################################################################################

    if len(dh_strs) != 42:
        rospy.logerr('stop service, invalid dh parameters')
        return
    if len(dd_strs) != 30:
        rospy.logerr('stop service, invalid delta_dh parameters')
        return

    dh = [float(i) for i in dh_strs]
    dd = [float(i) for i in dd_strs]

    # find urdf path
    curr_path = os.path.dirname(os.path.abspath(__file__))
    dirs = ['src', 'devel']
    ind = -1
    for d in dirs:
        ind = curr_path.find(d)
        if (ind != -1):
            break
    if (ind == -1):
        rospy.logerr('workspace directory not find')
        return
    src_path = curr_path[:ind] + 'src'
    urdf_path = ''
    for dirpath, dirnames, filenames in os.walk(src_path):
        if dirpath.endswith('tm_description'):
            urdf_path = dirpath + '/urdf'
            break
    if (urdf_path == ''):
        rospy.logerr('urdf directory not found')
        return

    urdf_name = '/' + original_model + '-nominal.urdf'
    new_urdf_name = '/' + new_model + '.urdf'
    if specific_w == '+M':
        new_urdf_name = '/macro.' + new_model + '.urdf'

    file_in = urdf_path + urdf_name
    file_out = urdf_path + new_urdf_name

    link_head = "<?xml version=\"1.0\" ?>\n"
    rospy.loginfo('[reference file path:] %s' % file_in)

    fr = open(file_in, 'r')
    link_data = fr.read()
    fr.close()

    root = ET.fromstring(link_data)

    udh = urdf_DH_from_tm_DH(dh, dd)
    xyzs, rpys = xyzrpys_from_urdf_DH(udh)
    modify_urdf(root, xyzs, rpys, udh)

    link_data = ET.tostring(root, encoding='UTF-8').decode('UTF-8')

    data_out = link_head + link_data

    file_save = ''
    if overwrite:
        file_save = file_in
        shutil.copyfile(file_in, file_out)
    else:
        file_save = file_out

    fw = open(file_save, 'w')
    fw.write(data_out)

    if overwrite:
        rospy.loginfo('File saved with new kinematic values')
        rospy.loginfo('[overwrite reference file path:] ' + str(file_in))
        rospy.loginfo('[new save file path:] ' + str(file_out))
    elif nominal_model_restore:
        rospy.loginfo('File restored with the nominal kinematic values')
        rospy.loginfo('[new save file path:] ' + str(file_save))
    else:
        rospy.loginfo('File saved with new kinematic values')
        rospy.loginfo('[new save file path:] ' + str(file_save))
    fw.close()


def main():
    try:
        _gen_urdf()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
