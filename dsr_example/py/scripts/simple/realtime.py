import rospy
from DSR_ROBOT import *


# to start a realtime thing we will need to call a bunch of services the driver provides:
# dsr_hw_interface.cpp line 894
# // Realtime Operations
# m_nh_realtime_service[0] = private_nh_.advertiseService("realtime/connect_rt_control", &DRHWInterface::connect_rt_control_cb, this);
# m_nh_realtime_service[1] = private_nh_.advertiseService("realtime/disconnect_rt_control", &DRHWInterface::disconnect_rt_control_cb, this);
# m_nh_realtime_service[2] = private_nh_.advertiseService("realtime/get_rt_control_output_version_list", &DRHWInterface::get_rt_control_output_version_list_cb, this);
# m_nh_realtime_service[3] = private_nh_.advertiseService("realtime/get_rt_control_input_version_list", &DRHWInterface::get_rt_control_input_version_list_cb, this);
# m_nh_realtime_service[4] = private_nh_.advertiseService("realtime/get_rt_control_input_data_list", &DRHWInterface::get_rt_control_input_data_list_cb, this);
# m_nh_realtime_service[5] = private_nh_.advertiseService("realtime/get_rt_control_output_data_list", &DRHWInterface::get_rt_control_output_data_list_cb, this);
# m_nh_realtime_service[6] = private_nh_.advertiseService("realtime/set_rt_control_input", &DRHWInterface::set_rt_control_input_cb, this);
# m_nh_realtime_service[7] = private_nh_.advertiseService("realtime/set_rt_control_output", &DRHWInterface::set_rt_control_output_cb, this);
# m_nh_realtime_service[8] = private_nh_.advertiseService("realtime/start_rt_control", &DRHWInterface::start_rt_control_cb, this);
# m_nh_realtime_service[9] = private_nh_.advertiseService("realtime/stop_rt_control", &DRHWInterface::stop_rt_control_cb, this);
# m_nh_realtime_service[10] = private_nh_.advertiseService("realtime/set_velj_rt", &DRHWInterface::set_velj_rt_cb, this);
# m_nh_realtime_service[11] = private_nh_.advertiseService("realtime/set_accj_rt", &DRHWInterface::set_accj_rt_cb, this);
# m_nh_realtime_service[12] = private_nh_.advertiseService("realtime/set_velx_rt", &DRHWInterface::set_velx_rt_cb, this);
# m_nh_realtime_service[13] = private_nh_.advertiseService("realtime/set_accx_rt", &DRHWInterface::set_accx_rt_cb, this);
# m_nh_realtime_service[14] = private_nh_.advertiseService("realtime/read_data_rt", &DRHWInterface::read_data_rt_cb, this);
# m_nh_realtime_service[14] = private_nh_.advertiseService("realtime/write_data_rt", &DRHWInterface::write_data_rt_cb, this);

# IM THINKING:

# connect_rt_control    (port 12347?)
# set_rt_control_input
# set_rt_control_output
# start_rt_control

#may also need the "version" string from get_rt_control_input_version_list

    # THEN IN A LOOP:

    # read_data_rt
    # write_data_rt

# CLEANUP:
# stop_rt_control
# disconnect_rt_control_cb