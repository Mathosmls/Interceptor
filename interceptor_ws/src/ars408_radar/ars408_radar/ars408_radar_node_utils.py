MAP_0X60B = {
    "ID": "id",
    "DistLong": "dist_long",
    "DistLat": "dist_lat",
    "VrelLong": "vrel_long",
    "VrelLat": "vrel_lat",
    "DynProp": "dyn_prop",
    "RCS": "rcs",
}

MAP_0X60C = {
    "ID": "id",
    "DistLong_rms_raw": "dist_long_rms_raw",
    "DistLat_rms_raw": "dist_lat_rms_raw",
    "VrelLong_rms_raw": "vrel_long_rms_raw",
    "VrelLat_rms_raw": "vrel_lat_rms_raw",
    "ArelLat_rms_raw": "arel_lat_rms_raw",
    "ArelLong_rms_raw": "arel_long_rms_raw",
    "Orientation_rms_raw": "orientation_rms_raw",
    "MeasState": "meas_state",
    "ProbOfExist": "prob_of_exist",
}

MAP_0X60D = {
    "ID": "id",
    "ArelLong": "arel_long",
    "ArelLat": "arel_lat",
    "OrientationAngle": "orientation_angle",
    "Class": "object_class",
    "Width": "width",
    "Length": "length",
}

MAP_0X201 = {
    "SensorID": "sensor_id",
    "MaxDistanceCfg": "max_distance_cfg",
    "RadarPowerCfg": "radar_power_cfg",
    "SortIndex": "sort_index",
    "OutputTypeCfg": "output_type_cfg",
    "MotionRxState": "motion_rx_state",
    "RCS_Threshold": "rcs_threshold",

    "NVMReadStatus": "nvm_read_status",
    "NVMWriteStatus": "nvm_write_status",
    "Persistent_Error": "persistent_error",
    "Interference": "interference",
    "Temperature_Error": "temperature_error",
    "Temporary_Error": "temporary_error",
    "Voltage_Error": "voltage_error",
    "CtrlRelayCfg": "ctrl_relay_cfg",
    "SendExtInfoCfg": "send_ext_info_cfg",
    "SendQualityCfg": "send_quality_cfg",
}

MAP_0X701 = {
    "ID": "id",
    "DistLong": "dist_long",
    "DistLat": "dist_lat",
    "VrelLong": "vrel_long",
    "VrelLat": "vrel_lat",
    "DynProp": "dyn_prop",
    "RCS": "rcs",
}

MAP_0X702 = {
    "ID": "id",
    "DistLong_rms": "dist_long_rms",
    "DistLat_rms": "dist_lat_rms",
    "VrelLong_rms": "vrel_long_rms",
    "VrelLat_rms": "vrel_lat_rms",
    "Pdh0": "pdh0",
    "AmbigState": "ambig_state",
    "InvalidState": "invalid_state",
}


def fill_msg_from_dict(msg, data, mapping):
    for src, dst in mapping.items():
        if src in data:
            value = data[src]
            # conversion vers bool si nécessaire
            msg_type = type(getattr(msg, dst))
            if msg_type == bool:
                value = bool(value)
            setattr(msg, dst, value)
