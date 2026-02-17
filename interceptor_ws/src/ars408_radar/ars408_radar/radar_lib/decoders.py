#Decode radar can messages


#Radar state
def decode_0x201(buf):
    """
    Radar state msg (0x201) decoding
    """

    NVMReadStatus = (buf[0] & 0x40) >> 6
    NVMWriteStatus = (buf[0] & 0x80) >> 7

    MaxDistanceCfg = (buf[1] << 2) | ((buf[2] & 0xC0) >> 6)

    Persistent_Error = (buf[2] & 0x20)>>5
    Interference = (buf[2] & 0x10) >>4
    Temperature_Error = (buf[2] & 0x08) >>3
    Temporary_Error = (buf[2] & 0x04) >>2
    Voltage_Error = (buf[2] & 0x02) >> 1

    RadarPowerCfg = ((buf[3] & 0x03)<< 1) |((buf[4] & 0x80) >> 7)
    
    SortIndex =  ((buf[4] & 0x70)>> 4) 
    SensorID = buf[4] & 0x07 
    
    MotionRxState= (buf[5] & 0xC0) >>6
    SendExtInfoCfg = (buf[5]& 0x20)>>5
    SendQualityCfg = (buf[5]& 0x10)>>4
    OutputTypeCfg= (buf[5]& 0x0C)>>2
    CtrlRelayCfg= (buf[5]& 0x02)>>1

    RCS_Threshold= (buf[7]& 0x1C)>>2


    return {
        "NVMReadStatus": NVMReadStatus,
        "NVMWriteStatus": NVMWriteStatus,
        "MaxDistanceCfg": MaxDistanceCfg,

        "Persistent_Error": Persistent_Error,
        "Interference": Interference,
        "Temperature_Error": Temperature_Error,
        "Temporary_Error": Temporary_Error,
        "Voltage_Error": Voltage_Error,

        "RadarPowerCfg": RadarPowerCfg,
        "SortIndex": SortIndex,
        "SensorID": SensorID,

        "MotionRxState": MotionRxState,
        "SendExtInfoCfg": SendExtInfoCfg,
        "SendQualityCfg": SendQualityCfg,
        "OutputTypeCfg": OutputTypeCfg,
        "CtrlRelayCfg": CtrlRelayCfg,

        "RCS_Threshold": RCS_Threshold
}  

#Cluster and Object filter configuration state header
def decode_0x203(buf):
    """
    Decode Cluster and Object filter configuration state header (0x203)
    """

    # bits 3..7 of byte 0
    FilterState_NofClusterFilterCfg = (buf[0] >> 3) & 0x1F

    # bits 3..7 of byte 1
    FilterState_NofObjectFilterCfg = (buf[1] >> 3) & 0x1F

    return {
        "NofClusterFilterCfg": FilterState_NofClusterFilterCfg,
        "NofObjectFilterCfg": FilterState_NofObjectFilterCfg,
    }

#Cluster and Object filter configuration state
def decode_0x204(buf):
    """
    Decoding object filters params (0x204)
    """
    FilterState_Active = (buf[0] >> 2) & 0x01
    FilterState_Index  = (buf[0] >> 3) & 0x0F
    FilterState_Type   = (buf[0] >> 7) & 0x01

    min_raw = (buf[1] <<8)| buf[2]
    max_raw = (buf[3] <<8)| buf[4]
   

    return {
        "Active": FilterState_Active,
        "Type": "Object" if FilterState_Type else "Cluster",
        "Index": FilterState_Index,
        "MinRaw": min_raw,
        "MaxRaw": max_raw,
    }

#Object list status
def decode_0x60A(buf):
    """
    Decoding objects header (0x60A)
    """
    NofObjects = buf[0]
    MeasCounter = (buf[1] << 8) | buf[2]
    InterfaceVersion = (buf[3] >> 4) & 0x0F

    return {
        "NofObjects": NofObjects,
        "MeasCounter": MeasCounter,
        "InterfaceVersion": InterfaceVersion
    }

# Object general information
def decode_0x60B(buf):
    """
    Decode Object general information (0x60B)
    """
    ID = buf[0]
    Distlong = (buf[1] << 5) | ((buf[2] >> 3) & 0x1F)
    DistLat = ((buf[2] & 0x07) << 8) | buf[3]
    VrelLong = (buf[4] << 2) | ((buf[5] >> 6) & 0x03)
    VrelLat = ((buf[5] & 0x3F) << 3) | ((buf[6] >> 5) & 0x07)
    DynProp = buf[6] & 0x07
    RCS = buf[7]

    return {
        "ID": ID,
        "DistLong_raw": Distlong,
        "DistLat_raw": DistLat,
        "VrelLong_raw": VrelLong,
        "VrelLat_raw": VrelLat,
        "DynProp": DynProp,
        "RCS_raw": RCS
    }

# Object quality information
def decode_0x60C(buf):
    """
    Decode Object quality informatio (0x60C)
    """
    ID = buf[0]
    DistLong_rms = (buf[1]>>3)&0x1F
    DistLat_rms = ((buf[1] & 0x07)<<2) | ((buf[2]>>6) &0x03) 
    VrelLong_rms = (buf[2]>>1)&0x1F
    VrelLat_rms = ((buf[2]&0x01)<<4)|((buf[3]>>4)&0x0F)
    ArelLong_rms = ((buf[3]&0x0F)<<1) |((buf[4]>>7)&0x01)
    ArelLat_rms = (buf[4] >> 2) & 0x1F
    Orientation_rms = ((buf[4] & 0x03)<<3)|(buf[5]>>5)
    ProbOfExist = (buf[6] >> 5) & 0x07
    MeasState = (buf[6] >> 2) & 0x07

    return {
        "ID": ID,
        "DistLong_rms_raw": DistLong_rms,
        "VrelLong_rms_raw": VrelLong_rms,
        "DistLat_rms_raw": DistLat_rms,
        "VrelLat_rms_raw": VrelLat_rms,
        "ArelLat_rms_raw": ArelLat_rms,
        "ArelLong_rms_raw": ArelLong_rms,
        "Orientation_rms_raw": Orientation_rms,
        "MeasState": MeasState,
        "ProbOfExist": ProbOfExist
    }

# Object extended information
def decode_0x60D(buf):
    """
    Decode Object extended information (0x60D)
    """
    ID = buf[0]
    ArelLong = (buf[1]<<3)|((buf[2]>>5)&0x07)
    ArelLat = ((buf[2] & 0x1F)<<4)|((buf[3]>>4)&0x0F)
    Class = buf[3] & 0x07 
    OrientationAngle = (buf[4]<<2)|((buf[5]>>6)&0x03)
    Length = buf[6]
    Width = buf[7]
 

    return {
        "ID": ID,
        "ArelLong_raw": ArelLong,
        "Class_raw": Class,
        "ArelLat_raw": ArelLat,
        "OrientationAngle_raw": OrientationAngle,
        "Width_raw": Width,
        "Length_raw": Length,
 
    }



#Cluster list status
def decode_0x600(buf):
    """
    Decoding of msg Cluster list status (0x600)
    """
    NofClustersNear = buf[0]
    NofClustersFar = buf[1]
    MeasCounter = (buf[2] << 8) | buf[3]
    InterfaceVersion = (buf[4]>>4) & 0x0F

    return {
        "NofClustersNear": NofClustersNear,
        "NofClustersFar": NofClustersFar,
        "MeasCounter": MeasCounter,
        "InterfaceVersion": InterfaceVersion
    }   


#Cluster general information
def decode_0x701(buf):
    """
    Decoding of msg Cluster_1_General (0x701)
    """
    ID = buf[0]
    DistLong = (buf[1] << 5) | ((buf[2] >> 3) & 0x1F)
    DistLat = ((buf[2] & 0x03) << 8) | buf[3]
    VrelLong = ((buf[4] & 0xFF) << 2) | ((buf[5]>>6) & 0x03)
    VrelLat = ((buf[5] & 0x3F) << 3) | ((buf[6]>> 5) & 0x07)
    DynProp = buf[6] & 0x07  
    RCS = buf[7] 

    return {
        "ID": ID,
        "DistLong": DistLong,
        "DistLat": DistLat,
        "VrelLong": VrelLong,
        "VrelLat": VrelLat,
        "DynProp": DynProp,
        "RCS": RCS
    }   

#Cluster quality information

def decode_0x702(buf):
    """
    Decoding of msg Cluster quality information (0x702)
    """
    ID = buf[0]
    DistLong_rms = (buf[1]>>3) & 0x1F
    DistLat_rms = ((buf[1] & 0x07)<<2) | ((buf[2]>>5)& 0x03)
    VrelLong_rms = (buf[2]>>1)& 0x1F
    VrelLat_rms = ((buf[2] & 0x01) << 4) | ((buf[3]>> 4) & 0x0F)
    Pdh0 = buf[3] & 0x07  
    AmbigState = (buf[4]>>5)& 0x1F 
    InvalidState= buf[4]& 0x07

    return {
        "ID": ID,
        "DistLong_rms": DistLong_rms,
        "DistLat_rms": DistLat_rms,
        "VrelLong_rms": VrelLong_rms,
        "VrelLat_rms": VrelLat_rms,
        "Pdh0": Pdh0,
        "AmbigState": AmbigState,
        "InvalidState": InvalidState
    }   



