import can
import time
from .decoders import decode_0x203,decode_0x204

def make_arbitration_id(id,base_arbitration_id):
        return base_arbitration_id | ((id & 0xF) << 4)


#Send non valid config to read all objects parameters
def read_param_objects(bus):
    filters_to_configure = [
        (0x0, 0,0,0,0),       # NofObj
        (0x1, 0,0,0,0),       # Distance
        (0x2, 0,0,0,0),       # Azimuth
        (0x3, 0,0,0,0),       # VrelOncome
        (0x4, 0,0,0,0),       # VrelDepart
        (0x5, 0,0,0,0),       # RCS
        (0x6, 0,0,0,0),       # Lifetime
        (0x7, 0,0,0,0),       # Size
        (0x8, 0,0,0,0),       # ProbExists
        (0x9, 0,0,0,0),       # Y
        (0xA, 0,0,0,0),       # X
        (0xB, 0,0,0,0),       # VYRightLeft
        (0xC, 0,0,0,0),       # VXOncome
        (0xD, 0,0,0,0),       # VYLeftRight
        (0xE, 0,0,0,0)       # VXDepart
    ]
    send_params_objects_0x202(bus,filters_to_configure,1,1)

#build buffer for : Cluster and Object filter configuration
def build_filter_cfg_0x202(
    filter_type: int,   # 0 = cluster, 1 = object
    filter_index: int,  # voir table 4
    active: int,
    valid: int,
    min_val: int = 0,
    max_val: int = 0,
):
    """
    Construit les 8 octets du message CAN 0x202 (FilterCfg)
    Les valeurs min/max doivent être DÉJÀ mises à l’échelle et signées.
    """

    buf = [0] * 8

    # Byte 0
    buf[0] |= (valid & 0x01) <<1
    buf[0] |= (active & 0x01) << 2
    buf[0] |= (filter_index & 0x0F) << 3
    buf[0] |= (filter_type & 0x01) << 7

    # MIN (à partir du bit 16)
    buf[1] |= (min_val >> 8) & 0x3F
    buf[2] |= min_val & 0xFF

    # MAX (à partir du bit 32)
    buf[3] |= (max_val >> 8) & 0x3F
    buf[4] |= max_val & 0xFF

    return buf


FILTER_SCALING = {
    0x0: {"res": 1.0,  "offset": 0},        # NofObj
    0x1: {"res": 0.1,  "offset": 0},        # Distance
    0x2: {"res": 0.025,  "offset": -52.375},    # Azimuth
    0x3: {"res": 0.0315,  "offset": 0},   # VrelOncome
    0x4: {"res": 0.0315,  "offset": 0},   # VrelDepart
    0x5: {"res": 0.025,  "offset": -52.375},    # RCS
    0x6: {"res": 0.1,  "offset": 0},        # Lifetime
    0x7: {"res": 0.025,  "offset": 0},      # Size
    0x8: {"res": 1.0,  "offset": 0},        # ProbExists
    0x9: {"res": 0.2,  "offset": -409.8},   # Y
    0xA: {"res": 0.2,  "offset": -1138.5},     # X
    0xB: {"res": 0.0315,  "offset": 0},   # VYRightLeft
    0xC: {"res": 0.0315,  "offset": 0},   # VXOncome
    0xD: {"res": 0.0315,  "offset": 0},   # VYLeftRight
    0xE: {"res": 0.0315,  "offset": 0},   # VXDepart
}



#Send the 0x202 buffer

def send_params_objects_0x202(bus, filters_to_configure, id=0,force_unactive =0,force_unvalid=0):
    
    for index, valid,active, min_v, max_v in filters_to_configure:
        print("try to modify filter index ", index)
        scale = FILTER_SCALING.get(index)
        if scale is None:
            raise ValueError(f"No scaling defined for filter index {hex(index)}")

        res = scale["res"]
        offset = scale["offset"]

        min_raw = int((min_v + offset)/ res) 
        max_raw = int((max_v + offset)/ res) 


        if force_unvalid :
            valid=0
        if force_unactive :
            active=0

        buf = build_filter_cfg_0x202(
            filter_type=1,
            filter_index=index,
            active=active,
            valid=valid,
            min_val=min_raw,
            max_val=max_raw,
        )

        msg = can.Message(arbitration_id=make_arbitration_id(id,0x202), data=buf, is_extended_id=False)
        bus.send(msg)
        time.sleep(0.05)





def build_filter_cfg_0x200(
    valid_l: list,   
    value_l:list
):
    """
    Construit les 8 octets du message CAN 0x200 (FilterCfg)
    Les valeurs doivent être DÉJÀ mises à l’échelle et signées.
    Le bit de validation doit être à 1 si on veut modifier le param :
    valid_l : [MaxDistance,SensorID,RadarPower,OutputType,SendQuality,SendExtInfo,SortIndex,StoreinNVM,CtrlRelay,Threshold]
    Valeur qu'on envoie :
    value_l : [MaxDistance,SensorID,RadarPower,OutputType,SendQuality,SendExtInfo,SortIndex,StoreinNVM,CtrlRelay,Threshold]
    
    """
    buf = [0] * 8
    for i, bit in enumerate(valid_l[:8]):
        if bit not in (0, 1, False, True):
            raise ValueError(f"build_filter_cfg_0x200 | Invalid validity bit at index {i} : {bit}")
        buf[0] |= int(bit) << i
    buf[1]= (value_l[0] >>2) & 0xFF
    buf[2]= (value_l[0] & 0x03)<< 6 
    buf[4]= (value_l[1] &0x07)| ((value_l[3]&0x03)<<3 )|((value_l[2] &0x07)<<5)
    buf[5]= ((valid_l[8] &0x01)| ((value_l[8]&0x01)<<1 )|((value_l[4] &0x01)<<2)|((value_l[5] &0x01)<<3) 
             |((value_l[6] &0x07)<<4) |((value_l[7] &0x01)<<6))
    buf[6]=(valid_l[9] &0x01)| ((value_l[8]&0x07)<<1)
            

    return buf


RADAR_CFG_0X200_SCALING = [
    {"name": "MaxDistance",   "res": 2},
    {"name": "SensorID",      "res": 1},
    {"name": "RadarPower",    "res": 1},
    {"name": "OutputType",    "res": 1},
    {"name": "SendQuality",   "res": 1},
    {"name": "SendExtInfo",   "res": 1},
    {"name": "SortIndex",     "res": 1},
    {"name": "StoreinNVM",    "res": 1},
    {"name": "CtrlRelay",     "res": 1},
    {"name": "Threshold",     "res": 1},
]


def send_params_cfg_0x200(bus, cfg, id=0):
    """
    cfg : dict {"ParamName": (valid, value)}
    id  : radar ID
    """
    valid_l = []
    value_l = []

    print("Sending config params (0x200)...")

    for param in RADAR_CFG_0X200_SCALING:
        name = param["name"]
        res  = param["res"]

        # récupère le tuple (valid, value), défaut = (0, 0)
        valid_bit, value = cfg.get(name, (0, 0))

        if res != 0:
            value_raw = int(value / res)
        else:
            value_raw = int(value)

        valid_l.append(int(valid_bit))
        value_l.append(value_raw)

    print("send_params_cfg0x200:", valid_l, value_l)

    buf = build_filter_cfg_0x200(valid_l, value_l)
    msg = can.Message(arbitration_id=make_arbitration_id(id, 0x200),
                      data=buf, is_extended_id=False)
    bus.send(msg)

    print("0x200 sent!")
    time.sleep(0.1)
       
