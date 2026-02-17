#exemple objects filters config 
filters_obj_0x202=[
#index, valid,active, min_v, max_v
        (0x0, 1,1,0,10),       # NofObj
        (0x1, 1,1,0,150),     # Distance
        (0x2, 0,0,0,0),       # Azimuth
        (0x3, 0,0,0,0),       # VrelOncome
        (0x4, 0,0,0,0),       # VrelDepart
        (0x5, 0,0,0,0),       # RCS
        (0x6, 0,0,0,0),       # Lifetime
        (0x7, 0,0,0,0),       # Size
        (0x8, 1,1,5,7),       # ProbExists
        (0x9, 0,0,0,0),       # Y
        (0xA, 0,0,0,0),       # X
        (0xB, 0,0,0,0),       # VYRightLeft
        (0xC, 0,0,0,0),       # VXOncome
        (0xD, 0,0,0,0),       # VYLeftRight
        (0xE, 0,0,0,0)       # VXDepart
    ]

#exemple params radar config 
cfg_radar_0x200_dict = {
    "MaxDistance":    (1, 196),
    "SensorID":       (1, 0),
    "RadarPower":     (1, 0),
    "OutputType":     (1, 1),
    "SendQuality":    (1, 1),
    "SendExtInfo":    (0, 1),
    "SortIndex":      (0, 1),
    "StoreInNVM":     (1, 1),
    "CtrlRelay":      (0, 0),
    "Threshold":      (0, 0),
}
