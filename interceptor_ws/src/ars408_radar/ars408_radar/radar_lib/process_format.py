from .decoders import decode_0x60A,decode_0x60B,decode_0x60C,decode_0x60D,decode_0x600,decode_0x701,decode_0x702
#utils fonctions to apply scales/res and handle the objects can messages

#scaling params for the objects info
scaling_dict_obj = {
    "0x60b": {  # Object_1_General
        "DistLong_raw": {"res": 0.2, "offset": -500},
        "DistLat_raw": {"res": 0.2, "offset": -204.6},
        "VrelLong_raw": {"res": 0.25, "offset": -128},
        "VrelLat_raw": {"res": 0.25, "offset": -64},
        "RCS_raw": {"res": 0.5, "offset": -64},
        "DynProp": {"res": 1, "offset": 0},  
        "ID": {"res": 1, "offset": 0}
    },
    "0x60c": {  # Object_2_Quality
        # "DistLong_rms_raw": {"res": None, "offset": None},
        # "DistLat_rms_raw": {"res": None, "offset": None},
        # "VrelLong_rms_raw": {"res": None, "offset": None},
        # "VrelLat_rms_raw": {"res": None, "offset": None},
        # "ArelLong_rms_raw": {"res": None, "offset": None},
        # "ArelLat_rms_raw": {"res": None, "offset": None},
        # "Orientation_rms_raw": {"res": None, "offset": None},
        # "MeasState": {"res": 1, "offset": 0},
        # "ProbOfExist": {"res": 1, "offset": 0},
        "ID": {"res": 1, "offset": 0}
    },
    "0x60d": {  # Object_3_Extended
        "ArelLong_raw": {"res": 0.01, "offset": -10},
        "ArelLat_raw": {"res": 0.01, "offset": -2.5},
        "OrientationAngle_raw": {"res": 0.4, "offset": -180},
        "Length_raw": {"res": 0.2, "offset": 1},
        "Width_raw": {"res": 0.2, "offset": 1},
        "Class_raw": {"res": 1, "offset": 0},  # généralement pas de conversion
        "ID": {"res": 1, "offset": 0}
    }
}


scaling_dict_clusters = {
        "ID": {"res": 1, "offset": 0},
        "DistLong": {"res": 0.2, "offset": -500},
        "DistLat": {"res": 0.2, "offset": -102.3},
        "VrelLong": {"res": 0.25, "offset": -128},
        "VrelLat": {"res": 0.25, "offset": -64},
        "DynProp": {"res": 1, "offset": 0},
        "RCS": {"res": 0.5, "offset": -64}   
}



#Create dict with the 0x6... trams 
def handle_object_messages(msg, state,send_quality,send_extInfo):
    """
    Assembles the object messages in a cycle.
    While the cycle is not complete → returns None.
    When it is complete → returns the complete cycle.
    """

    DECODERS = {
        0x60A: decode_0x60A,
        0x60B: decode_0x60B,
        0x60C: decode_0x60C,
        0x60D: decode_0x60D,
    }

    arb_id = msg.arbitration_id
    base_id = arb_id & 0xF0F

    if base_id not in DECODERS:
        return None

    decoded = DECODERS[base_id](msg.data)

    # ---------- 0x60A : nouveau cycle ----------
    if base_id == 0x60A:
        state["current_cycle"] = {
            "0x60a": decoded,
            "objects": {}
        }
        state["expected_objects"] = decoded["NofObjects"]
        return None

    # ---------- Ignore si pas encore de cycle ----------
    if "current_cycle" not in state:
        return None

    # ---------- Messages objets ----------
    obj_id = decoded["ID"]
    objects = state["current_cycle"]["objects"]

    if obj_id not in objects:
        objects[obj_id] = {}

    objects[obj_id][hex(base_id)] = decoded

    # ---------- Vérification de complétude ----------
    complete_objects = 0
    for obj in objects.values():
        if "0x60b" in obj and (not send_quality or "0x60c" in obj) and (not send_extInfo or"0x60d" in obj): 
            complete_objects += 1

    if complete_objects >= state["expected_objects"]:
        # Cycle complet
        complete_cycle = state["current_cycle"]
        # Nettoyage
        del state["current_cycle"]
        del state["expected_objects"]

        return complete_cycle

    return None

#Decode the raw object dicts to real value
def process_object_values(state_structured, scaling):
    """
    Transforms the raw values ​​of each object into readable physical values.
    Compatible with a structure of the following type:
    {"0x60a": {...},
        "objects": {
            obj_id: {
                "0x60b": {...},
                "0x60c": {...},
                ...}}}
     """

    result = {
        "0x60a": state_structured.get("0x60a"),
        "objects": {}
    }

    for obj_id, obj_data in state_structured.get("objects", {}).items():
        result["objects"][obj_id] = {}

        for msg_type, msg_values in obj_data.items():
            # msg_type = "0x60b", "0x60c", etc.
            result["objects"][obj_id][msg_type] = {}

            scaling_msg = scaling.get(msg_type)

            # Pas de scaling pour ce message → on copie tel quel
            if scaling_msg is None:
                result["objects"][obj_id][msg_type] = msg_values
                continue

            for key, raw_value in msg_values.items():
                if key in scaling_msg:
                    res = scaling_msg[key].get("res", 1)
                    offset = scaling_msg[key].get("offset", 0)

                    new_key = key.replace("_raw", "")
                    result["objects"][obj_id][msg_type][new_key] = raw_value * res + offset
                else:
                    # pas de règle → valeur brute conservée
                    result["objects"][obj_id][msg_type][key] = raw_value

    return result

def format_cfg_0x201(data) :
  #apply the res to 0x201 msgs
  data["MaxDistanceCfg"]*=2
  return data




def handle_cluster_messages(msg, state, send_quality):
    """
    Assemble les messages clusters en un cycle complet.
    Tant que le cycle n'est pas complet → retourne None.
    Quand il est complet → retourne le cycle complet.
    
    send_quality : bool, si True, on attend aussi les 0x702 (Cluster_2_Quality)
    """

    DECODERS = {
        0x600: decode_0x600,  # Cluster_0_Status
        0x701: decode_0x701,  # Cluster_1_General
        0x702: decode_0x702,  # Cluster_2_Quality
    }

    arb_id = msg.arbitration_id
    obj_id = (arb_id >> 4) & 0xF
    base_id = arb_id & 0xF0F

    if base_id not in DECODERS:
        return None

    decoded = DECODERS[arb_id](msg.data)

    # ---------- 0x600 : nouveau cycle ----------
    if arb_id == 0x600:
        state["current_cycle"] = {
            "0x600": decoded,
            "clusters": {}
        }
        state["expected_clusters"] = decoded.get("NofNearClusters", 0) + decoded.get("NofFarClusters", 0)
        return None

    # ---------- Ignore si pas encore de cycle ----------
    if "current_cycle" not in state:
        return None

    # ---------- Messages clusters ----------
    # On crée un identifiant unique pour chaque cluster (near/far)
    cluster_id = decoded["ID"] # ou un index fourni par le decode
    clusters = state["current_cycle"]["clusters"]
    if cluster_id not in clusters:
        clusters[cluster_id] = {}

    clusters[cluster_id][hex(base_id)] = decoded

    # ---------- Vérification de complétude ----------
    complete_clusters = 0
    for cluster in clusters.values():
        # 0x701 requis, 0x702 optionnel selon send_quality
        if "0x701" in cluster and (not send_quality or "0x702" in cluster):
            complete_clusters += 1

    if complete_clusters >= state["expected_clusters"]:
        complete_cycle = state["current_cycle"]
        del state["current_cycle"]
        del state["expected_clusters"]
        return complete_cycle

    return None


def process_cluster_values(state_structured, scaling):
    """
    Transforme les valeurs raw de chaque cluster en valeurs physiques lisibles.
    Compatible avec une structure du type :
    {
        "0x600": {...},
        "clusters": {
            cluster_id: {
                "0x701": {...},
                "0x702": {...}  # optionnel si send_quality=True
            }
        }
    }
    """

    result = {
        "0x600": state_structured.get("0x600"),
        "clusters": {}
    }

    for cluster_id, cluster_data in state_structured.get("clusters", {}).items():
        result["clusters"][cluster_id] = {}

        for msg_type, msg_values in cluster_data.items():
            result["clusters"][cluster_id][msg_type] = {}

            for key, raw_value in msg_values.items():
                if key in scaling:
                    res = scaling[key].get("res", 1)
                    offset = scaling[key].get("offset", 0)
                    result["clusters"][cluster_id][msg_type][key] = raw_value * res + offset
                else:
                    # Pas de règle → valeur brute conservée
                    result["clusters"][cluster_id][msg_type][key] = raw_value

    return result

