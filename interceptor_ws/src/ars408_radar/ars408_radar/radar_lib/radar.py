#Radar class for the ars408 


import can

from .decoders import decode_0x201,decode_0x203,decode_0x204
from .senders import read_param_objects, send_params_cfg_0x200,send_params_objects_0x202,make_arbitration_id
from .process_format import handle_object_messages, process_object_values, format_cfg_0x201,handle_cluster_messages,process_cluster_values
from .process_format import scaling_dict_obj,scaling_dict_clusters

class Radar:
    def __init__(self, channel="can0", interface="socketcan",scaling_objects=scaling_dict_obj,scaling_clusters=scaling_dict_clusters,id=0):
        self.bus = can.interface.Bus(channel=channel, interface=interface)
        self.object_state = {}
        self.cluster_state = {}
        self.last_status = None
        self.last_objects = None
        self.last_clusters = None 
        self.scaling_objects =scaling_objects
        self.scaling_clusters =scaling_clusters
        self.id=id
        self.send_quality=False
        self.send_extInfo=False
    # ------------------------
    # CONFIG / SEND
    # ------------------------
    def send_cfg_0x200(self, cfg):
        send_params_cfg_0x200(self.bus, cfg,id=self.id)
        valid, value = cfg.get("SensorID") 
        if valid :
            self.id = value

    def send_objects_filter(self, filter_obj):
           send_params_objects_0x202(self.bus, filter_obj,id=self.id,force_unactive =0,force_unvalid=0)
    
    def read_objects_filter(self) :
        read_param_objects(self.bus)
    # ------------------------
    # RECEIVE / HANDLE
    # ------------------------

    def handle_msg(self, msg):
        if (msg.arbitration_id & 0xF0F) == 0x201:  # base_id 0x201
            decoded = decode_0x201(msg.data)
            self.last_status = format_cfg_0x201(decoded)

            # mettre à jour l'ID dynamique
            new_id = self.last_status["SensorID"]
            if new_id != self.id:
                self.get_logger().info(f"Radar ID updated: {self.id} -> {new_id}")
                self.id = new_id

            self.send_extInfo = self.last_status["SendExtInfoCfg"]
            self.send_quality  = self.last_status["SendQualityCfg"]
            return {
                "type": "0x201", 
                "data": self.last_status
            }

        
        if msg.arbitration_id == make_arbitration_id(self.id,0x203) :
            decoded = decode_0x203(msg.data)
            return {
                "type": "0x203",
                "data": decoded
            }
        if msg.arbitration_id == make_arbitration_id(self.id,0x204) :
            decoded = decode_0x204(msg.data)
            return {
                "type": "0x204",
                "data": decoded
            }
        cycle_obj = handle_object_messages(msg, self.object_state,self.send_quality,self.send_extInfo)
        if cycle_obj is not None:
            processed_objects = process_object_values(cycle_obj, self.scaling_objects)
            self.last_objects = processed_objects
            return {
                "type": "0x60",
                "data": processed_objects
            }
        
        cycle_cluster = handle_cluster_messages(msg, self.cluster_state,self.send_quality)
        if cycle_cluster is not None:
            processed_clusters = process_cluster_values(cycle_cluster,self.scaling_clusters)
            self.last_clusters = processed_clusters
            return {
                "type": "0x70", 
                "data": processed_clusters}

        return None


    # ------------------------
    # LOOP HELPERS
    # ------------------------

    def recv_once(self, timeout=None):
        msg = self.bus.recv(timeout=timeout)
        if msg is None:
            return None
        return self.handle_msg(msg)

    def run(self):
        for msg in self.bus:
            event = self.handle_msg(msg)
            if event is not None:
                yield event

    # ------------------------
    # GETTERS
    # ------------------------

    def get_last_status(self):
        return self.last_status

    def get_last_objects(self):
        return self.last_objects
    
    def get_last_clusters(self):
        return self.last_clusters

    # ------------------------
    # CLEANUP
    # ------------------------

    def shutdown(self):
        try:
            self.bus.shutdown()
        except Exception:
            pass
