import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from .radar_lib.radar import Radar
from std_msgs.msg import Bool
from .ars408_radar_node_utils import *
from ars408_interfaces.msg import  Msg0x201,MsgObjects,MsgObject,Msg0x60B,Msg0x60C,Msg0x60D,Msg0x701,Msg0x702,MsgCluster,MsgClusters
from std_msgs.msg import Header
from ament_index_python.packages import get_package_share_directory
import threading
import yaml
import os

class RadarNode(Node):
    def __init__(self):
        super().__init__('radar_node')

        self.declare_parameter("cfg_radar_yaml", "")
        self.declare_parameter("filters_yaml", "")

        self.cfg_radar_yaml = self.get_parameter(
            "cfg_radar_yaml").get_parameter_value().string_value
        self.filters_yaml = self.get_parameter(
            "filters_yaml").get_parameter_value().string_value
        
        # --- Driver radar ---
        self.radar = Radar(id=0)

        # --- Publishers ---
        self.state_radar_pub = self.create_publisher(Msg0x201, 'radar/state_radar', 10)
        self.objects_pub = self.create_publisher(MsgObjects, 'radar/objects', 10)
        self.clusters_pub = self.create_publisher(MsgClusters, 'radar/clusters', 10)


        self.add_on_set_parameters_callback(self._on_param_change)

        # --- Thread for reading the radar ---
        self._radar_thread = threading.Thread(target=self._radar_loop)
        self._radar_thread.daemon = True
        self._radar_thread.start()

        #load the params
        self._apply_configs([
            rclpy.parameter.Parameter("cfg_radar_yaml", value=self.cfg_radar_yaml),
            rclpy.parameter.Parameter("filters_yaml", value=self.filters_yaml),
        ])

        self.get_logger().info("Radar node started")




    def _radar_loop(self):
        """non stop loop to iterate on radar.run()"""
        for event in self.radar.run():
            if event["type"] == "0x201":
                msg = self.state_dict_to_rosmsg(event["data"])
                self.state_radar_pub.publish(msg)

            if event["type"] == "0x60":
                msg_obj = self.objects_dict_to_rosmsg(event["data"])
                self.objects_pub.publish(msg_obj)

            if event["type"] == "0x70":
                msg_obj = self.clusters_dict_to_rosmsg(event["data"])
                self.clusters_pub.publish(msg_obj)

            # if event["type"] == "0x203":
            #     msg_203 = event["data"]
            #     self.get_logger().info(f"203 : {msg_203}")
            if event["type"] == "0x204":
                msg_204 = event["data"]
                self.get_logger().info(f"204 : {msg_204}")

    # ------------------------------------------------
    # Reload configs
    # ------------------------------------------------

    def _on_param_change(self, params):
        reload_needed = False
        for param in params:
            if param.name in ["cfg_radar_yaml", "filters_yaml"]:
                self.get_logger().info(f"Param {param.name} changed, reload needed")
                reload_needed = True
        if reload_needed:
            self._apply_configs(params)
        return SetParametersResult(successful=True)

    def _apply_configs(self,params):
        
        param_dict = {p.name: p.value for p in params}

        # ---------- Radar cfg 0x200 ----------
        if "cfg_radar_yaml" in param_dict:
            cfg_radar_path = param_dict["cfg_radar_yaml"]
            if cfg_radar_path:
                try:
                    cfg = self._load_cfg_radar(cfg_radar_path)
                    self.radar.send_cfg_0x200(cfg)
                    self.get_logger().info("Radar cfg 0x200 applied")
                except (FileNotFoundError, KeyError, yaml.YAMLError) as e:
                    self.get_logger().warning(f"Failed to load radar cfg YAML: {e}")
                except Exception as e:
                    self.get_logger().error(f"Failed to send radar cfg over CAN: {e}")

        # ---------- Object filters 0x202 ----------
        if "filters_yaml" in param_dict:
            filters_path = param_dict["filters_yaml"]
            if filters_path:
                try:
                    filters = self._load_filters(filters_path)
                    self.radar.send_objects_filter(filters)
                    self.get_logger().info("Filters 0x202 applied")
                except (FileNotFoundError, KeyError, yaml.YAMLError) as e:
                    self.get_logger().warning(f"Failed to load filters YAML: {e}")
                except Exception as e:
                    self.get_logger().error(f"Failed to send filters over CAN: {e}")

           

    # ------------------------------------------------
    # YAML loaders
    # ------------------------------------------------
    def _load_cfg_radar(self, path):
        """
        Load config from YAML et send dict :
        { "NaleParam": (valid, value), ... }
        """
        with open(path, "r") as f:
            data = yaml.safe_load(f)["cfg_radar_0x200"]

        cfg = {}
        for item in data:
            name = item.get("name")
            if name is None:
                raise RuntimeError("Each entry must have a 'name'")
            cfg[name] = (item["valid"], item["value"])

        return cfg

    def _load_filters(self, path):
        with open(path, "r") as f:
            data = yaml.safe_load(f)["filters_obj_0x202"]

        filters = []
        for item in data:
            filters.append((
                int(item["index"]),
                int(item["valid"]),
                int(item["active"]),
                float(item["min"]),
                float(item["max"]),
            ))
        return filters


    # -------------------
    # Transformations dict → ROS messages
    # -------------------
    def objects_dict_to_rosmsg(self, objects_dict):
        msg_objects = MsgObjects()
        msg_objects.header.stamp = self.get_clock().now().to_msg()
        msg_objects.header.frame_id = "map"

        # 0x60A (cycle header)
        info_60a = objects_dict.get("0x60a", {})
        msg_objects.nof_objects = info_60a.get("NofObjects", 0)
        msg_objects.meas_counter = info_60a.get("MeasCounter", 0)
        msg_objects.interface_version = info_60a.get("InterfaceVersion", 0)

        # --- Objects ---
        for obj_id, obj_data in objects_dict.get("objects", {}).items():
            radar_obj = MsgObject()

            if "0x60b" in obj_data:
                b = Msg0x60B()
                fill_msg_from_dict(b, obj_data["0x60b"], MAP_0X60B)
                radar_obj.msg0x60b = b

            if "0x60c" in obj_data:
                c = Msg0x60C()
                fill_msg_from_dict(c, obj_data["0x60c"], MAP_0X60C)
                radar_obj.msg0x60c = c

            if "0x60d" in obj_data:
                d = Msg0x60D()
                fill_msg_from_dict(d, obj_data["0x60d"], MAP_0X60D)
                radar_obj.msg0x60d = d

            msg_objects.objects.append(radar_obj)

        return msg_objects

    def state_dict_to_rosmsg(self, state_dict):
        msg = Msg0x201()
        fill_msg_from_dict(msg, state_dict, MAP_0X201)
        return msg
    

    def clusters_dict_to_rosmsg(self, clusters_dict):
        msg_clusters = MsgClusters()  # ton message "liste de clusters"

        # Header ROS
        msg_clusters.header.stamp = self.get_clock().now().to_msg()
        msg_clusters.header.frame_id = "map"

        # -------- 0x600 (cycle header) --------
        info_600 = clusters_dict.get("0x600", {})

        msg_clusters.nof_clusters_near = info_600.get("NofNearClusters", 0)
        msg_clusters.nof_clusters_far = info_600.get("NofFarClusters", 0)
        msg_clusters.meas_counter = info_600.get("MeasCounter", 0)
        msg_clusters.interface_version = info_600.get("InterfaceVersion", 0)

        # -------- Clusters --------
        for cluster_id, cluster_data in clusters_dict.get("clusters", {}).items():
            ros_cluster = MsgCluster()

            # ---- 0x701 General ----
            if "0x701" in cluster_data:
                m701 = Msg0x701()
                fill_msg_from_dict(m701, cluster_data["0x701"], MAP_0X701)
                ros_cluster.msg0x701 = m701

            # ---- 0x702 Quality (optionnel) ----
            if "0x702" in cluster_data:
                m702 = Msg0x702()
                fill_msg_from_dict(m702, cluster_data["0x702"], MAP_0X702)
                ros_cluster.msg0x702 = m702

            msg_clusters.clusters.append(ros_cluster)

        return msg_clusters

def main(args=None):
    rclpy.init(args=args)
    node = RadarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()