from flow.networks.base import Network
from flow.core.params import InitialConfig
from flow.core.params import TrafficLightParams
from numpy import pi, sin, cos, linspace

ADDITIONAL_NET_PARAMS = {
    # length of the ring road
    "length": 230,
    # number of lanes
    "lanes": 1,
    # speed limit for all edges
    "speed_limit": 30,
}


class CrossRoad_Network(Network):
    def __init__(self,
                 name,
                 vehicles,
                 net_params,
                 initial_config=InitialConfig(),
                 traffic_lights=TrafficLightParams()):
        super().__init__(name, vehicles, net_params, initial_config,
                         traffic_lights)

    def specify_nodes(self, net_params):
        length = net_params.additional_params["length"]
        nodes = [{"id": "center",
                  "x": 0,
                  "y": 0},
                {
                "id": "left",
                "x": -length / 2,
                "y": 0},
                {"id": "right",
                 "x": length / 2,
                 "y": 0},
                {"id": "bottom",
                 "x": 0,
                 "y": -length / 2},
                {"id": 'top',
                 "x": 0,
                 "y": length / 2,
                 }]
        return nodes

    def specify_edges(self, net_params):
        """See parent class."""
        length = net_params.additional_params["length"]
        edges = [
                {"id":
                    "horizontal_left_center",
                 "type":
                 "edgeType",
                "from":
                    "left",
                "to":
                    "center",
                "length":
                    length/2,},
                {"id":
                    "horizontal_center_left",
                 "type":
                     "edgeType",
                "from":
                    "center",
                "to":
                    "left",
                "length":
                    length / 2,},
                {"id":
                     "horizontal_right_center",
                 "type":
                     "edgeType",
                 "from":
                     "right",
                 "to":
                     "center",
                 "length":
                     length / 2, },
                {"id":
                     "horizontal_center_right",
                 "type":
                     "edgeType",
                 "from":
                     "center",
                 "to":
                     "right",
                 "length":
                 length / 2, },
                {"id":
                     "vertical_bottom_center",
                 "type":
                     "edgeType",
                 "from":
                     "bottom",
                 "to":
                     "center",
                 "length":
                     length / 2, },
                {"id":
                     "vertical_center_bottom",
                 "type":
                     "edgeType",
                 "from":
                     "center",
                 "to":
                     "bottom",
                 "length":
                     length / 2, },
                {"id":
                     "vertical_center_top",
                 "type":
                     "edgeType",
                 "from":
                     "center",
                 "to":
                     "top",
                 "length":
                     length / 2, },
                {"id":
                     "vertical_top_center",
                 "type":
                     "edgeType",
                 "from":
                     "top",
                 "to":
                     "center",
                 "length":
                 length / 2, },
                ]
        return edges

    def specify_routes(self, net_params):
        """See parent class."""
        rts = {
            "horizontal_left_center": ["horizontal_left_center","vertical_center_bottom"],
            "horizontal_right_center": ["horizontal_right_center","horizontal_center_left"],
            "vertical_bottom_center":["vertical_bottom_center","vertical_center_top"],
            "vertical_top_center": ["vertical_top_center", "vertical_center_bottom"],
            "horizontal_center_left":["horizontal_center_left"],
            "vertical_center_top":["vertical_center_top"]
            # "horizontal_center_top": ["horizontal_center_top"]
            # "vertical_bottom": ["vertical_bottom"],
            # "horizontal_right": ["horizontal_right"],
            # "vertical_top": ["vertical_top"],
        }
        return rts

    def specify_types(self, net_params):
        """See parent class."""
        lanes = net_params.additional_params["lanes"]
        speed_limit = net_params.additional_params["speed_limit"]

        types = [{
            "id": "edgeType",
            "numLanes": lanes,
            "speed": speed_limit
        }]

        return types