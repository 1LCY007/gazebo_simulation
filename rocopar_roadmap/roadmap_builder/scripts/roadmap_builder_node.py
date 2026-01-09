#!/usr/bin/env python3
import copy
import math
import time
from functools import wraps
from itertools import combinations

import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import rospy
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import OccupancyGrid
from roadmap_builder.msg import RoadMapEdge, RoadMapNode, RoadMap
from roadmap_builder.srv import (
    GenVoronoi,
    GenVoronoiRequest,
    GetFrontiers,
    GetFrontiersRequest,
)
from scipy.spatial import Delaunay, Voronoi
from tuw_multi_robot_msgs.msg import Graph
from visualization_msgs.msg import Marker, MarkerArray


def measure_time(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        rospy.loginfo(f"{func.__name__} took: {end_time - start_time:.4f} seconds")
        return result
    return wrapper

def get_dist(ps, pt):
    return round(math.sqrt((pt[0]-ps[0])**2 + (pt[1]-ps[1])**2), 3)

def point_line_distance(point, line):
    point_p = np.array(point)
    point_a = np.array(line[0])
    point_b = np.array(line[1])
    vector_ap = point_p - point_a
    vector_ab = point_b - point_a
    proj_length = np.dot(vector_ap, vector_ab)/np.dot(vector_ab, vector_ab)
    if proj_length < 0:
        return np.linalg.norm(vector_ap)
    elif proj_length > 1:
        return np.linalg.norm(point_p - point_b)
    else:
        return np.abs(np.cross(vector_ap, vector_ab)) / np.linalg.norm(vector_ab)

def create_point_marker(ns, id, point, color, scale):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.id = id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = point[0]
    marker.pose.position.y = point[1]
    marker.pose.position.z = 0.05
    marker.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color.a = color[0]
    marker.color.r = color[1]
    marker.color.g = color[2]
    marker.color.b = color[3]
    # marker.text = 'yes'
    return marker

def create_line_marker(ns, id, edge, color, scale):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.id = id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = scale
    marker.color.a = color[0]
    marker.color.r = color[1]
    marker.color.g = color[2]
    marker.color.b = color[3]
    point_f = Point(edge[0][0], edge[0][1], 0.05)
    point_t = Point(edge[1][0], edge[1][1], 0.05)
    marker.points = [point_f, point_t]
    return marker

def _set_frontier_marker_msg(frontiers):
    mes = MarkerArray()
    for id, frontier in enumerate(frontiers):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "my_namespace"
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = frontier.centroid.x
        marker.pose.position.y = frontier.centroid.y
        marker.pose.position.z = 0.05
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration(10.0)
        mes.markers.append(marker)
    return mes


class RoadmapBuilderNode():
    def __init__(self):
        rospy.init_node("roadmap_builder")
        # Variables
        self.occup_map = OccupancyGrid()
        self.filter_map = OccupancyGrid()
        self._voronoi_map_msg = Graph()
        self._roadmap_msg = RoadMap()
        self._roadmap_vis_msg = MarkerArray()
        self._centroid_marker_msg = MarkerArray()

        self.roadmap = nx.Graph()
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.segment_length = rospy.get_param("~segment_length", 1.0)
        # ROS interfaces
        # why buff_size=2**24? https://stackoverflow.com/questions/26415699/ros-subscriber-not-up-to-date
        rospy.wait_for_service("/gen_voronoi", timeout=10)
        self.voronoi_client = rospy.ServiceProxy("/gen_voronoi", GenVoronoi)
        rospy.wait_for_service('/get_frontiers', timeout=10)
        self.frontier_client = rospy.ServiceProxy('/get_frontiers', GetFrontiers)

        rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback, queue_size=1, buff_size=2**24)
        # self.filter_map_pub = rospy.Publisher('/filter_map', OccupancyGrid, queue_size=1)
        self.voronoi_pub = rospy.Publisher("/voronoi_map", Graph, queue_size=1)
        self.roadmap_vis_pub = rospy.Publisher('/roadmap_vis', MarkerArray, queue_size=10)
        self.roadmap_pub = rospy.Publisher('/roadmap', RoadMap, queue_size=10)
        self.frontier_vis_pub = rospy.Publisher('/centroid_markers', MarkerArray, queue_size=10)
        # self.regions_pub = rospy.Publisher('/regions', MarkerArray, queue_size=10)
        # Waiting for servives
        self.occup_map = rospy.wait_for_message(self.map_topic, OccupancyGrid)

        
        while not rospy.is_shutdown():
            self.voronoi_pub.publish(self._voronoi_map_msg)
            self.roadmap_pub.publish(self._roadmap_msg)
            self.roadmap_vis_pub.publish(self._roadmap_vis_msg)
            self.frontier_vis_pub.publish(self._centroid_marker_msg)
            rospy.sleep(1)

    @measure_time
    def map_callback(self, msg: OccupancyGrid):
        if msg.data == self.occup_map.data:
            rospy.loginfo("Roadmap Builder: map not changed")
            return
        else:
            rospy.loginfo("Roadmap Builder: occupancy map changed")
        self.occup_map = msg
        # self.filter_map = self.filter_occup_map(self.occup_map)
        self.filter_map = self.occup_map
        
        try:
            voronoi_res = self.voronoi_client.call(GenVoronoiRequest(self.filter_map))
            self._voronoi_map_msg = voronoi_res.voronoi_map
            # self.voronoi_pub.publish(voronoi_res.voronoi_map)
        except rospy.service.ServiceException as e:
            rospy.logerr(f"Voronoi service unavailable: {e}")
            return

        frontiers = []
        try:
            frontier_res = self.frontier_client.call(GetFrontiersRequest(self.filter_map))
            frontiers = frontier_res.frontiers
            self._centroid_marker_msg = _set_frontier_marker_msg(frontiers)
        except rospy.service.ServiceException as e:
            rospy.logwarn(f"Frontier service unavailable: {e}")
            # Continue processing without frontiers

        # Process of voronoi map
        self.roadmap = self.build_roadmap(voronoi_res.voronoi_map, frontiers)
        self._roadmap_msg = self.roadmap2RoadMap(self.roadmap)
        self._roadmap_vis_msg = self.roadmap2MarkerArray(self.roadmap)

        # self.roadmap_vis_pub.publish(roadmap_vis_msg)
        rospy.loginfo("Roadmap Builder: map callback done. seq: %s", msg.header.seq)
        

    def filter_occup_map(self, occup_map, num=8):
        """
        Filter the occupancy map with sense noise.
        """
        filter_map = copy.deepcopy(occup_map)
        # res = occup_map.info.resolution
        width = filter_map.info.width
        height = filter_map.info.height
        for x in range(width):
            for y in range(height):
                id = y*width + x
                value = filter_map.data[id]
                if filter_map.data[id] == -1:
                    for nx, ny in self.neighbors((x,y), num):
                        if nx in range(width) and ny in range(height):
                            nd = ny*width + nx
                            filter_map.data[nd] = 0  
                            value += filter_map.data[nd]
                    average = value/num
                    if average > -8:
                        filter_map.data[id] == 0 
        return filter_map

    def neighbors(self, node, num=8):
        """
        Get neighbors of the node.
        """
        neighbors_edge = [
            (1, 0), (0, 1), (-1, 0), (0, -1),
            (1, 1), (1, -1), (-1, 1), (-1, -1),
        ]
        neighbors = []
        for neigh in neighbors_edge:
            neighbors.append((node[0]+neigh[0], node[1]+neigh[1]))
        return neighbors

    def build_roadmap(self, voronoi, frontiers, safe_dist=2):
        """
        Build roadmap during the map exploration.
        """
        roadmap = nx.Graph()
        origin_pos = voronoi.origin.position  # Type: geometery_msgs/Point

        # Add edges and nodes from Voronoi Diagram
        for segment in voronoi.vertices:
            # Generate node of point_s
            x_s = round(segment.path[0].x+origin_pos.x, 2)
            y_s = round(segment.path[0].y+origin_pos.y, 2)
            point_s = (x_s, y_s)
            roadmap.add_node(point_s, label=['vertex'])
            # Generate node of point_t
            x_t = round(segment.path[-1].x+origin_pos.x, 2)
            y_t = round(segment.path[-1].y+origin_pos.y, 2)
            point_t = (x_t, y_t)
            roadmap.add_node(point_t, label=['vertex'])
            # Add the edge between point_s and point_t
            roadmap.add_edge(point_s, point_t, dist=get_dist(point_s, point_t))

        roadmap = self.connect_roadmap(roadmap)
        # Add frontiers into roadmap
        # for frontier in frontiers:
        #     centroid_node = (round(frontier.centroid.x, 2), round(frontier.centroid.y, 2))
        #     for node in self.find_close_nodes(roadmap, centroid_node):
        #         roadmap.add_node(centroid_node, label='frontier')
        #         roadmap.add_edge(centroid_node, node, cost=get_dist(centroid_node, node))
        return roadmap

    def connect_roadmap(self, roadmap):
        """
        Connect the roadmap to make it connected.
        """
        while not nx.is_connected(roadmap):
            # Find connected components
            components = list(nx.connected_components(roadmap))
            closest_edges = []
            for comp_i, comp_j in combinations(components, 2):
                closest_edge = (np.inf, None, None)
                # Find the closest edge
                for node_i in comp_i:
                    for node_j in comp_j:
                        if get_dist(node_i, node_j) < closest_edge[0]:
                            # check if the edge is corss with obstacles
                            if not self.if_cross_with_obstacles(node_i, node_j):
                                closest_edge = (get_dist(node_i, node_j), node_i, node_j)
                closest_edges.append(closest_edge)
            # Add closest edge
            for dist, node_i, node_j in closest_edges:
                roadmap.add_edge(node_i, node_j, dist=dist)
        return roadmap

    def if_cross_with_obstacles(self, node_s, node_t):
        """
        Check if the line between source and target node is cross with obstacles.
        """
        return False


    def roadmap2MarkerArray(self, roadmap: nx.Graph):
        """
        Transfer networkx.Graph to MarkerArray message.
        """
        marker_array = MarkerArray()
        for i, node in enumerate(roadmap.nodes()):
            if 'vertex' in roadmap.nodes[node]['label']:
                blue_index = [1.0, 0.0, 0.0, 1.0]
                marker = create_point_marker('vertex', i, node,
                                            color=blue_index, scale=0.1)
                marker_array.markers.append(marker)
            if 'frontier' in roadmap.nodes[node]['label']:
                red_index = [1.0, 1.0, 0.0, 0.0]
                marker = create_point_marker('frontier', i, node,
                                            color=red_index, scale=0.1)
                marker_array.markers.append(marker)            
        ## publish edges
        for i, edge in enumerate(roadmap.edges()):
            blue_index = [1.0, 0.0, 0.0, 1.0]
            marker = create_line_marker('edge', i, edge,
                                        color=blue_index, scale=0.02)
            marker_array.markers.append(marker)
        return marker_array

    def roadmap2RoadMap(self, roadmap: nx.Graph):
        """
        Transfer networkx.Graph to RoadMap message.
        """
        roadmap_msg = RoadMap()
        roadmap_msg.stamp = rospy.Time.now()
        for node in roadmap.nodes:
            node_msg = RoadMapNode()
            node_msg.point.x = node[0]
            node_msg.point.y = node[1]
            node_msg.labels = roadmap.nodes[node]['label']
            roadmap_msg.nodes.append(node_msg)
        for edge in roadmap.edges:
            edge_msg = RoadMapEdge()
            edge_msg.source.x = edge[0][0]
            edge_msg.source.y = edge[0][1]
            edge_msg.target.x = edge[1][0]
            edge_msg.target.y = edge[1][1]
            edge_msg.dist = roadmap.edges[edge]['dist']
            edge_msg.label = 'hard'
            roadmap_msg.edges.append(edge_msg)
        return roadmap_msg

    def find_nearest_node(self, roadmap, point):
        """
        Get the nearest node in roadmap to given point.
        ----------
        Paramaters:
            roadmap (networkx.Graph): roadmap in networkx.Graph form
            point (geometery/Point): the given point
        
        Returns:
            ((near_point_x, near_point_y), min_dis)
        """
        nodes_ = list(roadmap.nodes)
        min_index_ = 0
        min_dis_ = math.inf
        for i in range(len(nodes_)):
            dis_ = get_dist(point, nodes_[i])
            if dis_ < min_dis_:
                min_dis_ = dis_
                min_index_ = i
        return nodes_[min_index_], min_dis_

    def find_close_nodes(self, roadmap, point, close_dist=1.0):
        """
        Find all nodes within close_dist.
        """
        close_nodes = []
        nodes = list(roadmap.nodes)
        for node in nodes:
            if get_dist(point, node) <= close_dist:
                close_nodes.append(node)
                print(point, node)
        # if not close_nodes:
        #     close_node, _ = self.find_nearest_node(roadmap, point)
        #     close_nodes = [close_node]
        return close_nodes

    def display_roadmap(self, roadmap: nx.Graph):
        option = {
            "with_labels": True, 
            "font_weight": 'bold', 
            "node_color": ["blue" if roadmap.nodes[n]['label']=="raw" else "red" for n in roadmap.nodes],
            "pos": {n: n for n in list(roadmap.nodes)}
            }
        nx.draw(roadmap, **option)
        plt.show()


if __name__ == "__main__":
    try:
        rbn = RoadmapBuilderNode()
    except rospy.ROSInterruptException:
        pass