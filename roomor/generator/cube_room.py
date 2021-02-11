import numpy as np
import shapely
from shapely.geometry.polygon import Polygon
from sklearn.cluster import DBSCAN
import trimesh
import copy
from pcg_gazebo.generators.shapes import random_points_to_triangulation

from ..geometric_util import get_corrected_poly_with_model, get_square_horizon, add_dimension, get_extended_face
from ..room_generator_factory import RoomGeneratorFactory, RoomConfig

class CubeRoomConfig(RoomConfig):
    
    def __init__(self, 
                 model_manager,
                 wall_polygon, 
                 wall_thickness,
                 wall_height,
                 obstacle_size, 
                 target_size,
    ):
        
        super(CubeRoomConfig, self).__init__(model_manager)
        
        self.wall_polygon = wall_polygon
        self.wall_thickness = wall_thickness
        self.wall_height = wall_height
        self.obstacle_size = obstacle_size
        self.target_size = target_size
        
        self.wall_tag = 'wall'
        self.obstacle_tag = 'obstacle'
        self.target_tag = 'target'
        
        self.wall_color = 'black'
        self.obstacle_color = 'white'
        self.target_color = 'cyan'
        
        self.wall_config_base = [
            dict(
                type='extrude',
                args=dict(
                    polygon=wall_polygon,
                    height=wall_height,
                    thickness=wall_thickness,
                    cap_style='square',
                    join_style='mitre',
                    extrude_boundaries=True,
                    name=self.wall_tag,
                    color=self.wall_color
                )
            )
        ]
        
        self.obstacle_config_base = [
            dict(
                type='box',
                args=dict(
                    size=obstacle_size,
                    name=self.obstacle_tag,
                    color=self.obstacle_color
                )
            )
        ]
        
        self.target_config_base = [
            dict(
                type='box',
                args=dict(
                    size=target_size,
                    name=self.target_tag,
                    color=self.target_color
                )
            )
        ]
        
        self.wall_collision_polygon = None
        self.wall_interior_polygon = None
        self.wall_exterior_polygon = None
        
        self.ozpoints_tag = 'obstacle_zone_points'
        self.ozpolys_tag = 'obstacle_zone_polys'
        self.ozhulls_tag = 'obstacle_zone_hulls'
        self.point_group[self.ozpoints_tag] = list()
        self.polygon_group[self.ozpolys_tag] = list()
        self.polygon_group[self.ozhulls_tag] = list()

    @property
    def target_pose(self):
        return self.spawn_config[self.target_tag]
    
    @property
    def wall_pose(self):
        return self.spawn_config[self.wall_tag]
    
    @property
    def obstacle_pose(self):
        return self.spawn_config[self.obstacle_tag]
        
    def prepare_model_manager(self, max_obstacle_count, max_target_count):
        self.register_empty(self.wall_tag, self.wall_config_base, 1)
        self.set_modelspace_force(self.wall_tag)
        
        self.register_empty(self.obstacle_tag, self.obstacle_config_base, max_obstacle_count)
        self.set_modelspace(self.obstacle_tag)
        
        self.register_empty(self.target_tag, self.target_config_base, max_target_count)
        self.set_modelspace(self.target_tag, disable_collision=True)
        
        self.set_wall_collision_polys()
        
    def set_wall_collision_polys(self):
        exterior = self.wall_polygon.buffer(self.wall_thickness/2, join_style=2) ## mitre style
        self.wall_exterior_polygon = get_corrected_poly_with_model(exterior, self.wall_model)
        
        self.wall_interior_polygon = self.wall_exterior_polygon.buffer(-1*self.wall_thickness, join_style=2) ## mitre style
        self.wall_collision_polygon = Polygon(
            list(self.wall_exterior_polygon.exterior.coords), 
            [list(self.wall_interior_polygon.exterior.coords)]
        )
        
    def set_zone_group(self, obstacle_zone_points, obstacle_zone_polys, obstacle_zone_hulls):
        self.point_group[self.ozpoints_tag] = obstacle_zone_points
        self.polygon_group[self.ozpolys_tag] = obstacle_zone_polys
        self.polygon_group[self.ozhulls_tag] = obstacle_zone_hulls

    def set_components_pose(self, obstacle_poss, obstacle_oris, target_poss, target_oris):
        self.register_positions(self.wall_tag, [[0,0,self.wall_height/2]])
        self.register_positions(self.obstacle_tag, obstacle_poss)
        self.register_orientations(self.obstacle_tag, obstacle_oris)
        self.register_positions(self.target_tag, target_poss)
        self.register_orientations(self.target_tag, target_oris)
            
    @property
    def wall_model(self):
        if self.model_manager.is_set_modelspace(self.wall_tag):
            return self.model_manager.get_base_models(self.wall_tag, 1)[0]
        else:
            return None
    
    @property
    def obstacle_models(self):
        c = len(self.spawn_config[self.obstacle_tag]['positions'])
        if self.model_manager.is_set_modelspace(self.obstacle_tag):
            return self.model_manager.get_base_models(self.obstacle_tag, c)
        else:
            return None
        
    @property
    def target_models(self):
        c = len(self.spawn_config[self.target_tag]['positions'])
        if self.model_manager.is_set_modelspace(self.target_tag):
            return self.model_manager.get_base_models(self.target_tag, c)
        else:
            return None
        
    def spawn_all(self):
        self.set_modelspace_force(self.wall_tag)
        super(CubeRoomConfig, self).spawn_all()
        
    def get_freespace_poly(self):
        all_poly = []
        for polys in self.polygon_group[self.ozpolys_tag]:
            if isinstance(polys, Polygon):
                all_poly.append(polys)
            else:
                for p in polys:
                    all_poly.append(p)

        map_poly = Polygon(
            list(self.wall_interior_polygon.exterior.coords), 
            list(map(lambda p: list(p.exterior.coords), all_poly))
        )
        return map_poly
    
    def get_freezone_poly(self):
#         zones = [Polygon(points) for points in self.point_group[self.ozpoints_tag]]
        zones_union = shapely.ops.unary_union([Polygon(points) for points in self.point_group[self.ozpoints_tag]])
        map_poly = Polygon(
            list(self.wall_interior_polygon.exterior.coords), 
            [z.exterior.coords for z in zones_union]
        )
        return map_poly
        
class CubeRoomGenerator(RoomGeneratorFactory):
    
    def __init__(self,
                 obstacle_count=10,
                 obstacle_size=0.7,
                 agent_size=0.3,
                 target_size=0.2,
                 room_length_max=9,
                 room_mass_min=20,
                 room_mass_max=36,
                 room_wall_height=0.8,
                 room_wall_thickness=0.05,
                 wall_threshold=0.1,
                 ros_host="localhost", ros_port=11311, gazebo_host='localhost', gazebo_port=11345):
        
        super(CubeRoomGenerator, self).__init__(ros_host, ros_port, gazebo_host, gazebo_port)
        
        self.obstacle_count = obstacle_count
        self.obstacle_size = obstacle_size
        self.agent_size = agent_size
        self.target_size = target_size
        self.room_length_max = room_length_max
        self.room_mass_min = room_mass_min
        self.room_mass_max = room_mass_max
        self.room_wall_height = room_wall_height
        self.room_wall_thickness = room_wall_thickness
        self.wall_threshold = wall_threshold
        
    def _create_wall_poly(self):
        while True:
            poly = random_points_to_triangulation(
                x_min=-self.room_length_max/2,
                x_max=self.room_length_max/2,
                y_min=-self.room_length_max/2,
                y_max=self.room_length_max/2
            )
            area = poly.area
            if self.room_mass_max >= area and area >= self.room_mass_min:
                return poly
    
    def generate_new(self):
        wall_base = self._create_wall_poly()
        
        room_instance = CubeRoomConfig(
                model_manager=self.model_manager,
                wall_polygon=wall_base, 
                wall_thickness=self.room_wall_thickness,
                wall_height=self.room_wall_height,
                obstacle_size=[self.obstacle_size for _ in range(3)], 
                target_size=[self.target_size for _ in range(3)]
        )
        
        room_instance.prepare_model_manager(self.obstacle_count, self.obstacle_count)
        
#         wall_model = room_instance.wall_model
        
#         wall_mesh = get_corrected_mesh_from_model(wall_model) #-
#         wall_poly = get_poly_from_model(wall_model) #-
#         wall_interior_poly = get_interior_poly_from_extrude_mesh(wall_mesh, self.room_wall_height/2) #-
        
        obstacle_poss, obstacle_oris = self.generate_obstacles_pose(room_instance.wall_interior_polygon)
        
        obstacle_label = self.get_cluster(np.sqrt((self.obstacle_size**2)*2)+self.agent_size/2, obstacle_poss)
        
        obstacle_zone_points, obstacle_zone_polys, obstacle_zone_hulls = self.create_zones_with_obstacle(
            obstacle_label, obstacle_poss, obstacle_oris
        )
        
        target_poss, target_oris = self.generate_targets_pose(obstacle_zone_hulls)
        
        room_instance.set_zone_group(obstacle_zone_points, obstacle_zone_polys, obstacle_zone_hulls)
        room_instance.set_components_pose(
            obstacle_poss=obstacle_poss, 
            obstacle_oris=obstacle_oris, 
            target_poss=target_poss,
            target_oris=target_oris
        )
        
        return room_instance
        
    def generate_obstacles_pose(self, space_poly):
        sample_area = space_poly.buffer(-1*(self.wall_threshold+self.obstacle_size)) #+
        pos = trimesh.path.polygons.sample(sample_area, self.obstacle_count) #-
        pos = add_dimension(pos, self.obstacle_size/2) #-
        
        ori = np.zeros_like(pos)
        ori[:,2] = np.random.random([len(pos)])*np.pi*2
        
        return pos, ori
    
    def generate_targets_pose(self, zone_hulls):
        zone_samples = [None]*len(zone_hulls)
        for i in range(len(zone_samples)):
            ## sample_surface return tuple, so choice 0 index
            samples = trimesh.sample.sample_surface_even(zone_hulls[i], 100)[0]
            zone_samples[i] = samples[
                np.logical_and(
                    np.not_equal(samples[:,2], 0), 
                    np.not_equal(samples[:,2], self.obstacle_size)
                )
            ]

        choiced_goal = np.zeros([len(zone_hulls),3])
        for i in range(len(choiced_goal)):
            choiced_goal[i] = zone_samples[i][np.random.choice(range(len(zone_samples[i])))]

        choiced_goal[:,2] = self.target_size/2
        
        return choiced_goal, np.zeros_like(choiced_goal)
        
    def create_zones_with_obstacle(self, label, positions, orientations):
        cls_count = max(label)+1
        
        zone_vertices_2d = [None]*cls_count
        zone_polys = [None]*cls_count
        for i in range(cls_count):
            pts = positions[label==i]
            rts = orientations[label==i]
            vertices = [get_square_horizon(p, self.obstacle_size/2, r[2]) for p,r in zip(pts,rts)] #-
            vertices_all = np.concatenate(vertices)

            zone_vertices_2d[i] = trimesh.convex.hull_points(vertices_all[:,:2])
            zone_polys[i] = shapely.ops.unary_union([Polygon(v) for v in vertices])
            
        zone_hulls = [None]*cls_count
        for i in range(cls_count):
            p3 = np.zeros([len(zone_vertices_2d[i]),3])
            p3[:,:2] = zone_vertices_2d[i]
            zone_hulls[i] = trimesh.Trimesh(vertices=get_extended_face(p3, self.obstacle_size)).convex_hull #-
            
        return zone_vertices_2d, zone_polys, zone_hulls
        
        
    def get_cluster(self, eps, points): ## eps; np.sqrt((OBSTACLE_SIZE**2)*2)+AGENT_SIZE/2
        db = DBSCAN(eps=eps, min_samples=1).fit(np.array(points))
        return db.labels_