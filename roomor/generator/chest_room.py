import numpy as np

from randoor.generator import ChestSearchRoomGenerator
from shapely.geometry.polygon import Polygon

from ..room_generator_factory import RoomGeneratorFactory, RoomConfig

class ChestRoomConfig(RoomConfig):
    
    def __init__(self, 
                 model_manager,
                 randoor_config,
                 wall_polygon, 
                 wall_thickness,
                 wall_height,
                 obstacle_size, 
                 target_size,
                 key_size
    ):
        
        super(ChestRoomConfig, self).__init__(model_manager, randoor_config)

        self.randoor_config = randoor_config
        
        self.wall_polygon = wall_polygon
        self.wall_thickness = wall_thickness
        self.wall_height = wall_height
        self.obstacle_size = obstacle_size
        self.target_size = target_size
        self.key_size = key_size
        
        self.wall_tag = 'wall'
        self.obstacle_tag = 'obstacle'
        self.target_tag = 'target'
        self.key_tag = 'key'
        
        self.wall_color = 'black'
        self.obstacle_color = 'white'
        self.target_color = 'cyan'
        self.key_color = 'yellow'
        
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

        self.key_config_base = [
            dict(
                type='box',
                args=dict(
                    size=key_size,
                    name=self.key_tag,
                    color=self.key_color
                )
            )
        ]

    @property
    def target_pose(self):
        return self.spawn_config[self.target_tag]

    @property
    def key_pose(self):
        return self.spawn_config[self.key_tag]
    
    @property
    def wall_pose(self):
        return self.spawn_config[self.wall_tag]
    
    @property
    def obstacle_pose(self):
        return self.spawn_config[self.obstacle_tag]
        
    def prepare_model_manager(self, max_obstacle_count, max_target_count, max_key_count):
        self.register_empty(self.wall_tag, self.wall_config_base, 1)
        self.set_modelspace_force(self.wall_tag)
        
        self.register_empty(self.obstacle_tag, self.obstacle_config_base, max_obstacle_count)
        self.set_modelspace(self.obstacle_tag)
        
        self.register_empty(self.target_tag, self.target_config_base, max_target_count)
        self.set_modelspace(self.target_tag, disable_collision=True)

        self.register_empty(self.key_tag, self.key_config_base, max_key_count)
        self.set_modelspace(self.key_tag, disable_collision=True)

    def set_components_pose(self, obstacle_poss, obstacle_oris, target_poss, target_oris, key_poss, key_oris):
        self.register_positions(self.wall_tag, [[0,0,self.wall_height/2]])
        self.register_positions(self.obstacle_tag, obstacle_poss)
        self.register_orientations(self.obstacle_tag, obstacle_oris)
        self.register_positions(self.target_tag, target_poss)
        self.register_orientations(self.target_tag, target_oris)
        self.register_positions(self.key_tag, key_poss)
        self.register_orientations(self.key_tag, key_oris)
            
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

    @property
    def key_models(self):
        c = len(self.spawn_config[self.key_tag]['positions'])
        if self.model_manager.is_set_modelspace(self.key_tag):
            return self.model_manager.get_base_models(self.key_tag, c)
        else:
            return None
        
    def spawn_all(self):
        self.set_modelspace_force(self.wall_tag)
        super(ChestRoomConfig, self).spawn_all()
        
    def get_freespace_poly(self):
        return self.randoor_config.get_freespace_poly()
    
    def get_freezone_poly(self):
        return self.randoor_config.get_freezone_poly()
        
class ChestRoomGenerator(RoomGeneratorFactory):
    
    def __init__(self,
                 obstacle_count=10,
                 obstacle_size=0.7,
                 target_size=0.2,
                 key_size=0.2,
                 obstacle_zone_thresh=1.5,
                 distance_key_placing=0.7,
                 range_key_placing=0.3,
                 room_length_max=9,
                 room_wall_thickness=0.05,
                 wall_threshold=0.1,
                 room_wall_height=0.8,
                 ros_host="localhost", 
                 ros_port=11311, 
                 gazebo_host='localhost', 
                 gazebo_port=11345):
        
        super(ChestRoomGenerator, self).__init__(ros_host, ros_port, gazebo_host, gazebo_port)
        
        ## randoor SimpleSearchRoomGenerator's parameter ##
        self.obstacle_count = obstacle_count
        self.obstacle_size = obstacle_size
        self.target_size = target_size
        self.key_size = key_size
        self.obstacle_zone_thresh = obstacle_zone_thresh
        self.distance_key_placing = distance_key_placing
        self.range_key_placing = range_key_placing
        self.room_length_max = room_length_max
        self.room_wall_thickness = room_wall_thickness
        self.wall_threshold = wall_threshold
        ###################################################

        self.randoor_generator = ChestSearchRoomGenerator(
            obstacle_count=obstacle_count,
            obstacle_size=obstacle_size,
            target_size=target_size, 
            key_size=key_size,
            obstacle_zone_thresh=obstacle_zone_thresh,
            distance_key_placing=distance_key_placing,
            range_key_placing=range_key_placing,
            room_length_max=room_length_max,
            room_wall_thickness=room_wall_thickness,
            wall_threshold=wall_threshold
        )

        self.room_wall_height = room_wall_height
    
    def generate_new(self):
        randoor_config = self.randoor_generator.generate_new()
        
        ## get base shape from wall polygon
        wall_poly = randoor_config.get_polygons(randoor_config.tag_wall)[0]
        wall_base = Polygon(wall_poly.exterior.coords).buffer(-self.room_wall_thickness, cap_style=3, join_style=2)

        room_instance = ChestRoomConfig(
                model_manager=self.model_manager,
                randoor_config=randoor_config,
                wall_polygon=wall_base, 
                wall_thickness=self.room_wall_thickness,
                wall_height=self.room_wall_height,
                obstacle_size=[self.obstacle_size for _ in range(3)], 
                target_size=[self.target_size for _ in range(3)],
                key_size=[self.key_size for _ in range(3)]
        )
        
        room_instance.prepare_model_manager(self.obstacle_count, self.obstacle_count, self.obstacle_count)
        
        obstacle_xyy = np.array(randoor_config.get_positions(randoor_config.tag_obstacle))
        obstacle_poss = np.empty([len(obstacle_xyy), 3])
        obstacle_poss[:,:2] = obstacle_xyy[:,:2]
        obstacle_poss[:,2] = self.obstacle_size/2
        obstacle_oris = np.zeros_like(obstacle_poss)
        obstacle_oris[:,2] = obstacle_xyy[:,2]

        target_xyy = np.array(randoor_config.get_positions(randoor_config.tag_target))
        target_poss = np.empty([len(target_xyy), 3])
        target_poss[:,:2] = target_xyy[:,:2]
        target_poss[:,2] = self.target_size/2
        target_oris = np.zeros_like(target_poss)
        target_oris[:,2] = target_xyy[:,2]

        key_xyy = np.array(randoor_config.get_positions(randoor_config.tag_key))
        key_poss = np.empty([len(key_xyy), 3])
        key_poss[:,:2] = key_xyy[:,:2]
        key_poss[:,2] = self.key_size/2
        key_oris = np.zeros_like(key_poss)
        key_oris[:,2] = key_xyy[:,2]

        room_instance.set_components_pose(
            obstacle_poss=obstacle_poss, 
            obstacle_oris=obstacle_oris, 
            target_poss=target_poss,
            target_oris=target_oris,
            key_poss=key_poss,
            key_oris=key_oris
        )
        
        return room_instance

    def reposition_target(self, room_config):
        randoor_config = room_config.randoor_config
        self.randoor_generator.reposition_target(randoor_config)

        target_xyy = np.array(randoor_config.get_positions(randoor_config.tag_target))
        target_poss = np.empty([len(target_xyy), 3])
        target_poss[:,:2] = target_xyy[:,:2]
        target_poss[:,2] = self.target_size/2
        target_oris = np.zeros_like(target_poss)
        target_oris[:,2] = target_xyy[:,2]

        room_config.register_positions(room_config.target_tag, target_poss)
        room_config.register_orientations(room_config.target_tag, target_oris)
        room_config.apply(room_config.target_tag)

    def reposition_key(self, room_config):
        randoor_config = room_config.randoor_config
        self.randoor_generator.reposition_key(randoor_config)

        key_xyy = np.array(randoor_config.get_positions(randoor_config.tag_key))
        key_poss = np.empty([len(key_xyy), 3])
        key_poss[:,:2] = key_xyy[:,:2]
        key_poss[:,2] = self.key_size/2
        key_oris = np.zeros_like(key_poss)
        key_oris[:,2] = key_xyy[:,2]

        room_config.register_positions(room_config.key_tag, key_poss)
        room_config.register_orientations(room_config.key_tag, key_oris)
        room_config.apply(room_config.key_tag)
