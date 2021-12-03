import abc
import numpy as np
from pcg_gazebo.task_manager import GazeboProxy

from .model_manager import ModelManager
            
class RoomGeneratorFactory(object):

    def __init__(self, ros_host="localhost", ros_port=11311, gazebo_host='localhost', gazebo_port=11345):
        self.gazebo_proxy = GazeboProxy(
            ros_host=ros_host,
            ros_port=ros_port,
            gazebo_host=gazebo_host,
            gazebo_port=gazebo_port
        )
        self.model_manager = ModelManager(self.gazebo_proxy)
        self.randoor_generator = None
        
    @abc.abstractmethod
    def generate_new(self): ## return RoomConfig
        pass
        
        
class RoomConfig(object):
    
    def __init__(self, model_manager, randoor_config):
        self.model_manager = model_manager
        self.config_tags = list()
        self.spawn_config = dict() ## 'tag': {'config_base': ~, 'positions': ~, 'orientations': ~}
        self.randoor_config = randoor_config
        
    @abc.abstractmethod
    def prepare_model_manager(self):
        pass
    
    def register_empty(self, tag, config_base, count):
        self.config_tags.append(tag)
        self.spawn_config[tag] = dict(
            config_base=config_base,
            positions=np.zeros([count,3]),
            orientations=np.zeros([count,3])
        )
    
    def register_positions(self, tag, positions):
        self.spawn_config[tag]['positions'] = positions
        
    def register_orientations(self, tag, orientations):
        self.spawn_config[tag]['orientations'] = orientations
    
    def set_modelspace_force(self, tag, disable_collision=False):
        self.model_manager.set_modelspace_from_config(
            tag, 
            self.spawn_config[tag]['config_base'], 
            len(self.spawn_config[tag]['positions']),
            disable_collision=disable_collision
        )
    
    def set_modelspace(self, tag, disable_collision=False):
        if not self.model_manager.is_set_modelspace(tag):
            self.set_modelspace_force(tag, disable_collision)
        
    def spawn_all(self):
        for t in self.spawn_config.keys():
            self.apply(t)

    def apply(self, tag):
        self.model_manager.apply_model(tag, self.spawn_config[tag]['positions'], self.spawn_config[tag]['orientations'])
    
    def _get_all_moved_models(self, exclude_tags=[None]):
        moved = [
            self.model_manager.get_moved_models(
                tag, 
                self.spawn_config[tag]['positions'],
                self.spawn_config[tag]['orientations']
            ) for tag in self.config_tags if not tag in exclude_tags
        ]
        models = {}
        for ms in moved:
            for m in ms:
                models[m.name] = m
                
        return models
    
    def get_freespace_poly(self):
        return self.randoor_config.get_freespace_poly()
        
    def get_occupancy_grid(self, freespace_poly, origin_pos=(0,0), origin_ori=0, resolution=0.050, map_size=512):
        return self.randoor_config.get_occupancy_grid(freespace_poly, origin_pos, origin_ori, resolution, map_size)