import abc

import numpy as np
import shapely
import trimesh
from shapely.geometry import Polygon, Point

from multiprocessing import Pool, Array
from contextlib import closing

from pcg_gazebo.task_manager import GazeboProxy
from pcg_gazebo.simulation.world import World
from pcg_gazebo.simulation.model import SimulationModel

from pcg_gazebo.generators.occupancy import generate_occupancy_grid
from pcg_gazebo.visualization import plot_occupancy_grid
import io

from .model_manager import ModelManager
from .geometric_util import vec_to_trans
            
class RoomGeneratorFactory(object):

    def __init__(self, ros_host="localhost", ros_port=11311, gazebo_host='localhost', gazebo_port=11345):
        self.gazebo_proxy = GazeboProxy(
            ros_host=ros_host,
            ros_port=ros_port,
            gazebo_host=gazebo_host,
            gazebo_port=gazebo_port
        )
        self.model_manager = ModelManager(self.gazebo_proxy)
        
    @abc.abstractmethod
    def generate_new(self): ## return RoomConfig
        pass
        
        
class RoomConfig(object):
    
    def __init__(self, model_manager):
        self.model_manager = model_manager
        self.spawn_config = dict() ## 'tag': {'config_base': ~, 'positions': ~, 'orientations': ~}
        self.config_tags = list()
        
        self.point_group = dict()
        self.polygon_group = dict()
        self.mesh_group = dict()
        
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
            self.model_manager.apply_model(t, self.spawn_config[t]['positions'], self.spawn_config[t]['orientations'])
    
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
    
    @abc.abstractmethod
    def get_freespace_poly(self, exterior_wall_tag, exclude_tags=[None]):
        wall_name = self.model_manager.get_base_models(exterior_wall_tag, 1)[0].name
        
        models = self._get_all_moved_models(exclude_tags=exclude_tags)
        models['ground_plane'] = SimulationModel.from_gazebo_model('ground_plane')
        
        occ_poly = generate_occupancy_grid(models, ground_plane_models=[wall_name])
        
        space = Polygon(occ_poly['static']['ground_plane_models'].interiors[0])

        ex = ['ground_plane', 'ground_plane_models']
        interior_polys = [occ_poly['static'][name] for name in occ_poly['static'].keys() if not name in ex]
        union_poly = shapely.ops.unary_union(interior_polys)
        
        return Polygon(list(space.exterior.coords), [list(p.exterior.coords) for p in union_poly])
        
    @abc.abstractmethod
    def get_occupancy_grid(self, freespace_poly, origin_pos=(0,0), origin_ori=0, resolution=0.050, map_size=512):
#         wall_name = self.model_manager.get_base_models(exterior_wall_tag, 1)[0].name
        
#         models = self._get_all_moved_models(exclude_tags=exclude_tags)
#         models['ground_plane'] = SimulationModel.from_gazebo_model('ground_plane')        
        
#         fig = plot_occupancy_grid(
#             models,
#             with_ground_plane=True,
#             static_models_only=False,
#             exclude_contains=['ground_plane'],
#             ground_plane_models=[wall_name]
#         )

#         buf = io.BytesIO()
#         fig.savefig(buf, format='png')
#         enc = np.frombuffer(buf.getvalue(), dtype=np.uint8)
#         dst = cv2.imdecode(enc, cv2.IMREAD_GRAYSCALE)
        
        if origin_pos == (0,0) and origin_ori == 0:
            corrected = freespace_poly
        else:
            mat1 = vec_to_trans(origin_pos)
            mat2 = np.array([
                [np.cos(origin_ori), -np.sin(origin_ori), 0],
                [np.sin(origin_ori), np.cos(origin_ori), 0],
                [0, 0, 1]
            ])
            corrected = trimesh.path.polygons.transform_polygon(np.dot(mat2, mat1), freespace_poly)
    
        half_length = (map_size * resolution) // 2
        lin = np.linspace(-half_length, half_length, map_size)
        xx, yy = np.meshgrid(lin, lin)
        xc = xx.flatten()
        yc = yy.flatten()
        
        data = np.full([map_size*map_size], 0)

        xl = Array('d', xc)
        yl = Array('d', yc)
        
        global mu

        def mu(i):
            return corrected.contains(Point(xl[i], yl[i]))

        with closing(Pool()) as pool:
            data[pool.map(mu, range(len(xc)))] = 255
    
        return data
        