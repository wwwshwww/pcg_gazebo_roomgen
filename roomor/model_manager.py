import copy
from pcg_gazebo.simulation.properties.pose import Pose
from pcg_gazebo.generators.creators import create_models_from_config
from multiprocessing import Process

class ModelManager(): ## orientation: [roll, pitch, yaw] (degrees) or [qx, qy, qz, qw] (quaternion)
    
    namespace = 0
    
    def __init__(self, gazebo_proxy):
        self.gazebo_proxy = gazebo_proxy
        self.namespace = ModelManager.namespace
        ModelManager.namespace += 1
        self.configspaces = dict()
        self.modelspaces = dict()
    
    def is_set_modelspace(self, tag):
        return tag in self.modelspaces.keys()
        
    def set_modelspace_from_config(self, tag, config, max_model_count, disable_collision=False):
        configs = [copy.deepcopy(config)[0] for _ in range(max_model_count)]
        for i in range(max_model_count):
            configs[i]['args']['name'] = "mm-{}-{}_{}".format(self.namespace, tag, i)
        
        self.configspaces[tag] = configs
        models = create_models_from_config(configs)
        
        if disable_collision:
            for m in models:
                m.get_link_by_name('link').disable_collision()
        
        self.modelspaces[tag] = models
        
    def set_modelspace_from_models(self, tag, models):
        for i in range(len(models)):
            models[i].name = "mm-{}-{}_{}".format(self.namespace, tag, i)
            self.modelspaces[tag][i] = models[i]
        
    def get_moved_models(self, tag, positions, orientations):
        copy_models = copy.deepcopy(self.modelspaces[tag][:len(positions)])
        
        poses = [Pose(pos=p, rot=o) for p,o in zip(positions, orientations)]
        for i in range(len(positions)):
            copy_models[i].pose += poses[i]
        
        return copy_models
    
    def get_base_models(self, tag, count):
        return self.modelspaces[tag][0:count]
        
    def get_base_model_poses(self, tag, count):
        return [self.modelspaces[tag][i].pose for i in range(count)]
            
    def apply_model(self, tag, positions, orientations):
        self._delete_other_models()
        self._delete_models(tag, len(positions), len(self.modelspaces[tag]))
        
        spawn_f = [None] * len(positions)
        spawn_kwargs = [None] * len(positions)
        
        for i in range(len(spawn_f)):
            spawn_f[i] = self.modelspaces[tag][i].spawn
            spawn_kwargs[i] = dict(
                gazebo_proxy=self.gazebo_proxy, 
                robot_namespace=self.modelspaces[tag][i].name,
                pos=list(positions[i]),
                rot=list(orientations[i])
            )
        self._multiprocessing(spawn_f, spawn_kwargs)

    def _is_mine_model(self, model_name):
        namespace = model_name.split("-")[1]
        return ModelManager._is_manager_model(model_name) and namespace == str(self.namespace)
    
    @classmethod
    def _is_manager_model(cls, model_name):
        name = model_name.split("-")
        return name[0] == "mm" and str.isdigit(name[1])

    def _delete_models(self, tag, start, stop):     
        del_kwargs = [dict(model_name="mm-{}-{}_{}".format(self.namespace, tag, i)) for i in range(start, stop)]
        self._multiprocessing(self.gazebo_proxy.delete_model, del_kwargs)

    def _delete_other_models(self):
        manager_models = filter(ModelManager._is_manager_model, self.gazebo_proxy.get_model_names())
        other_models = filter(lambda m: not self._is_mine_model(m), manager_models)
        
        del_kwargs = [dict(model_name=m) for m in other_models]
        self._multiprocessing(self.gazebo_proxy.delete_model, del_kwargs)
            
    def _multiprocessing(self, targets, kwargs):
        if callable(targets):
            f = [targets] * len(kwargs)
        else:
            f = targets
            
        ps = [Process(target=f[i], kwargs=kwargs[i]) for i in range(len(kwargs))]
        
        for p in ps:
            p.start()
        for p in ps:
            p.join()
        while True:
            if all([p.exitcode == 0 for p in ps]):
                break