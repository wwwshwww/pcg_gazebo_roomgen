import numpy as np
import quaternion
import shapely
from shapely.geometry import Polygon
import trimesh
import pcg_gazebo
import copy

def vec_to_trans(vec):
    a = np.identity(len(vec)+1)
    a[:len(vec),len(vec)] = vec
    return a

def add_dimension(points, value=0):
    return np.concatenate([points[np.newaxis,:,i].T if i<len(points[0]) else np.full([len(points),1], value) for i in range(len(points[0])+1)], 1)
    
def get_square_horizon(base_pos, radius, z_angle=0):
    d1 = [1,1,-1,-1]
    d2 = [1,-1,-1,1]
    poss = np.array([np.quaternion(1,d1[i]*radius,d2[i]*radius,0) for i in range(4)])
    q = quaternion.from_euler_angles(0,0,z_angle)
    pq = q*poss*q.conj()
    
    return np.array([[p.x, p.y, p.z] for p in pq])+base_pos

def get_extended_face(face_vertices, length):
    face_bottom = np.copy(face_vertices)
    face_bottom[:,2] += length
    return np.concatenate([face_vertices, face_bottom])
    
def distance_filtered_poly(polygon, distance=0.01):
    c = np.round(polygon.exterior.coords, 3)
    ban = np.zeros([len(c)], dtype=int)
    for i in range(len(c)):
        if not ban[i]:
            n = np.linalg.norm(c-c[i], axis=1) < distance
            n[i] = False
            ban = np.logical_or(ban, n) 
    
    return Polygon(c[np.logical_not(ban)])

def get_initial_vec_from_model_collision(model):
    col = model.get_link_by_name(model.name).collisions[0]
    return col.pose.position

def get_corrected_poly_with_model(target_poly, model):
    col_vec = get_initial_vec_from_model_collision(model)
    corrected = shapely.affinity.affine_transform(target_poly, matrix=[1,0,0,1,col_vec[0],col_vec[1]])
    return corrected

def get_mesh_from_model(model):
    col_position = get_initial_vec_from_model_collision(model)
    col_mesh_before_trans = col.geometry._geometry_entity.get_meshes()[0]
    return col_mesh_before_trans, col_position

def get_corrected_mesh_from_model(model):    
    mesh, t = get_mesh_from_model(model)
    c_mesh = copy.deepcopy(mesh)
    c_mesh.apply_transform(vec_to_trans(t)) #-
    return c_mesh

def get_poly_from_model(model, mesh_slice_height): ## height; ROOM_WALL_HEIGHT/2
    c_mesh = get_corrected_mesh_from_model(model)
    poly = trimesh.path.polygons.projected(c_mesh, (0,0,mesh_slice_height))
    return poly

def get_interior_poly_from_extrude_mesh(mesh, mesh_slice_height):
    extrude_poly = trimesh.path.polygons.projected(mesh, (0,0,mesh_slice_height))
    return distance_filtered_poly(Polygon(extrude_poly.interiors[0]))

def get_interior_poly_from_extrude_model(model, mesh_slice_height):
    extrude_poly = get_poly_from_model(model, mesh_slice_height)
    extrude_interior_poly = distance_filtered_poly(Polygon(extrude_poly.interiors[0])) #-
    return extrude_interior_poly