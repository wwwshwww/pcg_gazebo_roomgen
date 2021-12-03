from roomor.generator import CubeRoomGenerator

generator = CubeRoomGenerator(obstacle_count=3)
room = generator.generate_new()

room.spawn_all()
print(room.target_pose['positions'])
generator.reposition_target(room)
print(room.target_pose['positions'])