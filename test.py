from roomor.generator import CubeRoomGenerator

c = CubeRoomGenerator(obstacle_count=10)
generated_room = c.generate_new()
generated_room.spawn_all()