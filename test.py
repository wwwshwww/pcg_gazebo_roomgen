from roomoro.generator import CubeRoomGenerator
import time

c = CubeRoomGenerator(obstacle_count=10)
generated_room1 = c.generate_new()
generated_room2 = c.generate_new()

st = time.time()
generated_room1.spawn_all()
print('spawn time: {}'.format(time.time()-st))

generated_room2.spawn_all()
generated_room1.spawn_all()