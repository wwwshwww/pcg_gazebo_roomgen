from roomor.generator import CubeRoomGenerator
import time

c = CubeRoomGenerator(obstacle_count=10)
generated_room = c.generate_new()

st = time.time()
generated_room.spawn_all()
print('spawn time: {}'.format(time.time()-st))