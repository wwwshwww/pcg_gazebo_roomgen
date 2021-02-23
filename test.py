from roomoro.generator import CubeRoomGenerator

import numpy as np
from PIL import Image
import time

c = CubeRoomGenerator(obstacle_count=10)
generated_room1 = c.generate_new()
generated_room2 = c.generate_new()

pose = [-3.0, 0.0, np.pi/2]

data = generated_room2.get_occupancy_grid(generated_room1.get_freespace_poly(), [-pose[0],-pose[1]], -pose[2])
img = Image.fromarray(data.reshape((512,512)).T, 'L')
img.save('my.png')

st = time.time()
generated_room1.spawn_all()
print('spawn time: {}'.format(time.time()-st))

generated_room2.spawn_all()
generated_room1.spawn_all()