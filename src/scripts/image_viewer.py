import numpy as np
import matplotlib.pyplot as plt

width = 640
height = 360
channels = 1  # mono8

with open('/tmp/captured_image_raw_1761045964.bin', 'rb') as f:
    data = np.frombuffer(f.read(), dtype=np.uint8)

image = data.reshape((height, width))  # no channels dimension
plt.imshow(image, cmap='gray')
plt.axis('off')
plt.show()

