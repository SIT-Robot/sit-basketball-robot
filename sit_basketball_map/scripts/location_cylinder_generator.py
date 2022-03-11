from PIL import Image, ImageDraw

img = Image.new(mode='RGB',
                size=(20, 100),
                color=(255, 255, 255))

draw = ImageDraw.Draw(img)

draw.rectangle(xy=[(0, 0), (20, 30)],
               fill=(0, 0, 255))
draw.rectangle(xy=[(0, 30), (20, 70)],
               fill=(0, 255, 0))
draw.rectangle(xy=[(0, 70), (20, 100)],
               fill=(0, 0, 255))
import matplotlib.pyplot as plt

plt.figure('test')
plt.imshow(img)
plt.show()

img.save('cylinder.png','png')
