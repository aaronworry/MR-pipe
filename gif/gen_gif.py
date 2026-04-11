import imageio
with imageio.get_writer(uri='map1.gif', mode='I', fps=10) as writer:
    for i in range(503):
        writer.append_data(imageio.imread(f'{i}.jpg'))