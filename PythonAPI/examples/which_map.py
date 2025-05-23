import carla

client = carla.Client('localhost', 2000)
client.set_timeout(5.0)

maps = client.get_available_maps()
for m in maps:
    print(m)