import trimesh

mesh = trimesh.load('tray_shaped_ee.obj', force='mesh')

if not mesh.is_watertight:
    print("Warning: Mesh is not watertight. Volume/inertia might be inaccurate.")

volume = mesh.volume
cog = mesh.center_mass
inertia = mesh.moment_inertia  # 3x3 numpy array

print("Volume:", volume)
print("Center of gravity:", cog)
print("Inertia matrix:\n", inertia)

