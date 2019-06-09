import bpy 
import numpy as np

def body():
    length= np.array([0.3, 0.5, 1.5])

    bpy.ops.mesh.primitive_cube_add(radius=length[2]/2,location=(0, 0, 0.0))
    bpy.context.object.scale[0] = length[0]/length[2]
    bpy.context.object.scale[1] = length[1]/length[2]

    length= np.array([0.3, 0.3, 0.3])

    bpy.ops.mesh.primitive_cube_add(radius=length[2]/2,location=(0, 0, 0.9))
    bpy.context.object.scale[0] = length[0]/length[2]
    bpy.context.object.scale[1] = length[1]/length[2]

    length= np.array([0.3, 0.8, 0.7])

    bpy.ops.mesh.primitive_cube_add(radius=length[2]/2,location=(0, 0, 0.4))
    bpy.context.object.scale[0] = length[0]/length[2]
    bpy.context.object.scale[1] = length[1]/length[2]

body()