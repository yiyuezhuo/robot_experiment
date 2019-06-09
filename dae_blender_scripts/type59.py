import bpy 
import numpy as np

def wheel():
    pass

def body():
    # 6.04m x 3.27 x 
    length= np.array([6.04, 3.27, 1.59])

    bpy.ops.mesh.primitive_cube_add(radius=length[0]/2,location=(0, 0, 0.0))
    bpy.context.object.scale[1] = length[1]/length[0]
    bpy.context.object.scale[2] = length[2]/length[0]

    length= np.array([3.04, 2.0, 1.2])

    bpy.ops.mesh.primitive_cube_add(radius=length[0]/2,location=(0, 0, 1.39))
    bpy.context.object.scale[1] = length[1]/length[0]
    bpy.context.object.scale[2] = length[2]/length[0]

    bpy.ops.mesh.primitive_cylinder_add(radius=0.4, depth=7.0, location = (2.5, 0, 1.39))
    bpy.context.object.rotation_euler[1]=np.pi/2

    for wheel_offset in [-2,-1,0,1,2]:
        bpy.ops.mesh.primitive_cylinder_add(radius=0.45, depth=3.5, location = (wheel_offset, 0, -0.5))
        bpy.context.object.rotation_euler[0]=np.pi/2

    bpy.ops.mesh.primitive_cylinder_add(radius=0.3, depth=3.5, location = (-2.5, 0, -0.1))
    bpy.context.object.rotation_euler[0]=np.pi/2



def cavalry():
    # 6.04m x 3.27m x 2.59m
    pass

body()