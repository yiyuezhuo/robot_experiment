import bpy 
import numpy as np

def body():
    bpy.ops.mesh.primitive_cylinder_add(radius=0.04, depth=0.053, location = (0.0, 0.0, 0.0))
    bpy.ops.mesh.primitive_ico_sphere_add(size=0.04, location=(0,0.00,0.027))
body()