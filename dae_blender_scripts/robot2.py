import bpy 
import numpy as np

#print('loading....')

def propeller(x,y,z,rot):
    bpy.ops.mesh.primitive_cylinder_add(radius=0.02, depth=0.1, location = (x,y,z+0.05))
    
    bpy.ops.mesh.primitive_cube_add(radius=0.15, location = (x,y,z+0.1-0.005))
    bpy.context.object.scale[1] = 0.1333
    bpy.context.object.scale[2] = 0.0033
    
    bpy.context.object.rotation_euler[2]=rot

def arm(x,y , rot):
    bpy.ops.mesh.primitive_cube_add(radius=0.494,location=(x, y, -0.0250))
    bpy.context.object.scale[1] = 0.1417/2
    bpy.context.object.scale[2] = 0.1012/2
    
    bpy.context.object.rotation_euler[2]=rot

def shell(x=0.5,y=0.5):
    print('start wtf')
    bpy.ops.mesh.primitive_cylinder_add(radius=0.14, depth=0.2, location = (x, y, 0.0))
    
    bpy.ops.mesh.primitive_cylinder_add(radius=0.07, depth=0.2, location = (x-0.35, y+0.35, 0.0))
    bpy.ops.mesh.primitive_cylinder_add(radius=0.07, depth=0.2, location = (x+0.35, y+0.35, 0.0))
    bpy.ops.mesh.primitive_cylinder_add(radius=0.07, depth=0.2, location = (x+0.35, y-0.35, 0.0))
    bpy.ops.mesh.primitive_cylinder_add(radius=0.07, depth=0.2, location = (x-0.35, y-0.35, 0.0))
    
    arm(x, y, np.pi/4)
    arm(x, y, -np.pi/4)
    
def nice_boat(x=0.5,y=0.5):
    shell(x=x,y=y)
    propeller(x-0.35, y+0.35, 0.1, -np.pi/4)
    propeller(x+0.35, y+0.35, 0.1,  np.pi/4)
    propeller(x+0.35, y-0.35, 0.1, -np.pi/4)
    propeller(x-0.35, y-0.35, 0.1,  np.pi/4)

# show example part    
#nice_boat(x=0,y=0)
# show shell
#shell(x=0,y=0)
# show propeller
#propeller(0,0,0,0)