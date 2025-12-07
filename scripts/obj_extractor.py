# import bpy
# import os

# # Write exported meshes to the ".blend" file location
# basedir = os.path.dirname(bpy.data.filepath)
# if not basedir:
#     raise Exception("Blend file is not saved!")

# # Save viewport state
# view_layer = bpy.context.view_layer
# obj_active = view_layer.objects.active
# selection = bpy.context.selected_objects
# bpy.ops.object.select_all(action='DESELECT')

# for obj in selection:
#     # Select single object
#     obj.select_set(True)
#     view_layer.objects.active = obj

#     # Move object to the world origin
#     orgLoc = obj.location.copy()
#     obj.location = (0.0, 0.0, 0.0)

#     # Export object
#     name = bpy.path.clean_name(obj.name)
#     fn = os.path.join(basedir, name)
#     bpy.ops.export_scene.obj(filepath=fn + ".obj", use_selection=True, use_edges=False, use_materials=False, use_triangles=True, axis_forward='Y', axis_up='Z')

#     # Move object back to its original location
#     obj.location = orgLoc
#     obj.select_set(False)
#     print("Written:", fn)

# # Restore viewport state
# view_layer.objects.active = obj_active
# for obj in selection:
#     obj.select_set(True)


# # <external_part name="4-hull-holder_visual_4-hull-holder" type="model" physics="submerged" buoyant="true">
# #     <physical>
# #         <mesh filename="meshes/4-hull-holder_visual_4-hull-holder.obj" scale="1.0" />
# #         <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 -0.0" />
# #     </physical>
# #     <material name="aluminium" />
# #     <look name="look" />
# #     <compound_transform rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0" />
# # </external_part>



import bpy
import os

# Ensure the .blend file is saved
basedir = os.path.dirname(bpy.data.filepath)
if not basedir:
    raise Exception("Blend file is not saved!")

# Output file for XML data
xml_output_path = os.path.join(basedir, "exported_meshes_info.txt")
xml_entries = []

# Save current viewport state
view_layer = bpy.context.view_layer
obj_active = view_layer.objects.active
selection = bpy.context.selected_objects
bpy.ops.object.select_all(action='DESELECT')

for obj in selection:
    obj.select_set(True)
    view_layer.objects.active = obj

    # Save and reset object location
    original_loc = obj.location.copy()
    obj.location = (0.0, 0.0, 0.0)

    # Clean name and file path
    name = bpy.path.clean_name(obj.name)
    filepath = os.path.join(basedir, name + ".obj")

    # Export selected object as OBJ
    bpy.ops.export_scene.obj(
        filepath=filepath,
        use_selection=True,
        use_edges=False,
        use_materials=False,
        use_triangles=True,
        axis_forward='Y',
        axis_up='Z'
    )

    # Restore original position
    obj.location = original_loc
    obj.select_set(False)

    # Generate XML entry
    xml_entry = f'''
    <external_part name="{name}" type="model" physics="submerged" buoyant="true">
    <physical>
        <mesh filename="meshes/{name}.obj" scale="1.0" />
        <origin rpy="0.0 0.0 0.0" xyz="{original_loc.x:.3f} {original_loc.y:.3f} {original_loc.z:.3f}" />
    </physical>
    <material name="aluminium" />
    <look name="look" />
    <compound_transform rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0" />
</external_part>\n'''
    xml_entries.append(xml_entry)

    print("Exported:", filepath)

# Restore viewport state
view_layer.objects.active = obj_active
for obj in selection:
    obj.select_set(True)

# Write XML entries to file
with open(xml_output_path, 'w') as f:
    f.writelines(xml_entries)

print("XML info written to:", xml_output_path)