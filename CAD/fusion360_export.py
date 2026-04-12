import adsk.core, adsk.fusion, adsk.cam, traceback
import os

def run(context):
    """
    Fusion 360 Add-In script to auto-export standard robotic mechanical links
    to both STEP (for assemblies) and STL (for slicing and simulation).
    """
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = app.activeProduct
        
        if not design:
            ui.messageBox('No active Fusion 360 design', 'Export Error')
            return

        # Hardcoded to your local PBL repository
        export_folder_stl = r"C:\Users\Riyansh\PBL_project\CAD\STL"
        export_folder_step = r"C:\Users\Riyansh\PBL_project\CAD\STEP"
        
        os.makedirs(export_folder_stl, exist_ok=True)
        os.makedirs(export_folder_step, exist_ok=True)

        components = design.allComponents
        export_mgr = design.exportManager
        
        count = 0
        for comp in components:
            # We filter for relevant parts (assuming you name them "link0", "link1", "base", etc.)
            name_lower = comp.name.lower()
            if "link" in name_lower or "base" in name_lower or "housing" in name_lower:
                
                # Clean up filename
                safe_name = comp.name.replace(":", "_").replace(" ", "_")
                
                # 1. Export STEP format
                step_path = os.path.join(export_folder_step, f"{safe_name}.step")
                step_options = export_mgr.createSTEPExportOptions(step_path, comp)
                export_mgr.execute(step_options)
                
                # 2. Export STL format
                stl_path = os.path.join(export_folder_stl, f"{safe_name}.stl")
                stl_options = export_mgr.createSTLExportOptions(comp)
                stl_options.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
                stl_options.filename = stl_path
                export_mgr.execute(stl_options)
                
                count += 1
                
        ui.messageBox(f'Successfully exported {count} mechanical links to:\n{export_folder_stl}\nand\n{export_folder_step}', 'Export Complete')

    except Exception as e:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
