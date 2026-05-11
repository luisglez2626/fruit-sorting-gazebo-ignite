# ==============================================================================
# File: config.py
# Purpose: Centralized configuration dictionary. 
#          FIXED: Adjusted gripper gap to match physics geometry, and 
#          removed the mathematically incorrect manual camera offset.
# ==============================================================================

SETTINGS = {
    "ui": {
        "window_title": "Advanced Arm Control Panel - Compact Mode",
        "window_geometry": "1250x520",
        
        "font_h1": ("Arial", 12, "bold"),
        "font_h2": ("Arial", 10, "bold"),
        "font_h3": ("Arial", 9, "bold"),
        "font_mono": ("monospace", 9),
        "font_mono_bold": ("monospace", 10, "bold"),
        
        "colors": {
            "home_btn": "yellow",              
            "search_btn": "lightblue",         
            "capture_btn": "purple",           
            "capture_text": "white",           
            
            "red_hover": "pink",               
            "red_pick": "lightcoral",          
            "red_close": "tomato",             
            "red_open": "lightgray",           
            
            "green_hover": "lightgreen",       
            "green_pick": "palegreen",         
            "green_close": "limegreen",        
            "green_open": "lightgray",         
            
            "center_hover": "lightblue",       
            "center_pick": "skyblue",          
            "center_open": "lightgray",        
            
            "reset_btn": "orange",             
            "auto_btn": "gold",                
            
            "status_ok": "blue",               
            "status_warn": "red",              
            "status_green": "green"            
        },
        
        "pad_x": 5,             
        "pad_y": 5,             
        "btn_pad_x": 2,         
        "btn_pad_y": 1,         
        
        "btn_width_sm": 4,      
        "btn_width_md": 8,      
        "btn_width_lg": 18      
    },
    
    "robot": {
        "step_cm": 1.0,
        "step_deg": 5.0,
        "hover_z_cm": 53.5,
        "pick_z_cm": 44.5,
        "gripper_open_mm": 100.0,
        "gripper_close_mm": 55.0, # Perfectly matches the 55mm diameter of the apples
        "center_x_cm": -50.0,
        "center_y_cm": 0.0,
        "tolerance_cm": 2.5,
        "tolerance_rad": 0.05
    },
    
    "simulation": {
        "reset_z": 0.4275,
        "apple_red_start_x": -0.5,
        "apple_red_start_y": 0.1,
        "apple_green_start_x": -0.5,
        "apple_green_start_y": -0.1
    }
}