
## Below is the dictionary format that is needed by simulator_main.py 

```
    {
        "hand": {"path": <absolute path to hand urdf file>,
                "position": [x, y, z]<float>,
                "orientation": [x, y, z, w]<floats>,
                "scaling": float, 
                "fixed": T/F,
                "starting_joint_angles": [joint_1, joint_2, ..., joint_n]<rads, float>,
                "palm_color": [r,g,b,a]<float 0-1>,
                "segment_colors":[segment_1, segment_2, ..., segment_n]<[r,g,b,a], float 0-1>
                },
        "object": {"path": <absolute path to object urdf file>,
                    "position": [x, y, z]<float>,
                    "orientation": [x, y, z, w]<floats>,
                    "scaling": float,
                    "fixed": T/F,
                    "color": [r,g,b,a]<float 0-1>
                    },
        "trial" : {"episode_number" : <int>,
                    "goal_locations" : {
                                        "x" : [goal_1, goal_2, ..., goal_n],
                                        "y" : [goal_1, goal_2, ..., goal_n]
                                        }
                    }
    }
```