#!/usr/bin/env python3
"""Generate truck SDFs — Cat Tech Challenge 2026 · Team TTSK12"""
from pathlib import Path
HERE = Path(__file__).parent.resolve()

FLEET = [
    ("100t", ( 9.0, 5.0, 3.0), (1.0, 0.8), (2.5, 4.0, 2.0), (0.20, 0.40, 0.85)),
    ("250t", (12.0, 7.0, 4.0), (1.3, 1.0), (3.0, 5.5, 2.5), (0.25, 0.70, 0.30)),
    ("400t", (15.0, 9.0, 5.0), (1.6, 1.2), (3.5, 7.0, 3.0), (0.85, 0.20, 0.20)),
]

def wheel_link(n, x, y, r, w):
    return f'''    <link name="{n}">
      <pose>{x:.3f} {y:.3f} {r:.3f} -1.5708 0 0</pose>
      <inertial><mass>500</mass><inertia><ixx>120</ixx><ixy>0</ixy><ixz>0</ixz><iyy>120</iyy><iyz>0</iyz><izz>200</izz></inertia></inertial>
      <visual name="v"><geometry><cylinder><radius>{r}</radius><length>{w}</length></cylinder></geometry>
        <material><ambient>0.08 0.08 0.08 1</ambient><diffuse>0.18 0.18 0.18 1</diffuse></material></visual>
      <collision name="c"><geometry><cylinder><radius>{r}</radius><length>{w}</length></cylinder></geometry>
        <surface><friction><ode><mu>1.5</mu><mu2>1.5</mu2></ode></friction></surface></collision>
    </link>'''

def wheel_joint(jn, ch):
    return f'''    <joint name="{jn}" type="revolute">
      <parent>chassis</parent><child>{ch}</child>
      <axis><xyz expressed_in="__model__">0 1 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper><effort>1e6</effort><velocity>50</velocity></limit>
        <dynamics><friction>0.05</friction></dynamics></axis>
    </joint>'''

def build_sdf(name, chassis, wheel, cab, color):
    cl,cw,ch = chassis; wr,ww = wheel
    cab_l,cab_w,cab_h = cab; cr,cg,cb = color
    chassis_z = wr + ch/2
    x_front, x_rear = cl/2 - 1.5, -(cl/2 - 1.5)
    y_left = cw/2 + ww/2 + 0.1; y_right = -y_left
    track_width = 2*y_left
    cab_x = cl/2 - cab_l/2 - 1.0
    cab_z = ch/2 + cab_h/2
    lidar_z = ch/2 + cab_h + 0.5
    mass = round(cl*cw*ch*90)
    ixx = round(mass*(cw**2+ch**2)/12); iyy = round(mass*(cl**2+ch**2)/12); izz = round(mass*(cl**2+cw**2)/12)
    wheels = "\n".join([wheel_link(n,x,y,wr,ww) for n,x,y in
        [("wheel_fl",x_front,y_left),("wheel_fr",x_front,y_right),
         ("wheel_rl",x_rear,y_left),("wheel_rr",x_rear,y_right)]])
    joints = "\n".join([wheel_joint(f"{n}_joint",n) for n in
        ["wheel_fl","wheel_fr","wheel_rl","wheel_rr"]])
    return f'''<?xml version="1.0"?>
<sdf version="1.10">
  <model name="truck_{name}">
    <self_collide>false</self_collide>
    <link name="chassis">
      <pose>0 0 {chassis_z:.3f} 0 0 0</pose>
      <inertial><mass>{mass}</mass><inertia><ixx>{ixx}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{iyy}</iyy><iyz>0</iyz><izz>{izz}</izz></inertia></inertial>
      <visual name="body"><geometry><box><size>{cl} {cw} {ch}</size></box></geometry>
        <material><ambient>{cr*0.55:.3f} {cg*0.55:.3f} {cb*0.55:.3f} 1</ambient><diffuse>{cr:.3f} {cg:.3f} {cb:.3f} 1</diffuse></material></visual>
      <collision name="body_c"><geometry><box><size>{cl} {cw} {ch}</size></box></geometry></collision>
      <visual name="cab"><pose>{cab_x:.3f} 0 {cab_z:.3f} 0 0 0</pose>
        <geometry><box><size>{cab_l} {cab_w} {cab_h}</size></box></geometry>
        <material><ambient>0.22 0.22 0.22 1</ambient><diffuse>0.42 0.42 0.42 1</diffuse></material></visual>
      <visual name="lidar_post"><pose>{cab_x:.3f} 0 {ch/2+cab_h+0.25:.3f} 0 0 0</pose>
        <geometry><cylinder><radius>0.12</radius><length>0.5</length></cylinder></geometry>
        <material><ambient>0.05 0.05 0.05 1</ambient></material></visual>
      <sensor name="lidar" type="gpu_lidar"><pose>{cab_x:.3f} 0 {lidar_z:.3f} 0 0 0</pose>
        <topic>/truck_{name}/scan</topic><update_rate>10</update_rate><always_on>1</always_on><visualize>false</visualize>
        <ray><scan>
          <horizontal><samples>360</samples><resolution>1</resolution><min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal>
          <vertical><samples>16</samples><resolution>1</resolution><min_angle>-0.26</min_angle><max_angle>0.26</max_angle></vertical>
        </scan><range><min>0.5</min><max>50.0</max><resolution>0.05</resolution></range></ray>
      </sensor>
    </link>
{wheels}
{joints}
    <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
      <left_joint>wheel_fl_joint</left_joint><left_joint>wheel_rl_joint</left_joint>
      <right_joint>wheel_fr_joint</right_joint><right_joint>wheel_rr_joint</right_joint>
      <wheel_separation>{track_width:.3f}</wheel_separation><wheel_radius>{wr}</wheel_radius>
      <odom_topic>/truck_{name}/odometry</odom_topic><tf_topic>tf</tf_topic>
      <frame_id>odom</frame_id><child_frame_id>base_link</child_frame_id>
      <odom_publish_frequency>20</odom_publish_frequency>
      <max_linear_acceleration>2.0</max_linear_acceleration><max_angular_acceleration>1.0</max_angular_acceleration>
    </plugin>
    <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
      <publish_link_pose>false</publish_link_pose><publish_model_pose>true</publish_model_pose>
      <publish_nested_model_pose>false</publish_nested_model_pose><update_frequency>10</update_frequency>
    </plugin>
  </model>
</sdf>
'''

def build_config(name):
    return f'''<?xml version="1.0"?>
<model>
  <name>truck_{name}</name><version>1.0</version>
  <sdf version="1.10">model.sdf</sdf>
  <description>{name.upper()} mining truck — Cat Tech Challenge 2026 · Team TTSK12</description>
</model>
'''

for entry in FLEET:
    name = entry[0]
    out = HERE / f"truck_{name}"; out.mkdir(exist_ok=True)
    (out/"model.sdf").write_text(build_sdf(*entry))
    (out/"model.config").write_text(build_config(name))
    print(f"  wrote {out.relative_to(HERE.parent)}/")
print(f"\nDone. Export resource path:\n    export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:{HERE}")
