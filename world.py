"""Generate an entirely local SDF scene: differential-drive robot and inspection panel."""
from pathlib import Path
import math

def box(name,pose,size,color,collision=True,static=True):
    geom=f'<geometry><box><size>{size}</size></box></geometry>'
    return f'''<model name="{name}"><static>{str(static).lower()}</static><pose>{pose}</pose><link name="body">
    {f'<collision name="collision">{geom}</collision>' if collision else ''}
    <visual name="visual">{geom}<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual></link></model>'''

def panel(label,x=4,y=0,z=.85):
    visuals=''
    if label:
        # Physical, rendered dark crack segments on the front face (not a fake sensor overlay).
        for i,(dy,dz,roll) in enumerate([(-.12,.24,-.35),(-.06,.10,.4),(0,-.04,-.25),(.05,-.18,.4),(.14,-.01,-.9)]):
            visuals+=f'''<visual name="crack{i}"><pose>-.032 {dy} {dz} {roll} 0 0</pose><geometry><box><size>.012 .045 .22</size></box></geometry><material><ambient>.015 .015 .015 1</ambient><diffuse>.015 .015 .015 1</diffuse></material></visual>'''
    return f'''<model name="panel"><static>true</static><pose>{x} {y} {z} 0 0 0</pose><link name="body"><collision name="collision"><geometry><box><size>.05 1.2 1.0</size></box></geometry></collision><visual name="panel"><geometry><box><size>.05 1.2 1.0</size></box></geometry><material><ambient>.78 .80 .82 1</ambient><diffuse>.78 .80 .82 1</diffuse></material></visual>{visuals}</link></model>'''

def robot(pose='0 0 .18 0 0 0'):
    wheels=''; joints=''
    for name,y in [('left',.23),('right',-.23)]:
        wheels+=f'''<link name="{name}_wheel"><pose>0 {y} -.055 1.5707963 0 0</pose><inertial><mass>.4</mass><inertia><ixx>.001</ixx><iyy>.001</iyy><izz>.001</izz></inertia></inertial><collision name="collision"><geometry><cylinder><radius>.125</radius><length>.05</length></cylinder></geometry><surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision><visual name="visual"><geometry><cylinder><radius>.125</radius><length>.05</length></cylinder></geometry><material><ambient>.08 .10 .12 1</ambient><diffuse>.08 .10 .12 1</diffuse></material></visual></link>'''
        joints+=f'''<joint name="{name}_joint" type="revolute"><parent>base_link</parent><child>{name}_wheel</child><axis><xyz expressed_in="__model__">0 1 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis></joint>'''
    return f'''<model name="robot"><pose>{pose}</pose><self_collide>false</self_collide>
    <link name="base_link"><inertial><pose>-.04 0 0 0 0 0</pose><mass>3</mass><inertia><ixx>.07</ixx><iyy>.07</iyy><izz>.09</izz></inertia></inertial>
    <collision name="body"><geometry><box><size>.36 .36 .14</size></box></geometry></collision>
    <visual name="body"><geometry><box><size>.36 .36 .14</size></box></geometry><material><ambient>.04 .55 .65 1</ambient><diffuse>.04 .55 .65 1</diffuse></material></visual>
    <collision name="caster"><pose>-.13 0 -.14 0 0 0</pose><geometry><sphere><radius>.04</radius></sphere></geometry><surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface></collision>
    <sensor name="camera" type="camera"><pose>.14 0 .64 0 0 0</pose><always_on>true</always_on><update_rate>10</update_rate><topic>/camera/image</topic><camera><horizontal_fov>1.15</horizontal_fov><image><width>320</width><height>240</height><format>R8G8B8</format></image><clip><near>.05</near><far>20</far></clip></camera></sensor>
    <sensor name="contact" type="contact"><always_on>true</always_on><update_rate>20</update_rate><topic>/robot/contacts</topic><contact><collision>body</collision></contact></sensor>
    </link>{wheels}{joints}
    <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive"><left_joint>left_joint</left_joint><right_joint>right_joint</right_joint><wheel_separation>.46</wheel_separation><wheel_radius>.125</wheel_radius><odom_publish_frequency>30</odom_publish_frequency><topic>/cmd_vel</topic><odom_topic>/odom</odom_topic><frame_id>odom</frame_id><child_frame_id>base_link</child_frame_id></plugin>
    <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher"><publish_link_pose>false</publish_link_pose><publish_model_pose>true</publish_model_pose><publish_nested_model_pose>true</publish_nested_model_pose><use_pose_vector_msg>true</use_pose_vector_msg><update_frequency>30</update_frequency><topic>/robot/world_pose</topic></plugin>
    </model>''' 

def world(label=1,blocked=True,distance=4,light=1.0,pose='0 0 .18 0 0 0',crack_y=0):
    scene=box('ground','0 0 -.05 0 0 0','20 20 .1','.22 .26 .30 1')
    scene+=panel(label,distance,crack_y)
    if blocked: scene+=box('occluder',f'{distance*.5} 0 .7 0 0 0','.25 1.1 1.4','.35 .27 .18 1')
    scene+=robot(pose)
    scene+='''<model name="overview"><static>true</static><pose>2 -4.8 3.8 0 .60 1.5707963</pose><link name="camera"><sensor name="overview" type="camera"><always_on>true</always_on><update_rate>5</update_rate><topic>/overview/image</topic><camera><horizontal_fov>1.25</horizontal_fov><image><width>480</width><height>360</height><format>R8G8B8</format></image><clip><near>.1</near><far>30</far></clip></camera></sensor></link></model>''' 
    return f'''<?xml version="1.0"?><sdf version="1.9"><world name="inspection"><gravity>0 0 -9.8</gravity>
    <physics name="physics" type="ignored"><max_step_size>.005</max_step_size><real_time_factor>2</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <scene><ambient>{.12*light} {.12*light} {.12*light} 1</ambient><background>.12 .16 .22 1</background><shadows>false</shadows></scene>
    <light type="directional" name="sun"><pose>0 0 5 0 0 0</pose><diffuse>{light} {light} {light} 1</diffuse><specular>.05 .05 .05 1</specular><direction>.6 .2 -1</direction></light>{scene}</world></sdf>'''

if __name__=='__main__': Path('inspection.sdf').write_text(world())
