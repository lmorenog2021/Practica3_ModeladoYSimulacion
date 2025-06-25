# Práctica 3: Simulación de Robots usando middleware
## Parte A
### Creación de .xacros y primeros pasos
Empezamos con el URDF único que nos genera blender automaticamente, y tenemos que fraccionarlo para de manera que cumpla con la estructura requerida.

![image](https://github.com/user-attachments/assets/7e914556-1626-44ad-b943-bf69dbd22703)

Para conseguir esto lo primero que hacemos es dividir los links y joints a los ficheros correspondientes, para posteriormente añadir el cabezal de xacro.

![image](https://github.com/user-attachments/assets/f4b57ca6-c631-4b04-bbe0-0718d690eea8)

El siguiente paso es crear un macro de XACRO en cada fichero, de manera que podamos importarlos en "robot.urdf.xacro". Como ejemplo usaré wheels.urdf.xacro, ya que es el que tiene los cambios más interesantes.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="rover_wheels" params="offset_x offset_y offset_z parent_link identifier">
    <!--Joint-->
    <joint name="${identifier}_Wheel_joint" type="continuous">
      <limit lower="0" upper="0" effort="0.00000" velocity="0.00000"/>
      <origin rpy="1.57080 0.00000 0.00000" xyz="${offset_x} ${offset_y} ${offset_z}"/>
      <parent link="${parent_link}"/>
      <child link="${identifier}_Wheel_link"/>
      <axis xyz="0.00000 0.00000 1.00000"/>
    </joint>

    <!--Link-->
    <link name="${identifier}_Wheel_link">
      <collision name="Wheel_collision">
        <origin rpy="0.00000 0.00000 0.00000" xyz="-0.00000 -0.00000 -0.00000"/>
        <geometry>
          <cylinder radius="0.30000" length="0.10000"/>
        </geometry>
      </collision>
      <inertial>
        <inertia ixx="0.02333" ixy="0.00000" ixz="0.00000" iyy="0.04500" iyz="0.00000" izz="0.02333"/>
        <origin rpy="0.00000 0.00000 0.00000" xyz="-0.00000 0.00000 -0.00000"/>
        <mass value="1.00000"/>
      </inertial>
      <visual name="${identifier}_Wheel">
        <origin rpy="0.00000 0.00000 0.00000" xyz="0.00000 0.00000 -0.00000"/>
        <material name="Wheels"/>
        <geometry>
          <cylinder radius="0.30000" length="0.10000"/>
        </geometry>
      </visual>
    </link>

  </xacro:macro>
</robot>
```
Gracias al uso de xacro en vez de tener 4 joints y 4 links diferentes para las 4 ruedas, usamos uno de cada y con variables de posición y de identificador podemos crear 4 instancias de la misma rueda en distintos sitios, esto simplificaría el proceso de cambiar los parametros de las ruedas, ya que solo se necesita tambiar una.

#### Sensores y cámaras
Tomando los ejemplos del aula virtual podemos crear los elementos necesarios para los siguientes puntos del trabajo.

Dos cámaras, una en la parte frontal del robot, y otra en la pinza. Es importante mantener la configuración exacta de estas cámaras, porque si no darán muchos problemas en los siguientes pasos de la práctica.
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro">

  <xacro:macro name="camera" params="offset_x offset_y offset_z parent_link" >

    <joint name="camera_link_joint" type="fixed">
      <origin rpy="-3.14159 1.57080 -3.14159" xyz="${offset_x} ${offset_y} ${offset_z}"/>
      <parent link="${parent_link}"/> 
      <child link="camera_link"/>
    </joint>

    <link name="camera_link">
      <visual name="camera">
        <origin rpy="3.14159 -1.57080 3.14159" xyz="0.00000 0.00000 0.00000"/>
        <material name="Sensors"/>
        <geometry>
          <box size="0.10000 0.30000 0.20000"/>
        </geometry>
      </visual>
    </link>

    <gazebo reference="camera_link">
      <sensor name="camera" type="camera">
        <visualize>true</visualize>
        <update_rate>30</update_rate>
        <topic>/front_camera/image</topic>
        <camera>
          <horizontal_fov>1.57</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.10</near>
            <far>15.0</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.007</stddev>
          </noise>
          <optical_frame_id>camera_link</optical_frame_id>
        </camera>
      </sensor>
    </gazebo>

  </xacro:macro>
</robot>
```
Un sensor IMU que se encuentra en el centro del robot.
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro">

  <xacro:macro name="imu_sensor" params="offset_x offset_y offset_z parent_link" >

    <joint name="IMU_link_joint" type="fixed">
      <origin rpy="0.00000 0.00000 0.00000" xyz="${offset_x} ${offset_y} ${offset_z}"/>
      <parent link="${parent_link}"/> 
      <child link="IMU_link"/>
    </joint>

    <link name="IMU_link">
      <visual name="IMU">
        <origin rpy="0.00000 0.00000 0.00000" xyz="0.00000 0.00000 0.00000"/>
        <material name="Sensors"/>
        <geometry>
          <box size="0.10000 0.10000 0.10000"/>
        </geometry>
      </visual>
    </link>

    <gazebo reference="IMU_link">
      <sensor name="IMU" type="imu">
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <topic>"imu_data"</topic>
      </sensor>
    </gazebo>
   
  </xacro:macro>
</robot>
```
### Crear robot.urdf.xacro y combinar los macros de xacro
Combinamos todos estos ficheros en este último, que es el encargado de tomar todas las piezas que hemos hechop con los macros de XACRO y unirlas para crear el robot final. Vemos las variables con los offsets necesarios para unir varias piezas, vemos también las 4 ruedas distintas siendo creadas con el mismo macro, y por último vemos como se añaden los sensores relevantes.
```xml
<?xml version="1.0"?>
<robot name="simple_rover" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <material name="Wheels">
    <color rgba="0.00000 0.00000 0.00000 1.00000"/> <!-- Black -->
  </material>

  <material name="Chasis">
    <color rgba="0.00000 0.00000 0.54510 1.00000"/> <!-- Dark Blue -->
  </material>

  <material name="Axels">
    <color rgba="0.50200 0.50200 0.50200 1.00000"/> <!-- Grey -->
  </material>

  <material name="Arm">
    <color rgba="0.67843 0.84705 1.00000 1.00000"/> <!-- Light Blue -->
  </material>

  <material name="Gripper">
    <color rgba="0.00000 1.00000 1.00000 1.00000"/> <!-- Bright Cyan -->
  </material>

  <material name="Sensors">
    <color rgba="0.00000 0.00000 0.00000 1.00000"/> <!-- Black -->
  </material>

  <!-- Includes -->
  <xacro:include filename="$(find simple_rover_description)/urdf/arm/gripper.urdf.xacro"/>
  <xacro:include filename="$(find simple_rover_description)/urdf/arm/scara.urdf.xacro"/>
  <xacro:include filename="$(find simple_rover_description)/urdf/base/robot_base.urdf.xacro"/>
  <xacro:include filename="$(find simple_rover_description)/urdf/sensors/camera.urdf.xacro"/>
  <xacro:include filename="$(find simple_rover_description)/urdf/sensors/imu_sensor.urdf.xacro"/>
  <xacro:include filename="$(find simple_rover_description)/urdf/sensors/sensor.urdf.xacro"/>
  <xacro:include filename="$(find simple_rover_description)/urdf/wheels/rover_wheels.urdf.xacro"/>
  
  <!-- Properties -->
  <!-- Front Left Wheel Offset -->
  <xacro:property name="FL_wheel_offset_x" value="-0.00000"/>
  <xacro:property name="FL_wheel_offset_y" value="1.00000"/>
  <xacro:property name="FL_wheel_offset_z" value="-0.00000"/>

  <!-- Front Right Wheel Offset -->
  <xacro:property name="FR_wheel_offset_x" value="-0.00000"/>
  <xacro:property name="FR_wheel_offset_y" value="-1.00000"/>
  <xacro:property name="FR_wheel_offset_z" value="-0.00000"/>

  <!-- Back Left Wheel Offset -->
  <xacro:property name="BL_wheel_offset_x" value="0.00000"/>
  <xacro:property name="BL_wheel_offset_y" value="1.00000"/>
  <xacro:property name="BL_wheel_offset_z" value="-0.00000"/>

  <!-- Back Right Wheel Offset -->
  <xacro:property name="BR_wheel_offset_x" value="0.00000"/>
  <xacro:property name="BR_wheel_offset_y" value="-1.00000"/>
  <xacro:property name="BR_wheel_offset_z" value="-0.00000"/>

  <!-- SCARA Arm Offset -->
  <xacro:property name="Scara_offset_x" value="0.00000"/>
  <xacro:property name="Scara_offset_y" value="0.00000"/>
  <xacro:property name="Scara_offset_z" value="0.50000"/>

  <!-- Gripper Offset -->
  <xacro:property name="Gripper_offset_x" value="-0.00000"/>
  <xacro:property name="Gripper_offset_y" value="-0.00000"/>
  <xacro:property name="Gripper_offset_z" value="-0.91476"/>

  <!-- Camera Offset -->
  <xacro:property name="Camera_offset_x" value="0.25000"/>
  <xacro:property name="Camera_offset_y" value="0.00000"/>
  <xacro:property name="Camera_offset_z" value="0.10000"/>

  <!-- IMU Offset -->
  <xacro:property name="IMU_offset_x" value="-0.00000"/>
  <xacro:property name="IMU_offset_y" value="-0.00000"/>
  <xacro:property name="IMU_offset_z" value="-0.00000"/>

  <!-- Sensor Offset -->
  <xacro:property name="Sensor_offset_x" value="-0.00000"/>
  <xacro:property name="Sensor_offset_y" value="-0.00000"/>
  <xacro:property name="Sensor_offset_z" value="-0.06000"/>


  <!-- Building -->
  <xacro:robot_base />

  <xacro:scara offset_x="${Scara_offset_x}" offset_y="${Scara_offset_y}" offset_z="${Scara_offset_z}" parent_link="Chasis.004_link"/>
  <xacro:gripper offset_x="${Gripper_offset_x}" offset_y="${Gripper_offset_y}" offset_z="${Gripper_offset_z}" parent_link="Cylinder.003_link"/>

  <xacro:rover_wheels offset_x="${FL_wheel_offset_x}" offset_y="${FL_wheel_offset_y}" offset_z="${FL_wheel_offset_z}" parent_link="Cube.002_link" identifier="FL"/>
  <xacro:rover_wheels offset_x="${FR_wheel_offset_x}" offset_y="${FR_wheel_offset_y}" offset_z="${FR_wheel_offset_z}" parent_link="Cube.002_link" identifier="FR"/>
  <xacro:rover_wheels offset_x="${BL_wheel_offset_x}" offset_y="${BL_wheel_offset_y}" offset_z="${BL_wheel_offset_z}" parent_link="Cube.001_link" identifier="BL"/>
  <xacro:rover_wheels offset_x="${BR_wheel_offset_x}" offset_y="${BR_wheel_offset_y}" offset_z="${BR_wheel_offset_z}" parent_link="Cube.001_link" identifier="BR"/>

  <!-- <xacro:camera offset_x="${Camera_offset_x}" offset_y="${Camera_offset_y}" offset_z="${Camera_offset_z}" parent_link="Chasis.004_link"/> -->
  <xacro:imu_sensor offset_x="${IMU_offset_x}" offset_y="${IMU_offset_y}" offset_z="${IMU_offset_z}" parent_link="base_link"/>
  <xacro:sensor offset_x="${Sensor_offset_x}" offset_y="${Sensor_offset_y}" offset_z="${Sensor_offset_z}" parent_link="Cube_link"/>

  <!-- sensores -->
  <!-- <xacro:sensor_camera node_name="front_camera" node_namespace="$(arg robot_ns)" frame_prefix="$(arg prefix)front_camera_" parent="$(arg prefix)base_link">
  <origin xyz="0.0 1.4 0.5" rpy="0 0 ${PI/2}"/>
  </xacro:sensor_camera> -->
  <!--
  xacro:sensor_camera node_name="arm_camera" node_namespace="$(arg robot_ns)" frame_prefix="$(arg prefix)arm_camera_" parent="$(arg prefix)base_effector_arm_link" >
          <origin xyz="0.0 0.0 -0.01" rpy="0 3.141592 0"/>
      </xacro:sensor_camera
  -->
  <!--
  <xacro:sensor_imu ns="$(arg robot_ns)" prefix="$(arg prefix)imu_" parent="$(arg prefix)base_link">
          <origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>
      </xacro:sensor_imu> 
  -->
  <!-- Gazebo ROS control plugins -->
  <!-- <xacro:include filename="$(find simple_rover_description)/urdf/ros2_control.urdf.xacro"/> -->
  <!--
  <xacro:arg name="config_controllers" default="$(find gz_ros2_control_demos)/config/diff_drive_controller.yaml"/> 
  -->
  <!-- <xacro:arg name="config_controllers" default="$(find simple_rover_description)/config/rover_controllers.yaml"/>
  <xacro:arg name="update_rate" default="100"/>
  <xacro:ros2_control/> -->
```
Los tags "Material" podrían pasarse a otro xacro utils, pero como son pocos me ha sido mas comodo tenerlos organizados en un lugar más central.
### Control en RVIZ
Para poder visualizar el robot completo y manipular los joints necesitamos crear un launcher.
```python
from os.path import join
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros.descriptions

def generate_launch_description():
    # Declare arguments
    description_file = LaunchConfiguration("description_file", default="robot.urdf.xacro")
    prefix = LaunchConfiguration("prefix", default="")
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')


    robot_description_content = Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("simple_rover_description"), "robots", "robot.urdf.xacro"]),
    ])

    robot_description_param = launch_ros.descriptions.ParameterValue(robot_description_content, value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
          'use_sim_time': use_sim_time,
          'robot_description': robot_description_param,
        }],
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([FindPackageShare("simple_rover_description"), "rviz", "rover_rviz.rviz"])]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node, #Comentar a veces
        rviz_node
    ])
```
Este launcher genera varios nodos, uno para el robot, otro con una GUI para mover los joints, y un último para RVIZ, que es cargado acorde con las especificaciones de un fichero .rviz.

Aquí podemos ver RVIZ en funcionamiento:
![image](https://github.com/user-attachments/assets/af54b221-ef7c-46e2-b39c-77aad42282b0)
![image](https://github.com/user-attachments/assets/ad053370-bb04-493f-a364-0b50827a766d)


### Diagrama de arbol
Con RVIZ funcionando, lanzamos en otra terminal este comando
```shell
ros2 run tf2_tools view_frames
```
Y vemos el PDF que genera:
![image](https://github.com/user-attachments/assets/035f276a-26fe-43ab-ad93-ed0c95517773)
En él se puede ver como todos los links originan de base_link y este a su vez origina de base_footprint.

## Parte B
Siguiendo las instrucciones del vídeo de clase configuramos MoveIt:
![Screenshot from 2025-05-11 13-49-17](https://github.com/user-attachments/assets/9fa0799a-00f9-4214-bdc2-a43dbac8fc43)
![Screenshot from 2025-05-11 14-49-57](https://github.com/user-attachments/assets/2a6095fd-1ce7-4bd3-adad-1a245a7bcbd9)
![Screenshot from 2025-05-11 14-50-10](https://github.com/user-attachments/assets/09df116d-5052-4116-a6c2-4a84e8e70b86)
![Screenshot from 2025-05-11 14-50-30](https://github.com/user-attachments/assets/718d83d5-5b66-4169-9fc8-2226f7f58b96)
![Screenshot from 2025-05-11 14-50-39](https://github.com/user-attachments/assets/68c96006-1ff1-46e6-9dc6-ffd2f0b8174e)
![Screenshot from 2025-05-11 14-50-51](https://github.com/user-attachments/assets/145c58d1-b9dd-441e-b45a-38b785c9f43d)
![Screenshot from 2025-05-11 14-50-59](https://github.com/user-attachments/assets/28879760-14d1-48a0-a279-29d7d3570b9f)


Comprobamos que el teleop funciona, y con las cámaras nos acercamos al cubo:
![Inicio](https://github.com/user-attachments/assets/82ceda21-43c9-4299-91db-4876040bf519)
![Destino](https://github.com/user-attachments/assets/377eeec2-c2bc-4535-ad6f-e89756a3dc49)

Con la GUI hacemos la planificacion y movemos el brazo y gripper.
![image](https://github.com/user-attachments/assets/1771c09e-e7c7-4f22-abc0-3edd994cf29d)
![image](https://github.com/user-attachments/assets/70131025-3845-43fd-8612-bb93cc285aee)

### Tiempo vs G-Parcial
Con el [ROSBAG] sacamos los valores del effort, y el sumatorio de estos es lo que nos da la G parcial.

![image](https://github.com/user-attachments/assets/ac4bb497-9622-4689-b78a-4e77ead6c91e)

Esta es la gráfica:

![image](https://github.com/user-attachments/assets/8285dab4-cbe9-4eed-a62e-ba0d306d4f65)

Y podemos ver 3 elementos distintos, un tramo de altas oscilaciones, un tramo plano, y un tramo curvado. El tramo de altas oscilaciones representan el momento en el que el brazo scara levanta la caja. El tramo plano es el momento en el que me puse a trastear con la GUI para llevar el gripper a la posición "lobby". Y el tramo curvo es cuando ya ejecuté ese movimiento 

### Tiempo vs posición de las ruedas
Del mismo rosbag sacamos las posiciones de las ruedas transmitidas en el topic joint_states.

![image](https://github.com/user-attachments/assets/8038ee50-9467-4eba-9194-f6e5fd42b566)


Podemos ver como en el principio se aprecia muchos cambios, mientras que en la segunda mitad los valores se mantienen constantes. Esto coincide con el momento de acercarse al cubo y el momento en el que el rover se queda quieto mientras maniobra con el SCARA. Los saltos que irregulares que se ven en la gráfica se producen por la interacción de las ruedas del rover con el terreno irregular propio de las obras.

### Tiempo vs aceleración de las ruedas
Con el IMU del rover podemos sacar las mediciones inerciales del sistema completo.

![image](https://github.com/user-attachments/assets/7303cec8-47f2-40aa-b580-bf51629b5992)


Con esta gráfica podemos ver una serie de picos de aceleración causados en igual parte por mis habilidades de pilotaje y por la irregularidad del suelo. Comparando estos valores con la posición de las ruedas en el tiempo, vemos como la aceleración coincide con el momento en el que el rover see acerca a las cajas.

![image](https://github.com/user-attachments/assets/7a546e76-5ed1-4c31-bbc2-b58f73be5fd9)



