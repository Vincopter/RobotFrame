# Описание модели робота (RobotFrame)
Проект RobotFrame реализует модель малоразмерного робота (до 30 кг), кубической формы, который имеет два боковых 3-DOF манипулятора, со специализированным захватом (gripper), установленного на платформу-шасси с четырехколесным дифференциальным приводом (skid-steer). Также, в своем составе, в модели реализованы захваты (gripper), транспортный отсек, с люком, рассчитанный на использование боковыми манипуляторами, а также набор сенсоров (лидары, оптические и глубинные камеры)
Концепция применения робота близка к работе складского робота, который обладает возможностью перемещения малоразмерных грузов с автоматизированным процессом захвата и погрузки объекта (pick-and-place), перемещении на небольшие расстояния до пункта работы оператора, с последующей разгрузкой. 

**Модель RobotFrame**<br>
<img src="pictures/picture01.png" width="300"/>
<img src="pictures/picture02.png" width="330"/>
<img src="pictures/picture03.png" width="245"/>

Содержание:
- "[Состав и структура пакета](#PacketStructure)"
- "[Зависимости и сборка пакета](#BuildingPacket)"
- "[Визуализация модели в Rviz](#VisualRviz)"
- "[Запуск модели в симуляторе Gazebo](#SimGazebo)"
- "[Порядок использования Docker для запуска симуляции Gazebo](#DockerGazebo)"


<h2 id="PacketStructure">Состав и структура пакета</h2>
Пакет RobotFrame содержит описание модели робота в URDF-формате, launch-файлы запуска симуляций в различных режимах, 
командные скрипты запуска симуляций, а такде реализацию контроллеров, используемых в симуляции и для демонстрации.
<details><summary>Пакет имеет следующую файловую структуру (разверните для просмотра)</summary>
<p>

```
├── interactive/                     # jupyter-файлы, в которых приводятся расчеты некоторых параметров модели.
├── pictures/                        # служебный каталог для изображений и видео-файлов.
├── robot_frame/                     # основная директория описания модели и конфигураций.
    ├── configs/                     # директория для генерируемых конфигурационных фалов контроллеров и симуляции в Gazebo
        ├── controllers.yaml                 # [GENERATED] сводный файл конфигурации контроллеров ros2_control.
        ├── gazebo_params.yaml               # [GENERATED] общие установки для симулятора Gazebo.
        ├── joy_params.yaml                  # [GENERATED] общие параметры устройства joystick для драйвера.
        ├── joy_teleop_params.yaml           # [GENERATED] параметры управления джойстиком (скорости/кнопки) в симуляции (например, OpenTX Radiomaster TX12)
        ├── sunplus_camera_info.yaml         # [GENERATED] данные калибровки камер Sunplus web-cameras.
        ├── sunplus_param_camera_front.yaml  # [GENERATED] параметры камер марки Sunplus web-cameras для симулятора.
        └── teleop_multiplexer_params.yaml   # [GENERATED] параметры мультиплексора навигационных контроллеров (приоритеты/скорости обработки сообщений)
    ├── description/                 # описание модели робота в URDF-формате.
        ├── resources/               # директория для описания дополнительных материалов модели и mesh-файлов.
        ├── box.xacro                # описание сегментов шасси и кузова модели.
        ├── cameras.xacro            # описание сегментов и сочлинений для датчиков типа "Камера".
        ├── gazebo_controller.xacro  # файл описания дифференциального привода для ros_control.
        ├── inertial.xacro           # макросы расчета показателей инерции и центров масс.
        ├── lidars.xacro             # описание сегментов и сочлинений для сенсоров типа "Лидар".
        ├── manipulators.xacro       # описание сегментов манипулятора и захвата.
        ├── manipulator_joints.xacro # описание joint-соединений для манипуляторов и захватов.
        ├── materials.xacro          # файл описания материалов используемые в модели.
        ├── robot.urdf.xacro         # главный URDF-файл описания соединений всех сегментов модели.
        ├── ros2_controller.xacro    # описания для плагина gazebo_ros2_control симулятора Gazebo.
        ├── wheels.xacro             # описание сегментов колес привода Skid-steer.
        └── robot_description.xml    # [GENERATED] финальное описанием модели робота для построения в симуляторе.
    ├── launchers/                   
        ├── cameras.launch.py        # запуск подключения к модели до двух usb-камер (например, вебкамеры) для отображения изображений в симуляции. 
        ├── controllers.py           # модуль генерации состава и параметров контроллеров ros2_control в модели в Gazebo, по данным URDF-описания.
        ├── gazebo.launch.py         # запуск симуляции в 'Gazebo classic' - модуль является верхне-уровневым исполнителем для запуска симуляции в Gazebo.
        ├── joystick_tx12.launch.py  # запуск авто определения в системе наличия джойстика и подключение его к управлению модели, наравне с клавиатурой.
        ├── multiplexer.launch.py    # запуск мультиплексора, объединяющего несколько контроллеров управления, в один узел команд изменения в навигации.
        ├── robot.launch.py          # модуль построения модели по URDF и запуска узла Robot State Publisher для публикации данных для преобразования координат.
        ├── rviz.launch.py           # запуск визуализации в Rviz2 - модуль является верхне-уровневым исполнителем для отображения модели в Rviz.
        ├── stopall.py               # сервисный скрипт остановки всех процессов симуляции (узлов/приложений) запущенной через launch-файлы
        ├── teleop.launch.py         # файл запуска узла управления с клавиатуры (пакет teleop_twist_keyboard).
        └── utils.py                 # хелперный файл, для установки и описания разделяемых переменных и функций, используемые в launch-файлах
    ├── scripts/                     
        ├── docker/                  
            ├── sitl_prepare.sh      # скрипт для генерации Docker-контейнеров и окружения для контейнеризации. 
            └── sitl_start.sh        # скрипт запуска Docker-контейнера и симуляции с Gazebo.
        ├── install_environment.sh       # скрипт первичной установки фреймворка ROS2 (Humble) и необходимых пакетов на сервер.
        ├── run_gazebo_empty_world.sh    # запуск симулятора Gazebo и мира без модели RobotFrame.
        ├── run_gazebo_simulation.sh     # запуск полной симуляции модели RobotFrame в Gazebo.
        ├── run_rviz.sh                  # запуск визуализации модели в Rviz.
        └── stopall.sh                   # служебный скрипт останова процессов симуляции на сервере.
    ├── sources/                     
        ├── demonstration.py         # модуль запуска демонстрации работы манипуляторов и захватов.
        ├── enumcam.py               # модуль для проверки конфигурирования камер для последующего использования.
        └── obstacle_controller.py   # контроллер коррекции маршрута и обхода препятствий.
├── rviz/                            # каталог содержания окружения для визуализации в Rviz2.
├── worlds/                          # каталог содержания окружения для симулятора Gazebo.
├── CMakeLists.txt                   # cmake-файл конфигурации системы сборки пакета утилитой colcon в ROS2.
├── package.xml                      # информация о пакете и зависимости пакета.
└── setup.py                         # python-скрипт для конфигурирования системы сборки для дистрибуции пакета.

```
</p>
</details>

<a name="BuildingPacket"></a> 
## Зависимости и сборка пакета
Модель разработана для использования в контексте платформы и программного фреймворка __ROS2 (Humble)__, под управлением ОС Ubuntu. Для сборки и конфигурирования пакета в ROS2 используется *colcon*.<br>
Поддержка работы на операционных системах ниже требуемой (Ubuntu 22.04) для фреймворка ROS2, достигается с помощью контейнеризации в Docker в случае запуска симуляции. Описание смотрите в рункте [Порядок использования Docker для запуска симуляции Gazebo](#DockerGazebo)<br><br>

Для построения и работы модели используется как стандартные пакеты, так и ряд дополнительных пакетов для фреймворка ROS2:
- Стандартный пакет ros-dev-tools, включающий утилиты для разработки, в частности компилятор colcon (https://github.com/ros-infrastructure/infra-variants).
- Пакет ros-humble-xacro, интерпретатор Xacro для ROS2 Humble (https://github.com/ros/xacro).
- DDS-сервер Cyclone (пакет ros-humble-rmw-cyclonedds-cpp), для непосредственного функционирования ROS2 Humble.
- Пакет gazebo-ros-pkgs, API и фреймворк для использование ROS в симуляции Gazebo (https://github.com/ros-simulation/gazebo_ros_pkgs).
- Стандартный пакет joystick, драйвер работы джойстика в Ubuntu.
- Пакет ros-humble-usb-cam, драйвер для USB-устройств V4L Camera (https://github.com/ros-drivers/usb_cam.git)
- Стандартный пакет ROS joint-state-publisher, включающий утилиты установки и публикации значений joint-соединений (https://github.com/ros/joint_state_publisher.git)
- Пакет controller_manager, компонент построения и управления моделью во фреймворке ROS2,  в частности, в случае заданной в URDF-формате (https://github.com/ros-controls/ros2_control.git).
- Пакет унифицированных контроллеров, содержащий шаблоны разнотипных контроллеров применяемых в ROS2 (https://github.com/ros-controls/ros2_controllers)
- Служебный пакет gazebo-ros2-control, инкапсулирующий использование контроллеров ROS2 в симуляторе Gazebo (https://github.com/ros-controls/gazebo_ros2_control).
- Отдельный пакет управления манипулятором вида «Захват» (gripper-controllers), входящий в пакет общих контроллеров ROS2 Controllers.
- Пакет мультиплексора сообщений Twist, регулирующих скорость (twist-mux), для объединения нескольких контроллеров управления (например, клавиатура, джойстик).
- Контроллер коррекции маршрута и обхода препятствий, реализованный как модуль в составе модели RobotFrame (obstacle_controller).

Приложения для симуляции и визализации модули:<br>
- Cимулятор Gazebo (https://github.com/gazebosim/gazebo-classic).
- Rviz (https://github.com/ros2/rviz).
- Rqt (https://github.com/ros-visualization/rqt)

Для сборки пакета необходимо иметь среду и настроенное окружение разработчика ROS2.
<details><summary>Для установки окружения и инструментов разработчика ROS (разверните для просмотра)</summary>
<p>

```
$ sudo apt update && sudo apt install -y build-essential cmake git python3-pip python-rosdep python3-vcstool wget python3-colcon-common-extensions
$ python3 -m pip install -U argcomplete flake8 flake8-blind-except flake8-builtins pytest-repeat pytestpytest-runner setuptools
$ sudo apt install --no-install-recommends -y libopencv-dev libasio-dev libtinyxml2-dev libqt5gui5
```
</p>
</details><br>
При необходимости сборки проекта локально на сервере под управлением ОС Ubuntu 22.04 и запуска симуляции рекомендуется предварительно выполнить установку ROS2 
и настройку окружения с помощью скрипта [install_environment.sh](robot_frame/scripts/install_environment.sh). После выполнения скрипта будет доступна вся необходимая среда для запуска симуляции в Gazebo. Далее, следует клонировать проект RobotFrame на локальный сервер и собрать пакет
```
colcon build --symlink-install --packages-select robot_frame
```

<a name="VisualRviz"></a> 
## Визуализация модели в Rviz
Для запуска визуализации для модели RobotFrame, в программе Rviz2, рекомендуется выполнить скрипт [run_rviz.sh](robot_frame/scripts/run_rviz.sh), который запускает процесс с помощью launch-файла [rviz.launch.py](robot_frame/launchers/rviz.launch.py). Запущенная в Rviz среда будет уже настроена на отображение модели и данных с топиков работающих с сенсорами (лидары, камеры, одометрия, координаты). Также, в окружении будет запущена и доступна программа «Joint State Publisher», в которой можно управлять положением соединений робота (joints).<br>
*Пример отображения модели в программе Rviz2*
<details><summary>разверните для просмотра</summary>
    <img src="pictures/picture04.png"/>
</details>

<a name="SimGazebo"></a> 
## Запуск модели в симуляторе Gazebo
При запуске визуализации

<a name="DockerGazebo"></a> 
## Порядок использования Docker для запуска симуляции Gazebo

hhhh




[Здесь подробное объяснение](#detailed-explanation)
<p id="detailed-explanation">Обратите внимание, это важно.</p>
<!-- Здесь вы найдёте зёрнышки мудрости! 🦉 -->
