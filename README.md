# Описание модели робота (RobotFrame)

Проект RobotFrame реализует модель малоразмерного робота (до 30 кг), кубической формы, который имеет два боковых 3-DOF манипулятора, со специализированным захватом (gripper), установленного на платформу-шасси с четырехколесным дифференциальным приводом (skid-steer). В своем составе, модель имеет захваты (gripper) на 3-DOF манипуляторе, а также транспортный отсек, с люком, рассчитанный на использование боковыми манипуляторами. 
Концепция применения робота близка к работе складского робота, который обладает возможностью перемещения малоразмерных грузов с автоматизированным процессом захвата и погрузки объекта (pick-and-place), перемещении на небольшие расстояния до пункта работы оператора, с последующей разгрузкой. 

![Модель RobotFrame](pictures/picture01.png)

<details><summary>CLICK here to see the details of the xacro file contents</summary>
<p>

```
<?xml version="1.0"?>
```

</p>
</details>

___
**Notice** that the model doesn't have any tags that make possible to interface its links and joints with Gazebo. Those elements are usually included in `<gazebo>` tag and they will come in the next step.

