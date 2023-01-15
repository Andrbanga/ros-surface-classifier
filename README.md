# ros-surface-classifier
ROS пакет для сегментации вертикальных и горизонтальных плоскостей из облака точек с использованием библиотеки PCL.

## Параметры

- vertical_model_max_iterations - максимальное число итераций работы RANSAC для сегментации вертикальных плоскостей
- horizontal_model_max_iterations - максимальное число итераций работы RANSAC для сегментации горизонтальных плоскостей
- vertical_model_angle_eps - угол отклоения для вертикальной плоскости (в градусах)
- horizontal_model_angle_eps - угол отклонения для горизонтальной плоскости (в градусах)
- vertical_model_distance_threshold - порог отклонения точек от искомой вертикальной плоскости
- horizontal_model_distance_threshold - порог отклонения точек от искомой горизонтальной плоскости


## Запуск

Для тестового запуска выполните 

```
roslaunch ros-surface-classifier segment_planes.launch
```
и запустите тестовый bag файл