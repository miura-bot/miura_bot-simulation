# Find inertial origin
The miura base link (mori-design-2.stl) has the origin at the edge that is offset [-70.9012, -112.5, -6.0] from the center of mass. From the CoM to the shaft is -64.9519. We can thus first offset the base link by [-5.9493 0.0 -6.0], and has the inertial origin w.r.t to base link coordinate of [64.9519 112.5 0.0]

In the case of 2-cell miura pattern, as we rotate the base link coordinate by 60 degree, the following is used to offset the inertial origin for the second base link w.r.t the rotational transformation

```python
import math
import numpy as np

def rad(degree):
    return (degree / 180.0) * math.pi

rot_mat = np.array([[math.cos(rad(60), -math.sin(rad(60)))],
                    [math.cos(rad(60), math.sin(rad(60)))]])

pre_rotate_coor = np.array([[64.9519],
                            [112.5]])

post_rotate_coor = rot_mat @ pre_rotate_coor
```

Then we can put a revolute joint at the edge of the two unit cells.

**Note**: When apply rpy transformation to the link, the xyz will have to follow. For example, in the case of the first link, we set the xyz offset to [-5.9493 0.0 -6.0], but in the second link, because there is a 60 degree rotation of z axis, T = [[cos(60), -sin(60)], [cos(60), sin(60)]], we need to apply the same transformation to the resulting xyz offset. resulting in new xyz offset of T @ [-5.9493, 0.0] -> [-2.97465, -2.97465] (omitting Z for simplicity).