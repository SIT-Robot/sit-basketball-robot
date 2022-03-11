from urdf_parser_py.urdf import *


def sphere_inertial_matrix(m: float, r: float) -> Inertial:
    inertia_xyz = (2 / 5) * m * (r ** 2)
    return Inertial(
        mass=m,
        inertia=Inertia(
            ixx=inertia_xyz,
            iyy=inertia_xyz,
            izz=inertia_xyz, ))


def cylinder_inertial_matrix(m: float, r: float, h: float) -> Inertial:
    inertia_xy = m * (3 * r * r + h * h) / 12
    inertia_z = m * r * r / 2
    return Inertial(
        mass=m,
        inertia=Inertia(
            ixx=inertia_xy,
            iyy=inertia_xy,
            izz=inertia_z, ))


def box_inertial_matrix(m, x_length, y_length, z_length):
    return Inertial(
        mass=m,
        inertia=Inertia(
            ixx=m * (y_length ** 2 + z_length ** 2) / 12,
            iyy=m * (x_length ** 2 + z_length ** 2) / 12,
            izz=m * (x_length ** 2 + y_length ** 2) / 12, ))
