# You can install with sudo pip install sympy
import sympy as sp


def main():
    d1 = sp.Symbol("d_1")
    a2 = sp.Symbol("a_2")
    a3 = sp.Symbol("a_3")
    d4 = sp.Symbol("d_4")
    d5 = sp.Symbol("d_5")
    d6 = sp.Symbol("d_6")

    th1 = sp.Symbol("\\theta_1")
    th2 = sp.Symbol("\\theta_2")
    th3 = sp.Symbol("\\theta_3")
    th4 = sp.Symbol("\\theta_4")
    th5 = sp.Symbol("\\theta_5")
    th6 = sp.Symbol("\\theta_6")

    B01 = make_DH(th1, d1, 0, 0)


# See Theory of Applied Robotics pg. 242
# Uses the Denavit-Hartenberg method of reference frames
def make_DH(theta, d, a, alpha):
    # First, rotate by alpha around the x-axis
    R_x = sp.Matrix([[1, 0, 0, 0],
                     [0, sp.cos(alpha), sp.sin(alpha), 0],
                     [0, -sp.sin(alpha), sp.cos(alpha), 0],
                     [0, 0, 0, 1]])
    # Then travel by a along the x-axis
    D_x = sp.Matrix([[1, 0, 0, a],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])
    # Then rotate by theta around the new z-axis
    R_z = sp.Matrix([[sp.cos(theta), sp.sin(theta), 0, 0],
                     [-sp.sin(theta), sp.cos(theta), 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])
    # Then travel by d along the z-axis
    D_z = sp.Matrix([[1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, d],
                     [0, 0, 0, 1]])
    return D_z * R_z * D_x * R_x


if __name__ == '__main__':
    main()

