""" This module obviously contains the EllipsoidGenerator class.

In the future, maybe this will contain other generators as well. Or maybe I'll keep it to one generator per file. I haven't decided yet.
"""

import numpy as np

class EllipsoidGenerator:
    """ Used to generate a half-ellipsoid mesh.

    Upon construction, a sphere of vertices is created. Then the sphere vertices are streched into an ellipsoid and the faces are created.
    num_pts is the number of points in each vertex row an column. So there are num_pts^2 vertices and 2*(num_pts-1)^2 faces
    TODO: Make configurable to generate half ellipsoid, full ellipsoid, etc.
    """
    a = 1
    b = 1
    c = 1
    num_pts = 40
    rotation = 2*np.pi

    x_sphere = None
    y_sphere = None
    z_sphere = None
    vertices_sphere = None
    vertices = None
    faces = None


    def __init__(self, a, b, c, num_pts = 20):
        self.a = a
        self.b = b
        self.c = c
        self.num_pts = num_pts
        # Only generating half at this point
        self.rotation = np.pi

        self.generate_sphere()

    def generate_sphere(self):
        """Generate a unit sphere with desired sampling """
        self.z_sphere = np.linspace(-1, 1, self.num_pts+2)[1:-1]
        r = np.sqrt(1.0 - self.z_sphere**2)[:,None]
        theta = np.linspace(0, self.rotation, self.num_pts+1)[:-1][None,:]
        self.x_sphere = r*np.cos(theta)
        self.y_sphere = r*np.sin(theta)


    def generate_ellipsoid(self):
        """Generate ellipsoid from sphere using the stored a,b,c values

        TODO: Put in center vertex/faces if needed
        """

        # Store sphere vertices as x, y, z for convenience
        x = self.x_sphere
        y = self.y_sphere
        z = self.z_sphere

        # Form vertices. This is really just setting their location in the array. They are still a sphere
        vertices_sphere = np.empty([self.num_pts*self.num_pts,3])
        for i in range(self.num_pts):
            vertices_sphere[i*self.num_pts:(i+1)*self.num_pts,:] = np.array([x[:,i], y[:,i], z]).T
        self.vertices_sphere = vertices_sphere

        # Form the faces
        num_faces = (self.num_pts-1)**2*2
        faces = np.empty([num_faces,3])
        ind = 0
        # loop over z values
        for i in range(self.num_pts-1):
            # loop over xy pairs
            for j in range(self.num_pts-1):
                # bottom right triangle
                v1 = i*self.num_pts + j
                v2 = (i+1)*self.num_pts + j
                v3 = (i+1)*self.num_pts + j + 1
                faces[ind] = np.array([v1,v2,v3])

                # upper left triangle
                v1 = i*self.num_pts + j
                v2 = i*self.num_pts + j + 1
                v3 = (i+1)*self.num_pts + j + 1
                faces[ind+1] = np.array([v3,v2,v1])
                ind += 2
        self.faces = faces

        # Stretch the sphere into an ellipsoid
        self.update_ellipsoid(self.a, self.b, self.c)

    def update_ellipsoid(self, a, b, c):
        self.vertices = np.matmul(self.vertices_sphere, np.array([[a, 0, 0], [0, b, 0], [0, 0, c]]))







